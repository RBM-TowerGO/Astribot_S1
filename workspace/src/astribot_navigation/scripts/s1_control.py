#!/usr/bin/env python3
"""S1 supervisor + Home CLI. Default: read-only. No SDK import before preflight."""
import argparse
import builtins
from contextlib import contextmanager
import fcntl
import json
import math
import os
from pathlib import Path
import sys
import time

CONTROLLER = '/astribot_chassis_astribot_head_astribot_torso_astribot_arm_left_astribot_arm_right_astribot_gripper_left_astribot_gripper_right'
HARDWARE = '/astribot_hardware_node'
HOME = {
    'astribot_torso': [0.29, -0.58, 0.29, 0.0],
    'astribot_arm_left': [0.159, -0.022, -1.42, 1.66, -0.345, 0.115, 0.125],
    'astribot_gripper_left': [0.0],
    'astribot_arm_right': [-0.159, -0.022, 1.42, 1.66, 0.345, 0.115, -0.125],
    'astribot_gripper_right': [0.0],
    'astribot_head': [0.0, 0.0],
}
DOFS = {name: len(q) for name, q in HOME.items()}
DOFS['astribot_chassis'] = 3


class ControlError(RuntimeError):
    pass


def log(message):
    print(message, flush=True)


def finite_vector(values, length):
    try:
        return len(values) == length and all(math.isfinite(v) for v in values)
    except (TypeError, ValueError):
        return False


def home_errors(positions):
    errors = {}
    for name, target in HOME.items():
        actual = positions.get(name, [])
        if not finite_vector(actual, len(target)):
            errors[name] = '反馈缺失或无效'
            continue
        error = max(abs(a-b) for a, b in zip(actual, target))
        tolerance = 3.0 if 'gripper' in name else 0.05
        if error > tolerance:
            errors[name] = round(error, 5)
    return errors


def bootstrap():
    """Only local setup scripts; never source robot_env.sh (sudo/sysctl/writes)."""
    if os.environ.get('TOWERGO_S1_BOOTSTRAPPED') == '1':
        if os.environ.get('ROS_DOMAIN_ID') != '25':
            raise ControlError('需要 ROS_DOMAIN_ID=25')
        return
    setup = Path('/opt/astribot_ros/middle_ware/local_setup.bash')
    if not setup.is_file():
        raise ControlError('请在 S1 上运行此脚本；本机只支持 --help 和纯逻辑测试。')
    # Extra paths cover the SDK without running env.sh, which rewrites DDS XML.
    shell = r'''
source /opt/astribot_ros/middle_ware/local_setup.bash || exit 2
source /opt/astribot_ros/software/local_setup.bash || exit 2
if [ -f /opt/astribot_ros/third_pkg/local_setup.bash ]; then
  source /opt/astribot_ros/third_pkg/local_setup.bash || exit 2
fi
SDK_DIR=/home/astribot/astribot_sdk_aarch64
export PYTHONPATH="$SDK_DIR/third_party/astribot_ros_middleware_py:$SDK_DIR:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$SDK_DIR/astribot_sdk/core/common/robotics_library_py:$SDK_DIR/astribot_sdk/core/common/whole_body_control/third_party:$SDK_DIR/third_party/drake/lib:$SDK_DIR/third_party/third_pkg:/opt/drake/lib:${LD_LIBRARY_PATH:-}"
export ROBOT_TYPE=S1 ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml
export TOWERGO_S1_BOOTSTRAPPED=1
exec /usr/bin/python3 "$@"
'''
    os.execv('/bin/bash', ['bash', '-c', shell, 's1-control', str(Path(__file__).resolve()), *sys.argv[1:]])


class Observer:
    """Dedicated ROS context: status queries and subscriptions only."""
    def __init__(self, timeout):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from astribot_msgs.msg import RobotJointState
        from rcl_interfaces.msg import Log
        self.ros = rclpy
        self.timeout = timeout
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('towergo_s1_control_observer', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.samples = {}
        self.supervisor_logs = []
        self.subscriptions = []
        def receive_log(message):
            if message.name == 'system_supervisor_node':
                self.supervisor_logs.append(message.msg)
                self.supervisor_logs = self.supervisor_logs[-6:]
        self.subscriptions.append(self.node.create_subscription(
            Log, '/rosout', receive_log, qos_profile_sensor_data))
        for name in DOFS:
            def receive(message, part=name):
                self.samples[part] = (time.monotonic(), list(message.position))
            self.subscriptions.append(self.node.create_subscription(
                RobotJointState, '/' + name + '/joint_space_states',
                receive, qos_profile_sensor_data))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=min(0.1, max(0.0, end-time.monotonic())))

    def request(self, service, srv_type, operation=False):
        client = self.node.create_client(srv_type, service)
        try:
            if not client.wait_for_service(timeout_sec=self.timeout):
                raise ControlError(f'未发现服务：{service}；没有发送请求。')
            future = client.call_async(srv_type.Request())
            end = time.monotonic() + self.timeout
            while not future.done() and time.monotonic() < end:
                self.executor.spin_once(timeout_sec=0.1)
            if not future.done():
                future.cancel()
                detail = ('操作请求执行结果未知；不要自动重试，请先检查状态。' if operation
                          else '这是只读查询，没有发送配置、激活或运动命令。')
                raise ControlError(f'服务响应超时：{service}。{detail}')
            result = future.result()
            if result is None:
                raise ControlError(f'服务没有返回结果：{service}')
            return result
        finally:
            self.node.destroy_client(client)

    def state(self, name=CONTROLLER):
        from lifecycle_msgs.srv import GetState
        state = self.request(name + '/get_state', GetState).current_state
        return state.id, state.label

    def trigger(self, action):
        from std_srvs.srv import Trigger
        service = '/supervisor/' + action + '_system'
        log('请求：' + service)
        result = self.request(service, Trigger, operation=True)
        log(f'返回 success={result.success}, message={result.message}')
        if not result.success:
            raise ControlError(f'{action} 失败：{result.message}')

    def positions(self, since=0.0):
        now = time.monotonic()
        return {name: q for name, (stamp, q) in self.samples.items()
                if stamp > since and 0 <= now-stamp <= 0.5 and finite_vector(q, DOFS[name])}

    def wait_feedback(self, timeout=None):
        end = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < end:
            self.spin(0.1)
            if set(self.positions()) == set(DOFS):
                return
        missing = sorted(set(DOFS)-set(self.positions()))
        raise ControlError('未收到全部有效、近期关节状态：' + ', '.join(missing))

    def has_owner(self):
        self.spin(1.0)
        return any(name == '/astribot/control_rights'
                   for name, types in self.node.get_service_names_and_types())

    def diagnostics(self):
        from rcl_interfaces.srv import GetParameters
        result = {'joystick_device_exists': Path('/dev/input/js0').exists()}
        try:
            state, label = self.state('/joy_node')
            result['joy_node'] = {'id': state, 'label': label}
        except ControlError as exc:
            result['joy_node'] = {'error': str(exc)}
        try:
            # Empty GetParameters request is a read-only connectivity check.
            self.request('/system_supervisor_node/get_parameters', GetParameters)
            result['supervisor_parameter_service_responding'] = True
        except ControlError as exc:
            result['supervisor_parameter_service_responding'] = False
            result['supervisor_error'] = str(exc)
        result['recent_supervisor_logs'] = list(self.supervisor_logs)
        if result['joy_node'].get('id') == 1 and not result['joystick_device_exists']:
            result['hint'] = ('joy_node 尚未配置，且 /dev/input/js0 不存在。'
                              '检查手柄/接收器；监督节点可能正在重试手柄配置。'
                              '不要通过跳过状态检查来运行 Home。')
        return result

    def verify_home(self):
        # Drain bounded subscription queues before accepting post-motion samples.
        self.spin(0.5)
        since = time.monotonic()
        end = since + self.timeout
        stable = 0
        errors = {'feedback': '尚无动作后的反馈'}
        last_stamps = {}
        while time.monotonic() < end:
            self.spin(0.15)
            errors = home_errors(self.positions(since=since))
            stamps = {name: self.samples.get(name, (0, []))[0] for name in HOME}
            fresh_round = all(stamps[name] > last_stamps.get(name, since) for name in HOME)
            stable = stable + 1 if not errors and fresh_round else 0
            last_stamps = stamps
            if stable >= 3:
                return
        raise ControlError('Home 到位未确认：' + json.dumps(errors, ensure_ascii=False))

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.ros.shutdown(context=self.context)


def ensure_active(observer):
    state, label = observer.state()
    log(f'控制驱动状态：{label} ({state})')
    if state == 2:  # lifecycle inactive
        observer.trigger('activate')
        state, label = observer.state()
    if state != 3:
        if state == 1:
            raise ControlError('系统尚未配置。先运行 configure，确认成功后再运行 home。')
        raise ControlError(f'控制驱动未激活：{label} ({state})；不进入 SDK。')
    hardware_state, label = observer.state(HARDWARE)
    if hardware_state != 3:
        raise ControlError(f'硬件节点未激活：{label} ({hardware_state})')
    observer.wait_feedback()


def perform_home(observer, factory=None):
    ensure_active(observer)
    if observer.has_owner():
        raise ControlError('检测到其他 SDK 控制者；本脚本不强制抢占，请先正常退出原控制程序。')
    if factory is None:
        from astribot_sdk.core.astribot_api.astribot_client import Astribot
        factory = Astribot
    robot = None
    moving = False
    try:
        # SDK catches EOFError and may claim rights. Return an empty answer to
        # refuse takeover even if another owner appears after the graph check.
        original_input = builtins.input
        try:
            builtins.input = lambda *args: ''
            robot = factory(high_control_rights=False, node_name='towergo_home')
        finally:
            builtins.input = original_input
        if not robot.get_control_rights_status():
            raise ControlError('SDK 未取得控制权；取消 Home。')
        observer.wait_feedback()
        log('执行 Home：躯干、双臂、头部回位，左右夹爪完全张开。')
        moving = True
        result = robot.move_to_home()
        log('SDK Home 返回：' + repr(result))
        if result is False or (isinstance(result, str) and any(
                word in result.lower() for word in ('fail', 'error', 'abort', 'cancel'))):
            raise ControlError('SDK 报告 Home 失败。')
        observer.verify_home()
        moving = False
        log('Home 已由动作后的关节反馈确认到位。')
    except BaseException:
        if moving and robot is not None and robot.get_control_rights_status():
            try:
                robot.stop_robot()
                log('异常退出：已请求 SDK 整机停止；这不是物理急停。')
            except Exception as exc:
                log('SDK 停止请求失败：' + str(exc))
        raise
    finally:
        if robot is not None:
            try:
                robot.astribot_interface.shutdown()
            except Exception as exc:
                log('SDK 清理提示：' + str(exc))


def run(action, observer):
    if action == 'status':
        report = {}
        for label, node in [('controller', CONTROLLER), ('hardware', HARDWARE)]:
            try:
                state, name = observer.state(node)
                report[label] = {'id': state, 'label': name}
            except ControlError as exc:
                report[label] = {'error': str(exc)}
        observer.spin(1.0)
        report['fresh_joint_feedback'] = sorted(observer.positions())
        report['sdk_control_owner_discovered'] = observer.has_owner()
        report['diagnostics'] = observer.diagnostics()
        log(json.dumps(report, ensure_ascii=False, indent=2))
        return 2 if any('error' in report[k] for k in ('controller', 'hardware')) else 0
    if action == 'home':
        perform_home(observer)
    elif action == 'activate':
        ensure_active(observer)
        log('驱动已激活，全部关节状态有效。未执行 Home。')
    else:
        state, label = observer.state()
        expected = 1 if action == 'configure' else 3
        target = 2
        if state == target:
            log(f'当前为 {label}，无需 {action}。')
            return 0
        if state != expected:
            raise ControlError(f'当前状态 {label} ({state}) 不适合 {action}。')
        observer.trigger(action)
        state, label = observer.state()
        if state != target:
            raise ControlError(f'{action} 后状态未确认：{label} ({state})')
        log(f'{action} 完成，控制驱动状态：{label}。')
    return 0


@contextmanager
def action_lock(action):
    if action == 'status':
        yield
        return
    path = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp')) / f'towergo-s1-control-{os.getuid()}.lock'
    with path.open('a') as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ControlError('另一个 s1_control 操作仍在执行。') from exc
        yield


def main():
    parser = argparse.ArgumentParser(description='S1 命令行控制；默认 status 只读，不申请控制权。')
    parser.add_argument('action', nargs='?', default='status',
                        choices=['status', 'configure', 'activate', 'home', 'deactivate'])
    parser.add_argument('--timeout', type=float, default=8.0, help='每次服务/反馈等待秒数，默认 8')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 60:
        parser.error('--timeout 必须在 (0, 60] 内')
    observer = None
    try:
        bootstrap()
        observer = Observer(args.timeout)
        with action_lock(args.action):
            return run(args.action, observer)
    except KeyboardInterrupt:
        log('已中断；激活请求若已发出，其结果应重新查询。')
        return 130
    except Exception as exc:
        log('失败：' + str(exc))
        return 2
    finally:
        if observer is not None:
            observer.close()


if __name__ == '__main__':
    sys.exit(main())
