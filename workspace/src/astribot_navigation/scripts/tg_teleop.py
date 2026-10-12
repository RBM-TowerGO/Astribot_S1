#!/usr/bin/env python3
"""Standalone S1 terminal teleop. Importing this module never loads the SDK."""
import argparse
import fcntl
import math
import os
from pathlib import Path
import select
import signal
import struct
import sys
import termios
import threading
import time
import tty
import unicodedata

from tg_pose import HOLD, NAMES, PoseControl, load_pack
from tg_torso import HeightLimit, TorsoHeight, DEFAULT_SPEED as DEFAULT_TORSO_SPEED, MAX_SPEED as MAX_TORSO_SPEED, vector

HZ = 100.0
MAX_LINEAR_SPEED = 1.0
LEASE = 0.2
MAX_CYCLE = 0.1


def fit_status(text, columns):
    """Keep redraws on one terminal row, including double-width Chinese text."""
    remaining = max(0, columns - 1)
    result = []
    for char in text:
        width = 0 if unicodedata.combining(char) else (2 if unicodedata.east_asian_width(char) in 'WF' else 1)
        if width > remaining:
            break
        result.append(char)
        remaining -= width
    return ''.join(result)


class SessionHealth:
    """Optional inherited pipe, containing sender monotonic timestamps."""
    def __init__(self, fd=None):
        self.fd, self.buffer, self.latest = fd, b'', None
        if fd is not None:
            os.set_blocking(fd, False)

    def check(self, now):
        if self.fd is None:
            return
        while True:
            try:
                data = os.read(self.fd, 4096)
            except BlockingIOError:
                break
            if not data:
                raise RuntimeError('建图会话健康连接关闭；退出遥操')
            self.buffer += data
            while len(self.buffer) >= 8:
                self.latest = struct.unpack('!d', self.buffer[:8])[0]
                self.buffer = self.buffer[8:]
        # A heartbeat may arrive between taking 'now' and reading the pipe.
        if self.latest is None or not math.isfinite(self.latest) or not -.02 <= now-self.latest <= .5:
            raise RuntimeError('建图会话心跳超过 0.5 秒未更新；退出遥操')

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def speed(value, low, high):
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise argparse.ArgumentTypeError('速度必须在 %g–%g 之间' % (low, high))
    return value


def position(value):
    if len(value) != 3 or not all(math.isfinite(float(x)) for x in value):
        raise RuntimeError('底盘位置必须是有限的 [x, y, yaw]')
    return [float(x) for x in value]


class Keys:
    def __init__(self, linear=.05, angular=.10):
        self.linear, self.angular = linear, angular
        self.key, self.until, self.quit = '', 0., False
        self.message = '等待新按键'
        self.hold_since, self.hold_batches, self.last_input = 0., 0, 0.

    def stop(self, message):
        self.key, self.until, self.message = '', 0., message
        self.hold_since, self.hold_batches = 0., 0

    def feed(self, data, now):
        keys = data.lower()
        self.last_input = now
        if any(k in keys for k in ('x', '\x03', '\x04')):
            self.quit = True
            self.stop('退出')
            return
        if not keys or any(k not in 'wsadqerfhp+-=' for k in keys):
            self.stop('保持目标')
            return
        pose_keys = set(keys) & set('hp')
        if pose_keys:
            # Reject mixed pose/motion/speed batches; buffered text is not a hold.
            if len(set(keys)) != 1:
                self.stop('混合姿态输入：停止')
                return
            k = keys[0]
            if self.key != k or now >= self.until:
                self.hold_since, self.hold_batches = now, 0
            self.hold_batches += 1
            self.key, self.until = k, now + LEASE
            self.message = '长按初始姿态' if k == 'h' else '长按打包姿态'
            return
        # Speed changes do not renew the movement lease.
        for k in keys:
            if k in '+-=':
                if self.key in ('h', 'p'):
                    self.stop('调速：停止姿态')
                sign = -1 if k == '-' else 1
                self.linear = round(max(.01, min(MAX_LINEAR_SPEED, self.linear + sign * .01)), 5)
                self.angular = round(max(.025, min(.25, self.angular + sign * .025)), 5)
            else:
                self.key, self.until = k, now + LEASE
                self.message = {'w': '前进', 's': '后退', 'a': '左移', 'd': '右移', 'q': '左转', 'e': '右转', 'r': '腰部升高', 'f': '腰部降低'}[k]

    def sample(self, now):
        if self.key and now >= self.until:
            self.stop('输入超时：保持目标')
        return {'w': (self.linear, 0., 0.), 's': (-self.linear, 0., 0.),
                'a': (0., self.linear, 0.), 'd': (0., -self.linear, 0.),
                'q': (0., 0., self.angular), 'e': (0., 0., -self.angular)}.get(self.key, (0., 0., 0.))

    def pose(self, now):
        self.sample(now)
        selected = {'h': 'home', 'p': 'pack'}.get(self.key)
        confirmed = selected if self.hold_batches >= 3 and now - self.hold_since >= HOLD else None
        return confirmed, selected


class Driver:
    def __init__(self, backend, now):
        self.backend = backend
        self.target = position(backend.desired())
        self.last = now
        self.failed = False
        self.sent = False

    def check_cycle(self, now):
        if self.failed:
            raise RuntimeError('控制已故障，不能自动恢复')
        try:
            if not 0 < now - self.last <= MAX_CYCLE:
                raise RuntimeError('控制循环停顿超过 0.1 秒；退出，不补发动作')
            self.last = now
            if not self.backend.has_control():
                raise RuntimeError('控制权丢失；停止发送')
            self.backend.check()
        except Exception:
            self.failed = True
            raise

    def apply_velocity(self, velocity):
        if self.failed:
            raise RuntimeError('控制已故障，不能自动恢复')
        try:
            vx, vy, w = velocity
            if not all(math.isfinite(x) for x in (vx, vy, w)) or math.hypot(vx, vy) > MAX_LINEAR_SPEED or abs(w) > .25:
                raise RuntimeError('速度无效或超过限位')
            if vx or vy or w:
                # Vendor 202 uses accumulated LOCAL x/y/yaw, without rotation.
                self.target[0] += vx / HZ
                self.target[1] += vy / HZ
                self.target[2] += w / HZ
                self.sent = True
            if self.sent:
                self.backend.send(list(self.target))
        except Exception:
            self.failed = True
            raise

    def tick(self, now, velocity):
        self.check_cycle(now)
        self.apply_velocity(velocity)


def conflicts(proc=Path('/proc'), include_teleop=True):
    """Best-effort process detection; a lock separately guards this tool."""
    found = []
    for p in proc.iterdir():
        if not p.name.isdigit() or int(p.name) == os.getpid():
            continue
        try:
            argv = (p / 'cmdline').read_bytes().decode(errors='replace').split('\0')
        except (OSError, ProcessLookupError):
            continue
        names = [Path(a).name for a in argv if a]
        direct = bool(set(names) & {'tg-chassis.py', '202-chassis_joy_control_local.py',
                                    '203-chassis_joy_control_global.py'})
        direct = direct or any(a.endswith('/towergo_real/chassis') for a in argv)
        teleop = include_teleop and 'tg_teleop.py' in names and '--dry-run' not in argv
        ros = any(argv[i:i+3] == ['run', 'towergo_real', 'chassis'] for i in range(len(argv)))
        module = any(argv[i:i+2] == ['-m', 'towergo_real.chassis'] for i in range(len(argv)))
        if direct or ros or module or teleop:
            found.append(p.name)
    return found


class SessionLock:
    def __init__(self):
        # Share the existing ROS chassis backend's lock name.
        base = Path(os.environ.get('XDG_RUNTIME_DIR') or '/tmp')
        self.path = base / ('towergo-chassis-%d.lock' % os.getuid())
        self.file = None

    def __enter__(self):
        self.file = self.path.open('a')
        try:
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.file.close()
            self.file = None
            raise RuntimeError('已有底盘控制实例持有锁，拒绝启动') from None
        return self

    def __exit__(self, *args):
        if self.file:
            self.file.close()


def configure_terminal(fd):
    tty.setcbreak(fd)
    attrs = termios.tcgetattr(fd)
    # cbreak retains XON/XOFF: Ctrl+S can otherwise freeze status output and
    # block the control loop. Treat these bytes as unknown stop keys instead.
    attrs[0] &= ~(termios.IXON | termios.IXOFF | getattr(termios, 'IXANY', 0))
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)


class FakeBackend:
    def __init__(self):
        self.p = [0., 0., 0.]
        self.torso_q = [0., 0., 0., 0.]

    def torso_state(self):
        return [0., 0., 1.2 + self.torso_q[0], 0., 0., 0., 1.], list(self.torso_q)

    def torso_reference(self):
        return self.torso_state()

    def solve_torso(self, target):
        return [target[2] - 1.2, 0., 0., 0.]

    def check_torso(self, joints):
        pass

    def send_torso(self, joints):
        self.torso_q = list(joints)

    def pose_available(self, name):
        return True  # Synthetic dry-run only; no real pack values.

    def prepare_pose(self):
        pass

    def execute_pose(self, name, cancel):
        cancel.wait(1.)

    def stop_pose(self):
        pass

    def desired(self):
        return list(self.p)

    def has_control(self):
        return True

    def check(self):
        pass

    def send(self, p):
        self.p = list(p)

    def close(self):
        pass


class SDKBackend:
    def __init__(self, pack=None):
        self.pack = pack
        from astribot_sdk.core.astribot_api.astribot_client import Astribot
        import astribot_ros_middleware as middleware
        self.bot, self.thread = None, None
        self.done = threading.Event()
        self.error = None
        try:
            self.bot = Astribot(freq=HZ, high_control_rights=True)
            if not self.has_control():
                raise RuntimeError('未获得底盘控制权')
            def spin():
                try:
                    while not self.done.is_set():
                        middleware.spin()
                except Exception as exc:
                    self.error = exc
            self.thread = threading.Thread(target=spin, daemon=True)
            self.thread.start()
            position(self.bot.get_current_joints_position([self.bot.chassis_name])[0])
            position(self.desired())
        except BaseException:
            self.close()
            raise

    def desired(self):
        return self.bot.get_desired_joints_position([self.bot.chassis_name])[0]

    def has_control(self):
        return bool(self.bot and self.bot.get_control_rights_status())

    def check(self):
        if self.error is not None:
            raise RuntimeError('SDK 通信线程异常: %s' % self.error)

    def send(self, p):
        self.bot.set_joints_position([self.bot.chassis_name], [p],
                                    control_way='filter', use_wbc=False, add_default_torso=False)

    @staticmethod
    def torso_call(stage, function, *args, **kwargs):
        try:
            return function(*args, **kwargs)
        except HeightLimit:
            raise
        except Exception as exc:
            raise RuntimeError('腰部%s失败 [%s]: %s' %
                               (stage, type(exc).__name__, exc)) from exc

    def torso_state(self):
        names = [self.bot.torso_name]
        def read_pose():
            if getattr(self, '_torso_pose_compat_noted', False):
                native = self.bot.astribot_interface._AstribotInterface__astribot_class
                return native.get_pose_chassis_to_torso_current()
            try:
                return self.bot.get_current_cartesian_pose(names=names, frame='chassis')[0]
            except KeyError as exc:
                if exc.args != (self.bot.torso_name,):
                    raise
                # Installed SDKs can omit torso in their Cartesian name map.
                # This dedicated getter reads the same chassis-frame torso pose;
                # it does not solve IK, publish a command, or acquire control.
                native = self.bot.astribot_interface._AstribotInterface__astribot_class
                pose = native.get_pose_chassis_to_torso_current()
                if not getattr(self, '_torso_pose_compat_noted', False):
                    print('\nSDK 通用位姿映射缺少躯干；已改用躯干专用位姿读取。', flush=True)
                    self._torso_pose_compat_noted = True
                return pose
        return (self.torso_call('位姿读取', read_pose),
                self.torso_call('关节反馈读取', self.bot.get_current_joints_position, names)[0])

    def torso_reference(self):
        names = [self.bot.torso_name]
        joints = vector(self.torso_call('当前目标读取', self.bot.get_desired_joints_position, names)[0],
                        4, '躯干当前目标')
        poses = self.torso_call('目标正解', self.bot.get_forward_kinematics, names, [joints])
        if not isinstance(poses, dict) or self.bot.torso_name not in poses:
            raise RuntimeError('SDK 躯干正解结果格式无效')
        return poses[self.bot.torso_name], joints

    def solve_torso(self, target):
        valid, result = self.torso_call('逆解', self.bot.get_inverse_kinematics,
                                       [self.bot.torso_name], [target])
        if not valid:
            raise HeightLimit('当前姿态无法到达该高度')
        if not isinstance(result, dict) or self.bot.torso_name not in result:
            raise RuntimeError('SDK 躯干逆解结果格式无效')
        return result[self.bot.torso_name]

    def check_torso(self, joints):
        lower, upper = self.torso_call('限位读取', self.bot.get_joints_position_limit,
                                      [self.bot.torso_name])
        lo, hi = vector(lower[0], 4, '躯干下限'), vector(upper[0], 4, '躯干上限')
        if any(a >= b for a, b in zip(lo, hi)):
            raise RuntimeError('SDK 躯干限位无效')
        if any(not a + .0001 <= q <= b - .0001 for q, a, b in zip(joints, lo, hi)):
            raise HeightLimit('已到 SDK 关节限位')
        distance = float(self.torso_call('自碰撞检查', self.bot.get_self_closest_point,
                                         torso_joints_position=joints)[0])
        if not math.isfinite(distance):
            raise RuntimeError('SDK 自碰撞距离无效')
        if distance < .03:
            raise HeightLimit('SDK 自碰撞距离小于 3 cm')

    def send_torso(self, joints):
        self.torso_call('目标发送', self.bot.set_joints_position, [self.bot.torso_name], [joints],
                                    control_way='direct', use_wbc=False, add_default_torso=False)

    def pose_available(self, name):
        return name == 'home' or self.pack is not None

    def prepare_pose(self):
        if not self.has_control():
            raise RuntimeError('姿态控制权丢失')
        self.check()
        self.bot.stop_robot()
        # Same reset used by the vendor when acquiring control: do not replay
        # the old accumulated chassis target after a pose stop.
        native = self.bot.astribot_interface._AstribotInterface__astribot_class
        native.set_current_position_as_desired_joint_position()
        self.bot.restart_robot()

    def stop_pose(self):
        self.bot.stop_robot()

    def execute_pose(self, name, cancel):
        if cancel.is_set():
            return
        if name == 'home':
            result = self.bot.move_to_home(duration=5., use_wbc=False)
            if not cancel.is_set() and isinstance(result, str) and any(
                    word in result.lower() for word in ('fail', 'error')):
                raise RuntimeError('SDK 初始姿态失败: ' + result)
            return
        if name != 'pack' or self.pack is None:
            raise RuntimeError('未配置打包姿态')
        lower, upper = self.bot.get_joints_position_limit(NAMES)
        if len(lower) != len(NAMES) or len(upper) != len(NAMES):
            raise RuntimeError('SDK 关节限位格式无效')
        for step in self.pack:
            commands = [step['joints'][n] for n in NAMES]
            for row, lo, hi in zip(commands, lower, upper):
                if len(row) != len(lo) or len(row) != len(hi) or any(
                        not math.isfinite(a) or not math.isfinite(b) or not a <= q <= b
                        for q, a, b in zip(row, lo, hi)):
                    raise RuntimeError('打包姿态超过 SDK 关节限位')
        native = self.bot.astribot_interface._AstribotInterface__astribot_class
        for step in self.pack:
            if cancel.is_set():
                return
            if not self.has_control():
                raise RuntimeError('姿态控制权丢失')
            commands = [step['joints'][n] for n in NAMES]
            # Recheck each leg from its actual starting configuration.
            safe = native.check_to_joint_position(dict(
                name_list=NAMES, command_list=commands,
                duration=step['duration'], safe_distance=.03))
            if cancel.is_set():
                return
            if not safe:
                raise RuntimeError('SDK 拒绝打包路径：碰撞检查未通过')
            result = self.bot.move_joints_position(
                NAMES, commands, duration=step['duration'],
                use_wbc=False, add_default_torso=False)
            if not cancel.is_set() and isinstance(result, str) and any(
                    word in result.lower() for word in ('fail', 'error')):
                raise RuntimeError('SDK 打包姿态失败: ' + result)

    def close(self):
        try:
            if self.bot:
                iface = self.bot.astribot_interface
                iface.shutdown()
                end = time.monotonic() + 1.5
                while getattr(iface, 'have_control_rights', False) and time.monotonic() < end:
                    time.sleep(.05)
                if getattr(iface, 'have_control_rights', False):
                    raise RuntimeError('SDK 控制权未在 1.5 秒内释放')
        finally:
            self.done.set()
            if self.thread:
                self.thread.join(timeout=.2)


def finish(driver, backend, dry, clock=time.monotonic, sleep=time.sleep):
    """Best-effort target hold, always followed by release; never integrate."""
    error = None
    try:
        if driver and driver.sent and not dry:
            end = clock() + 3.
            while clock() < end:
                if not backend.has_control():
                    raise RuntimeError('控制权已丢失，未继续保持目标')
                backend.check()
                backend.send(list(driver.target))
                sleep(1. / HZ)
    except Exception as exc:
        error = exc
    finally:
        try:
            backend.close()
        except Exception as exc:
            error = RuntimeError('%s; 释放失败: %s' % (error or '收尾', exc))
    if error:
        raise RuntimeError(str(error))


def run(args):
    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        raise RuntimeError('需要交互终端，请使用 ssh -t S1；不接受管道输入')
    keys = Keys(args.linear_speed, args.angular_speed)
    health = SessionHealth(getattr(args, 'session_fd', None))
    backend = driver = poses = None
    torso_speed = getattr(args, 'torso_speed', DEFAULT_TORSO_SPEED)
    pack = load_pack(getattr(args, 'pack_pose', None))
    saved = termios.tcgetattr(fd)
    handlers = {}
    signaled = []
    def halt(sig, frame):
        signaled.append(sig)
        keys.quit = True
        keys.stop('收到退出信号')
    try:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            handlers[sig] = signal.signal(sig, halt)
        with SessionLock():
            try:
                health.check(time.monotonic())
                others = conflicts(include_teleop=not args.dry_run)
                if others:
                    raise RuntimeError('检测到其他底盘控制进程 PID=%s，请先退出该程序' % ','.join(others))
                print('演练模式：不连接 SDK' if args.dry_run else
                      '将申请 S1 控制权。软件停止为保持目标，可能存在收敛尾程。', flush=True)
                backend = FakeBackend() if args.dry_run else SDKBackend(pack)
                if keys.quit:
                    return 128 + signaled[0] if signaled else 0
                health.check(time.monotonic())
                driver = Driver(backend, time.monotonic())
                poses = PoseControl(backend)
                torso = TorsoHeight(backend)
                configure_terminal(fd)
                print('READY | W/S 前后 · Q/E 左右转 · A/D 左右横移 · R/F 腰部升降 · 长按 H 初始/P 打包 · 空格停止 · +/- 调速 · X 退出', flush=True)
                print('坐标模式 SDK202_LOCAL：W/S 仅增减局部 x，A/D 仅增减局部 y，Q/E 仅增减 yaw；不作全局旋转。', flush=True)
                print('按键有效期 0.2 秒；H/P 需连续重复 0.6 秒，松键超时停止；短按不切换姿态。', flush=True)
                print('R/F 仅调整躯干高度，速度 %.3f m/s；松键保持目标。' % torso_speed, flush=True)
                print('打包姿态：' + ('仅模拟（演练）' if args.dry_run else
                      ('已加载配置' if pack else '未配置，P 不执行动作')), flush=True)
                show = 0.
                while not keys.quit:
                    # Always wait one fresh period: never catch up delayed cycles.
                    time.sleep(1. / HZ)
                    now = time.monotonic()
                    health.check(now)
                    if select.select([fd], [], [], 0)[0]:
                        data = os.read(fd, 4096)
                        if not data:
                            keys.quit = True
                            break
                        keys.feed(data.decode('ascii', errors='replace'), now)
                    if keys.quit:
                        break
                    velocity = keys.sample(now)
                    height_velocity = {'r': torso_speed, 'f': -torso_speed}.get(keys.key, 0.)
                    requested, selected = keys.pose(now)
                    # Health/control checks precede any pose restart or dispatch.
                    driver.check_cycle(now)
                    was_owned = poses.active or poses.blocked or poses.needs_resume
                    suppress = poses.tick(requested, selected, keys.until, keys.last_input, now)
                    if suppress or not height_velocity or poses.needs_resume:
                        torso.reset()
                    if poses.active or poses.needs_resume:
                        driver.sent = False
                    if not suppress and not selected:
                        if poses.needs_resume and (any(velocity) or height_velocity):
                            poses.resume()
                            driver.target = position(backend.desired())
                            driver.sent = False
                            driver.last = time.monotonic()
                            termios.tcflush(fd, termios.TCIFLUSH)
                            keys.stop('姿态停止已解除；重新按方向键驾驶，或按 R/F 调高低')
                        elif not poses.needs_resume:
                            driver.apply_velocity(velocity)
                            torso.tick(height_velocity, keys.until,
                                       allowed=lambda: not keys.quit and keys.key in ('r', 'f'))
                            if height_velocity:
                                keys.message = torso.message
                    elif selected and not was_owned and not poses.active:
                        # Arming/disabled pose only freezes the previous base target.
                        driver.apply_velocity((0., 0., 0.))
                    if suppress:
                        keys.message = poses.message
                    if now >= show:
                        status = '%s | v=%.3f m/s w=%.3f rad/s | local_cmd=[%.4f, %.4f, %.4f]' % (
                            keys.message, keys.linear, keys.angular, *driver.target)
                        columns = os.get_terminal_size(fd).columns or 240
                        print('\r\033[2K' + fit_status(status, columns), end='', flush=True)
                        show = now + .2
                print('\n退出：停止积分，清理控制会话。', flush=True)
                return 128 + signaled[0] if signaled else 0
            finally:
                if backend:
                    try:
                        if poses:
                            if poses.needs_resume:
                                driver.sent = False
                            poses.close()
                    finally:
                        finish(driver, backend, args.dry_run)
    finally:
        health.close()
        try:
            termios.tcsetattr(fd, termios.TCSANOW, saved)
        except (OSError, termios.error):
            pass  # SSH may already have closed the terminal.
        for sig, handler in handlers.items():
            signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description='S1 SSH 底盘遥操；X 退出，Q/E 旋转')
    parser.add_argument('--linear-speed', type=lambda x: speed(x, .01, MAX_LINEAR_SPEED), default=.05)
    parser.add_argument('--angular-speed', type=lambda x: speed(x, .025, .25), default=.10)
    parser.add_argument('--torso-speed', type=lambda x: speed(x, .001, MAX_TORSO_SPEED), default=DEFAULT_TORSO_SPEED, help='腰部升降速度 m/s，默认 0.02，范围 0.001–0.02')
    parser.add_argument('--dry-run', action='store_true', help='不加载 SDK，仅演练键盘与目标积分')
    default_pack = Path(__file__).resolve().parents[1] / 'config/teleop_pack_pose.json'
    parser.add_argument('--pack-pose', default=str(default_pack) if default_pack.exists() else None, help='带来源及动作顺序的打包姿态 JSON；未指定则禁用 P')
    parser.add_argument('--session-fd', type=int, default=None, help=argparse.SUPPRESS)
    args = parser.parse_args()
    try:
        return run(args)
    except Exception as exc:
        print('\nERROR: %s' % exc, file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    sys.exit(main())
