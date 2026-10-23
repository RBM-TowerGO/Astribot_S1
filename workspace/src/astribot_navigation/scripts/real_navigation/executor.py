"""Separate SDK process. JSON messages carry task IDs, source times and deadlines."""
import json
import math
import os
from pathlib import Path
import sys
import time
import signal
import yaml
from .core import Drive, validate_config
from .stability import Window
from .chassis_interface import ChassisInterface
from .vendor_stop import stop_chassis


def handover(client, request, pump, now=time.monotonic, timeout=3.):
    """Request vendor release and wait for acknowledgement AND disappearance."""
    if not client.service_is_ready():
        return
    future = client.call_async(request)
    deadline = now() + timeout
    while not future.done() and now() < deadline:
        pump()
    if not future.done():
        future.cancel()
        raise RuntimeError('厂商控制权交接响应超时；未创建 SDK')
    response = future.result()
    result = getattr(response, 'response', None)
    if result != 'success':
        raise RuntimeError('厂商拒绝控制权交接：' + str(result))
    deadline = now() + timeout
    while client.service_is_ready() and now() < deadline:
        pump()
    if client.service_is_ready():
        raise RuntimeError('厂商已回复 success，但原控制权服务未释放；未创建 SDK')


class SDK:
    def __init__(self, config, node):
        from tg_teleop import SessionLock, conflicts
        from towergo_real.sdk_backend import construct_without_interactive_takeover
        self.lock = SessionLock()
        self.lock.__enter__()
        self.bot = None
        self.error = None
        self.last_feedback = None
        try:
            import rclpy
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            occupied = conflicts()
            if occupied:
                raise RuntimeError('底盘控制程序占用，PID: ' + ', '.join(occupied))
            from astribot_msgs.srv import RawRequest
            client = node.create_client(RawRequest, '/astribot/control_rights')
            try:
                client.wait_for_service(timeout_sec=1.)
                handover(client, RawRequest.Request(),
                         lambda: rclpy.spin_once(node, timeout_sec=.05))
            finally:
                node.destroy_client(client)
            sys.path.insert(0, config['sdk_root'])
            from astribot_sdk.core.astribot_api.astribot_client import Astribot
            handlers={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
            try:
                self.bot = construct_without_interactive_takeover(Astribot, freq=100., high_control_rights=False,
                                                                   node_name='towergo_navigation_sdk')
            finally:
                for sig,handler in handlers.items():signal.signal(sig,handler)
            if not self.has_control():
                raise RuntimeError('没有获得底盘控制权')
            self.commands = ChassisInterface(self.bot, stop_chassis)
            # Installed SDK owns its executor and ROS spin thread.
            from astribot_msgs.msg import RobotJointState
            from rclpy.qos import qos_profile_sensor_data
            def received(msg):
                # Only freshness is taken from ROS; SDK supplies native coordinate values.
                self.last_feedback = time.monotonic()
            self.subscription = node.create_subscription(RobotJointState, '/astribot_chassis/joint_space_states', received, qos_profile_sensor_data)
        except BaseException:
            self.close()
            raise

    def desired(self):
        return self.commands.desired()

    def feedback(self):
        if self.error:
            raise RuntimeError('SDK 通信失败: '+str(self.error))
        return self.commands.feedback(), self.last_feedback or -1e20

    def has_control(self):
        return bool(self.bot and self.bot.get_control_rights_status())

    def send(self, p):
        self.commands.send(p)

    def stop(self):
        self.commands.stop()

    @property
    def requires_restart(self):
        return self.commands.stopped

    def close(self):
        if self.bot:
            self.bot.astribot_interface.shutdown()
        self.lock.__exit__(None,None,None)


class FakeSDK:
    """Explicit replay-only in-memory executor, not a physical robot model."""
    def __init__(self):self.p=[0.,0.,0.];self.last_feedback=time.monotonic()
    def desired(self):return self.p[:]
    def feedback(self):return self.p[:],time.monotonic()
    def has_control(self):return True
    def send(self,p):self.p=p[:]
    def stop(self):pass
    def close(self):pass


def main():
    import rclpy
    from std_msgs.msg import String
    config = yaml.safe_load(Path(sys.argv[1]).read_text())
    mode = sys.argv[2]
    validate_config(config, hardware=mode=='hardware')
    if mode not in ('hardware','mock') or os.environ.get('ROS_DOMAIN_ID') != ('25' if mode=='hardware' else '125'):
        raise RuntimeError('Incorrect executor mode/domain')
    rclpy.init(args=[])
    node = rclpy.create_node('navigation_executor',namespace='/towergo/nav')
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    backend = None
    drive = None
    last_sequence = -1
    status = node.create_publisher(String,'executor_status',1)
    try:
        backend = SDK(config,node) if mode=='hardware' else FakeSDK()
        deadline = time.monotonic()+5
        while backend.last_feedback is None and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        drive = Drive(backend,config,time.monotonic())
        def control(msg):
            nonlocal last_sequence
            try:
                data = json.loads(msg.data)
                now = time.monotonic()
                if data['sequence'] <= last_sequence or not 0 <= now-data['sent'] < .2:
                    return
                last_sequence = data['sequence']
                if data['op']=='stop':
                    drive.stop()
                elif data['op']=='cancel_stationary':
                    span=settling.span(now)
                    confirmed=bool(span is not None and len(span)==3 and math.hypot(span[0],span[1])<.005 and span[2]<.01)
                    drive.cancel_stationary(now,confirmed)
                elif data['op']=='prepare':
                    drive.prepare(data['task'],data['speed'])
                    drive.permit(data['task'],now)
                    drive.command(data['task'],0.,0.,now)
                elif data['op']=='command':
                    drive.permit(data['task'],now)
                    drive.command(data['task'],data['v'],data['w'],now)
            except Exception as exc:
                drive.stop(str(exc))
        node.create_subscription(String,'executor_control',control,1)
        last_status = 0.
        settling=Window(.15)
        last_settling_feedback=None
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.001)
            now = time.monotonic()
            if now-drive.last_tick < .01:
                continue
            command = drive.tick(now)
            if now-last_status >= .05:
                stationary=False;span=None
                try:
                    p,received=backend.feedback()
                    observed=time.monotonic()
                    if 0<=observed-received<config['feedback_timeout']:
                        if received!=last_settling_feedback:
                            settling.add(observed,p);last_settling_feedback=received
                        span=settling.span(observed)
                        stationary=bool(span is not None and len(span)==3 and
                                        math.hypot(span[0],span[1])<.005 and span[2]<.01)
                    else:settling.rows.clear()
                except Exception:settling.rows.clear()
                status.publish(String(data=json.dumps(dict(task=drive.task, speed=drive.limit, fault=drive.fault,
                                                         command=command, rights=backend.has_control(),sent=now,tracking=drive.tracking,stationary=stationary,settling_span=span))))
                last_status=now
    except KeyboardInterrupt:
        pass
    finally:
        # Let the first termination run cleanup; repeated signals cannot interrupt stop.
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        try:
            if drive:
                drive.stop()
        finally:
            if backend:
                backend.close()
            if rclpy.ok():
                node.destroy_node()
                rclpy.try_shutdown()

if __name__ == '__main__':
    main()
