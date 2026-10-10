"""ROS hosting for control.py; 250 Hz control is independent of ROS callbacks."""
import json
import os
import signal
import threading
import time
from .control import Controller, FakeBackend, State
from .contracts import hardware_contract


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.signals import SignalHandlerOptions

    # Our handlers stop the controller before ROS entities/context are destroyed.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    class Chassis(Node):
        def __init__(self):
            super().__init__('towergo_chassis')
            self.backend_name = self.declare_parameter('backend', 'disabled').value
            self.contract_path = self.declare_parameter('hardware_contract', '').value
            self.declare_parameter('use_sim_time', False) if not self.has_parameter('use_sim_time') else None
            if self.get_parameter('use_sim_time').value:
                raise RuntimeError('Control always uses real time')
            if self.backend_name not in ('disabled', 'fake', 'astribot'):
                raise ValueError('Unknown backend')
            if self.backend_name == 'fake' and os.environ.get('ROS_DOMAIN_ID') != '125':
                raise ValueError('Fake backend requires Domain 125')
            limits = None
            self.contract = None
            if self.backend_name == 'astribot':
                if os.environ.get('ROS_DOMAIN_ID') != '25':
                    raise RuntimeError('Astribot backend requires Domain 25')
                self.contract, limits = hardware_contract(self.contract_path)
            self.core = Controller(limits)
            self.lock = threading.RLock()
            self.feedback_lock = threading.Lock()
            self.feedback_value = ([], float('-inf'))
            self.feedback_header = None
            self.quit = threading.Event()
            self.create_subscription(TwistStamped, '/towergo/cmd_vel_teleop', self.command, QoSProfile(depth=1))
            self.status_pub = self.create_publisher(String, '/towergo/control_status', QoSProfile(depth=1))
            self.create_service(Trigger, '/towergo/chassis/enable', self.enable)
            self.create_service(Trigger, '/towergo/chassis/disable', self.disable)
            if self.contract:
                from astribot_msgs.msg import RobotJointState
                self.create_subscription(RobotJointState, self.contract['feedback_topic'], self.feedback,
                                         qos_profile_sensor_data, callback_group=ReentrantCallbackGroup())
            self.create_timer(.2, self.status)
            self.worker = threading.Thread(target=self.loop, name='towergo-chassis-loop', daemon=True)
            self.worker.start()

        def feedback(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            now_ros = self.get_clock().now().nanoseconds*1e-9
            valid = 0 <= now_ros-stamp <= self.core.limits.feedback_timeout
            with self.feedback_lock:
                valid = valid and (self.feedback_header is None or stamp > self.feedback_header)
                self.feedback_header = stamp
                self.feedback_value = (list(msg.position), time.monotonic() if valid else float('-inf'))

        def measured(self):
            with self.feedback_lock:
                return list(self.feedback_value[0]), self.feedback_value[1]

        def conflict(self):
            return any(name == '/astribot/control_rights' for name, _ in self.get_service_names_and_types())

        def enable(self, req, resp):
            with self.lock:
                try:
                    if self.quit.is_set():
                        raise RuntimeError('Control process is shutting down')
                    if self.backend_name == 'disabled':
                        raise RuntimeError('backend=disabled; no SDK will be constructed')
                    if self.core.backend is not None:
                        raise RuntimeError('Disable/restart before re-enabling')
                    if self.backend_name == 'fake':
                        factory = lambda: FakeBackend(time.monotonic)
                    else:
                        # A feedback stream must already exist; never initialize SDK to
                        # manufacture an apparent read-only state probe.
                        _, received = self.measured()
                        if time.monotonic()-received > self.core.limits.feedback_timeout:
                            raise RuntimeError('No fresh measured chassis feedback')
                        from .sdk_backend import AstribotPositionBackend
                        factory = lambda: AstribotPositionBackend(self.contract_path, self.measured, self.conflict)
                    self.core.enable(factory, time.monotonic, lambda: self.get_clock().now().nanoseconds*1e-9)
                    resp.success, resp.message = True, 'Enabled; keyboard must send zero before a new movement key'
                except Exception as exc:
                    resp.success, resp.message = False, str(exc)
            return resp

        def disable(self, req, resp):
            with self.lock:
                resp.success = self.core.disable()
                resp.message = self.core.reason
                if self.backend_name == 'astribot':
                    # Vendor shutdown closes the global ROS context. Restart the
                    # control process for a new hardware session; never auto-rearm.
                    self.quit.set()
            return resp

        def command(self, msg):
            with self.lock:
                if self.quit.is_set():
                    return
                if msg.header.frame_id != 'base_link':
                    if self.core.state in (State.READY, State.MOVING):
                        self.core.fault('Command frame must be base_link')
                    return
                t = msg.twist
                values = [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z]
                self.core.command(values, msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                                  time.monotonic(), self.get_clock().now().nanoseconds*1e-9)

        def loop(self):
            while not self.quit.is_set():
                start = time.monotonic()
                with self.lock:
                    self.core.tick(start)
                self.quit.wait(max(0., .004-(time.monotonic()-start)))

        def status(self):
            with self.lock:
                if self.quit.is_set():
                    return
                if self.count_publishers('/towergo/cmd_vel_teleop') > 1 and self.core.state in (State.READY, State.MOVING):
                    self.core.fault('Multiple keyboard command publishers')
                msg = String()
                msg.data = json.dumps({'state': self.core.state.value, 'reason': self.core.reason,
                                       'backend': self.backend_name, 'velocity': self.core.velocity})
                self.status_pub.publish(msg)

        def halt(self):
            self.quit.set()
            self.worker.join(timeout=1.)
            with self.lock:
                self.core.fault('Control process shutting down')

        def close(self):
            with self.lock:
                self.core.disable()

    node = None
    executor = None
    try:
        node = Chassis()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        def on_signal(signum, frame):
            node.quit.set()
        for sig in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
            signal.signal(sig, on_signal)
        while rclpy.ok() and not node.quit.is_set():
            executor.spin_once(timeout_sec=.1)
    finally:
        if node is not None:
            node.halt()
            if executor is not None:
                # Vendor Humble does not join MultiThreadedExecutor's pool in
                # shutdown(). Drain queued callbacks while ROS entities live.
                pool = getattr(executor, '_executor', None)
                if pool is not None:
                    pool.shutdown(wait=True)
                executor.shutdown()
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
