"""Short-lived key commands from a dedicated SSH TTY."""
import os
import select
import signal
import sys
import termios
import time
import tty
from .control import KeyboardLease


def main(args=None):
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.qos import QoSProfile, DurabilityPolicy
    if not sys.stdin.isatty():
        raise SystemExit('A TTY is required: ssh -t S1 (do not pipe/replay keyboard input)')
    if os.environ.get('ROS_DOMAIN_ID') not in ('25', '125'):
        raise SystemExit('Expected real Domain 25 or isolated test Domain 125')
    rclpy.init(args=args)
    node = rclpy.create_node('towergo_keyboard')
    pub = node.create_publisher(TwistStamped, '/towergo/cmd_vel_teleop',
                                QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
    lease = KeyboardLease()
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    handlers = {}
    def stop_signal(signum, frame):
        lease.quit = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        handlers[sig] = signal.signal(sig, stop_signal)
    def publish(v=0., w=0.):
        if rclpy.ok():
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x, msg.twist.angular.z = v, w
            pub.publish(msg)
    print('W/S 前后 · A/D 原地转向 · 空格停止 · Q 退出；按键有效期 0.2 秒。', flush=True)
    try:
        tty.setcbreak(fd)
        termios.tcflush(fd, termios.TCIFLUSH)
        previous = time.monotonic()
        deadline = previous
        while rclpy.ok() and not lease.quit:
            now = time.monotonic()
            if now - previous > .3:
                # A stalled terminal must not consume its buffered movement keys.
                termios.tcflush(fd, termios.TCIFLUSH)
                lease.feed('', now)
                lease.quit = True
                break
            previous = now
            if select.select([fd], [], [], 0)[0]:
                data = os.read(fd, 4096)
                if not data:
                    break
                lease.feed(data.decode('ascii', errors='replace'), now)
            publish(*lease.sample(now))
            rclpy.spin_once(node, timeout_sec=0)
            deadline += .05
            time.sleep(max(0., deadline - time.monotonic()))
    finally:
        try:
            publish()
            # Allow a bounded delivery opportunity; receiver watchdog is independent.
            rclpy.spin_once(node, timeout_sec=.05)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
            for sig, handler in handlers.items():
                signal.signal(sig, handler)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
