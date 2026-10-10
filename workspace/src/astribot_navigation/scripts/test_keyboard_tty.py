#!/usr/bin/env python3
"""PTY + ROS keyboard check in Domain 125; no chassis node or SDK."""
import argparse
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import termios
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='125' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Test requires local Domain 125')
    import rclpy
    from geometry_msgs.msg import TwistStamped
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    path=Path(args.output)
    if path.exists():raise RuntimeError('Refusing to replace report')
    rclpy.init();node=rclpy.create_node('keyboard_pty_test');messages=[]
    node.create_subscription(TwistStamped,'/towergo/cmd_vel_teleop',lambda m:messages.append(m),10)
    def spin(t):
        end=time.monotonic()+t
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    master,slave=pty.openpty();saved=termios.tcgetattr(slave)
    process=subprocess.Popen(['ros2','run','towergo_real','keyboard'],stdin=slave,stdout=slave,stderr=slave,start_new_session=True)
    try:
        spin(2);assert messages and process.poll() is None
        for key,v,w in [('w',.1,0.),('s',-.1,0.),('a',0.,.2),('d',0.,-.2)]:
            messages.clear();os.write(master,key.encode());spin(.15)
            assert any(m.twist.linear.x==v and m.twist.angular.z==w for m in messages),key
            spin(.2);assert messages[-1].twist.linear.x==messages[-1].twist.angular.z==0.
        os.write(master,b'w');spin(.1);os.write(master,b' ');spin(.1)
        assert messages[-1].twist.linear.x==0.
        os.write(master,b'q');spin(.2);process.wait(timeout=3)
        assert process.returncode==0 and termios.tcgetattr(slave)==saved
        assert messages[-1].twist.linear.x==messages[-1].twist.angular.z==0.
        path.write_text(json.dumps(dict(passed=True,hardware_used=False,wasd=True,lease_expiry=True,space=True,q_exit=True,terminal_restored=True),indent=2))
        print(path.read_text())
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=3)
        os.close(master);os.close(slave);node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
