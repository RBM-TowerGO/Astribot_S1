import os,time,subprocess
from pathlib import Path
import pytest
if not (Path('/.dockerenv').is_file() and os.environ.get('ROS_DOMAIN_ID') == '125'
        and os.environ.get('ROS_LOCALHOST_ONLY') == '1'):
    pytest.skip('Simulation ROS integration requires the isolated offline container (Domain 125, localhost)',
                allow_module_level=True)
rclpy=pytest.importorskip('rclpy')
pytest.importorskip('sensor_msgs_py')
from ament_index_python.packages import get_package_prefix
# A missing simulation overlay inside the test container is an error, not a skip.
get_package_prefix('astribot_nav_bridge')
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data

def test_command_guard_stale_scan_invalid_twist_and_clock_jump():
    rclpy.init();n=Node('guard_test');out=[]
    p=subprocess.Popen(['ros2','run','astribot_nav_bridge','command_guard','--ros-args','-p','use_sim_time:=true'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    try:
        scan=n.create_publisher(LaserScan,'/scan',qos_profile_sensor_data);cmd=n.create_publisher(Twist,'/cmd_vel_nav',10);clock=n.create_publisher(Clock,'/clock',10)
        n.create_subscription(Twist,'/cmd_vel',out.append,10)
        def spin(duration=.08):
            end=time.monotonic()+duration
            while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
        spin(2.)
        def publish(t,scanned=True,value=.2):
            stamp=Time(sec=int(t),nanosec=int(round((t-int(t))*1e9)))
            clock.publish(Clock(clock=stamp));spin(.02)
            if scanned:m=LaserScan();m.header.stamp=stamp;scan.publish(m)
            m=Twist();m.linear.x=value;cmd.publish(m);spin(.02)
        for i in range(10):publish(1+i*.05)
        assert any(m.linear.x>.1 for m in out)
        out.clear()
        for i in range(20):publish(1.5+i*.05,False)
        assert out and out[-1].linear.x==0
        for i in range(5):publish(3+i*.05,True,float('nan'))
        assert out[-1].linear.x==0
        # Time rollback must not cause a stale velocity to resume.
        out.clear()
        for i in range(5):
            clock.publish(Clock(clock=Time(sec=1,nanosec=i*50_000_000)));spin()
        assert not out or all(m.linear.x==0 for m in out)
    finally:
        p.terminate();p.wait(timeout=10);n.destroy_node();rclpy.shutdown()
