"""End-to-end C++ scan conversion and pairing tests, run in the offline image."""
import math,os,subprocess,time
from pathlib import Path
import pytest
if not (Path('/.dockerenv').is_file() and os.environ.get('ROS_DOMAIN_ID') == '125'
        and os.environ.get('ROS_LOCALHOST_ONLY') == '1'):
    pytest.skip('Simulation ROS integration requires the isolated offline container (Domain 125, localhost)',
                allow_module_level=True)
rclpy=pytest.importorskip('rclpy')
pytest.importorskip('sensor_msgs_py')
from ament_index_python.packages import get_package_prefix
get_package_prefix('astribot_nav_sensors')
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,LaserScan
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from tf2_ros import StaticTransformBroadcaster

def test_scan_transform_sync_and_stale_clouds():
    rclpy.init();node=Node('scan_contract_test');received=[]
    process=subprocess.Popen(['ros2','run','astribot_nav_sensors','scan_processor','--ros-args','-p','use_sim_time:=true','-p','scan_source:=dual'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    try:
        front=node.create_publisher(PointCloud2,'/livox/lidar_front',qos_profile_sensor_data)
        back=node.create_publisher(PointCloud2,'/livox/lidar_back',qos_profile_sensor_data)
        clock=node.create_publisher(Clock,'/clock',10)
        node.create_subscription(LaserScan,'/scan',received.append,qos_profile_sensor_data)
        tf=StaticTransformBroadcaster(node)
        transforms=[]
        for side,x,yaw in [('front',.3,0.),('back',-.3,math.pi)]:
            t=TransformStamped();t.header.frame_id='base_link';t.child_frame_id='lidar_'+side;t.transform.translation.x=x;t.transform.translation.z=.3;t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2);transforms.append(t)
        tf.sendTransform(transforms)
        def spin(seconds=.4):
            end=time.monotonic()+seconds
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        spin(2.)
        def stamp(t):return Time(sec=int(t),nanosec=round((t-int(t))*1e9))
        def pair(a,b,now):
            clock.publish(Clock(clock=stamp(now)));spin(.05)
            front.publish(create_cloud_xyz32(Header(stamp=stamp(a),frame_id='lidar_front'),[[1.,0.,0.],[-2.,0.,0.],[float('nan'),0.,0.],[0.,0.,0.]]))
            back.publish(create_cloud_xyz32(Header(stamp=stamp(b),frame_id='lidar_back'),[[1.,0.,0.]]));spin()
        pair(1.,1.03,1.03)
        assert not received,'Mismatched timestamps must not be fused'
        pair(2.,2.01,2.01)
        assert len(received)==1
        finite=[r for r in received[0].ranges if math.isfinite(r)]
        assert received[0].header.frame_id=='lidar_front'
        assert len(finite)==2 and abs(min(finite)-1.)<1e-5 and abs(max(finite)-1.6)<1e-5
        pair(2.1,2.1,3.)
        assert len(received)==1,'Stale point clouds must be rejected'
    finally:
        process.terminate();process.wait(timeout=10);node.destroy_node();rclpy.shutdown()
