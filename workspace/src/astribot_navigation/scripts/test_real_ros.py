#!/usr/bin/env python3
"""Domain-125 integration checks. Reuses a bag; never touches robot services."""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--bag',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='125' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Integration test requires local Domain 125')
    import rclpy
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2,Imu
    from livox_ros_driver2.msg import CustomMsg
    from geometry_msgs.msg import TwistStamped
    from std_msgs.msg import String,Bool
    from std_srvs.srv import Trigger
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag,storage_id=''),rosbag2_py.ConverterOptions('',''))
    clouds=[];imus=[]
    while reader.has_next() and (len(clouds)<20 or len(imus)<20):
        t,d,_=reader.read_next()
        if t=='/livox/lidar_front' and len(clouds)<20:clouds.append(deserialize_message(d,PointCloud2))
        if t=='/livox/imu_front' and len(imus)<20:imus.append(deserialize_message(d,Imu))
    if len(clouds)<20 or len(imus)<20:raise RuntimeError('Need 20 real front frames and IMUs')
    rclpy.init();node=rclpy.create_node('towergo_real_integration')
    received=[];converted_imus=[];faults=[];statuses=[];latencies=[];sent={}
    def cloud_received(msg):
        received.append(msg)
        if msg.timebase in sent:latencies.append(time.monotonic()-sent[msg.timebase])
    node.create_subscription(CustomMsg,'/towergo/lio/lidar',cloud_received,20)
    node.create_subscription(Imu,'/towergo/lio/imu',converted_imus.append,100)
    node.create_subscription(Bool,'/towergo/mapping_fault',lambda m:faults.append(m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String,'/towergo/control_status',lambda m:statuses.append(json.loads(m.data)),10)
    cp=node.create_publisher(PointCloud2,'/livox/lidar_front',qos_profile_sensor_data)
    ip=node.create_publisher(Imu,'/livox/imu_front',qos_profile_sensor_data)
    kp=node.create_publisher(TwistStamped,'/towergo/cmd_vel_teleop',1)
    processes=[];logs=[];report={'hardware_used':False,'domain':125,'source_bag':args.bag}
    output=Path(args.output);output.mkdir(parents=True,exist_ok=False)
    def start(package,exe,params):
        log=(output/(exe+'.log')).open('w');logs.append(log)
        p=subprocess.Popen(['ros2','run',package,exe,'--ros-args',*params],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p);return p
    def spin(seconds,velocity=None):
        end=time.monotonic()+seconds;next_command=0
        while time.monotonic()<end:
            if velocity is not None and time.monotonic()>=next_command:
                m=TwistStamped();m.header.frame_id='base_link';m.header.stamp=node.get_clock().now().to_msg()
                m.twist.linear.x=float(velocity);kp.publish(m);next_command=time.monotonic()+.05
            rclpy.spin_once(node,timeout_sec=.005)
    try:
        adapter=start('towergo_livox_adapter','livox_adapter',['-p','imu_acceleration_unit:=g'])
        end=time.monotonic()+15
        while cp.get_subscription_count()==0 or ip.get_subscription_count()==0:
            if time.monotonic()>end or adapter.poll() is not None:raise RuntimeError('Converter failed to start')
            spin(.05)
        spin(.5)
        for cloud,imu in zip(clouds,imus):
            key=cloud.header.stamp.sec*10**9+cloud.header.stamp.nanosec
            sent[key]=time.monotonic();cp.publish(cloud);ip.publish(imu);spin(.1)
        spin(.5)
        assert len(received)==20,(len(received),'clouds received')
        assert len(converted_imus)==20
        for original,converted in zip(clouds,received):
            assert converted.timebase==original.header.stamp.sec*10**9+original.header.stamp.nanosec
            assert converted.point_num==original.width*original.height
            assert converted.points[-1].offset_time>90_000_000
            assert converted.header.frame_id=='lidar_front'
        assert abs(converted_imus[0].linear_acceleration.z-imus[0].linear_acceleration.z*9.80665)<1e-6
        # Invalid fields must never be silently replaced with synthetic timing.
        bad=copy.deepcopy(clouds[-1]);bad.header.stamp.sec+=1
        bad.fields=[f for f in bad.fields if f.name!='timestamp'];cp.publish(bad);spin(.2)
        assert len(received)==20
        cp.publish(clouds[0]);spin(.2)
        assert any(faults),'Clock reversal did not latch a session fault'
        report['converter']={'passed':True,'frames':20,'points_per_frame':received[0].point_num,
                             'latency_mean_ms':sum(latencies)/len(latencies)*1000,
                             'latency_max_ms':max(latencies)*1000,
                             'imu_unit_conversion':True,'missing_time_rejected':True,'clock_reset_latched':True}
        chassis=start('towergo_real','chassis',['-p','backend:=fake'])
        service=node.create_client(Trigger,'/towergo/chassis/enable')
        if not service.wait_for_service(timeout_sec=15):raise RuntimeError('Fake chassis startup failed')
        spin(.3,0.)
        future=service.call_async(Trigger.Request())
        deadline=time.monotonic()+3
        while not future.done() and time.monotonic()<deadline:spin(.02,0.)
        assert future.done() and future.result().success, ('Fake backend did not enable',future.result().message if future.done() else 'timeout')
        spin(.25,0.);spin(.4,-.1)
        assert any(s['state']=='moving' and s['velocity'][0]<0 for s in statuses),statuses
        spin(.4)  # Stop publishing: independent receiver timeout must latch fault.
        assert statuses[-1]['state']=='fault',statuses[-1]
        spin(.25,.1)
        assert statuses[-1]['state']=='fault','Unexpected automatic recovery'
        report['fake_chassis']={'passed':True,'reverse':True,'publisher_loss_latched_stop':True,'no_auto_rearm':True}
        report['passed']=True
    except BaseException as exc:
        report['passed']=False;report['error']=repr(exc)
        raise
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))


if __name__=='__main__':main()
