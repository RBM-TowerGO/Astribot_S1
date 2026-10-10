#!/usr/bin/env python3
"""Private FAST-LIO smoke test with upstream nominal extrinsics, never /odom or /map.
This tests initialization/transport only, not calibrated robot SLAM.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time
import yaml


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='125' or not Path('/.dockerenv').exists():
        raise RuntimeError('Use the isolated replay container')
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2,Imu
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_share_directory
    from towergo_real.session import stop_all
    parser=argparse.ArgumentParser();parser.add_argument('--bag',required=True);parser.add_argument('--output',required=True);args=parser.parse_args()
    root=Path(args.output);root.mkdir(parents=True,exist_ok=False)
    params=yaml.safe_load((Path(get_package_share_directory('fast_lio'))/'config/mid360.yaml').read_text())
    p=params['/**']['ros__parameters'];p['use_sim_time']=True
    p['common']['lid_topic']='/towergo/lio/lidar';p['common']['imu_topic']='/towergo/lio/imu'
    p['mapping']['extrinsic_est_en']=False
    p['publish'].update(path_en=False,map_en=False,dense_publish_en=True,scan_bodyframe_pub_en=True)
    p['pcd_save']['pcd_save_en']=False
    (root/'nominal_upstream_DIAGNOSTIC_ONLY.yaml').write_text(yaml.safe_dump(params))
    rclpy.init();node=rclpy.create_node('static_lio_transport_test');poses=[];clouds=[];norms=[];processes=[];logs=[]
    def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
    def pose(m):
        v=m.pose.pose.position;q=m.pose.pose.orientation
        assert all(math.isfinite(x) for x in [v.x,v.y,v.z,q.x,q.y,q.z,q.w])
        poses.append((stamp(m),v.x,v.y,v.z,time.monotonic()))
    node.create_subscription(Odometry,'/towergo/smoke/odom_raw',pose,100)
    node.create_subscription(PointCloud2,'/towergo/smoke/cloud_body',lambda m:clouds.append(stamp(m)),qos_profile_sensor_data)
    node.create_subscription(Imu,'/towergo/lio/imu',lambda m:norms.append(math.sqrt(sum(x*x for x in [m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]))),qos_profile_sensor_data)
    def spawn(name,command):
        log=(root/(name+'.log')).open('w');logs.append(log)
        proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(proc);return proc
    def spin(t):
        end=time.monotonic()+t
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    report=dict(hardware_used=False,nominal_extrinsics_only=True,calibrated_slam_verified=False,moving_slam_verified=False,image_id=os.environ.get('TOWERGO_IMAGE_ID'))
    try:
        converter=spawn('converter',['ros2','run','towergo_livox_adapter','livox_adapter','--ros-args','-p','use_sim_time:=true','-p','imu_acceleration_unit:=g'])
        fast=spawn('fast_lio',['ros2','run','fast_lio','fastlio_mapping','--ros-args','--params-file',str(root/'nominal_upstream_DIAGNOSTIC_ONLY.yaml'),
                  '-r','/Odometry:=/towergo/smoke/odom_raw','-r','/cloud_registered_body:=/towergo/smoke/cloud_body',
                  '-r','/cloud_registered:=/towergo/smoke/cloud_world','-r','/tf:=/towergo/smoke/tf_raw','-r','/tf_static:=/towergo/smoke/tf_static_raw'])
        spin(4)
        player=spawn('player',['ros2','bag','play',args.bag,'--clock','--rate','1.0','--topics','/livox/lidar_front','/livox/imu_front'])
        start=time.monotonic()
        while player.poll() is None:
            if time.monotonic()-start>90 or converter.poll() is not None or fast.poll() is not None:raise RuntimeError('Pipeline exited or timed out')
            spin(.1)
        spin(2)
        assert player.returncode==0 and len(poses)>400 and len(clouds)>400,(player.returncode,len(poses),len(clouds))
        assert len(set(clouds)&{p[0] for p in poses})>400
        assert all(b[0]>a[0] for a,b in zip(poses,poses[1:]))
        assert node.count_publishers('/odom')==0 and node.count_publishers('/map')==0 and node.count_publishers('/tf')==0
        report.update(passed=True,odom_frames=len(poses),deskewed_cloud_frames=len(clouds),converted_imu_count=len(norms),
                      mean_acceleration_si=sum(norms)/len(norms),trajectory_span_seconds=(poses[-1][0]-poses[0][0])*1e-9,
                      processing_wall_seconds=poses[-1][4]-poses[0][4],
                      stationary_endpoint_displacement_m=math.dist(poses[0][1:4],poses[-1][1:4]),
                      public_tf_odom_map_publishers=0,
                      note='Static initialization and transport only; upstream nominal extrinsics and g assumption are NOT field calibration.')
        (root/'raw_trajectory.json').write_text(json.dumps(poses))
    except BaseException as exc:
        report.update(passed=False,error=repr(exc));raise
    finally:
        try:
            stop_all(reversed(processes));report['clean_shutdown']=True
        except Exception as exc:
            report.update(passed=False,clean_shutdown=False,shutdown_error=str(exc));raise
        finally:
            for log in logs:log.close()
            node.destroy_node();rclpy.shutdown()
            (root/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':main()
