#!/usr/bin/env python3
"""Domain-125 synthetic ROS transport/save test, never field calibration or motion."""
import argparse
from array import array
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
from types import SimpleNamespace
import numpy as np
import yaml


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '125' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Test requires isolated Domain 125 and localhost discovery')
    from towergo_real.contracts import CALIBRATION_EVIDENCE
    from towergo_real.session import Observer, save_map, verify_map, stop_process
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2, PointField, LaserScan
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_share_directory
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    root=Path(args.output).resolve();root.mkdir(parents=True,exist_ok=False)
    proof=root/'SYNTHETIC_ONLY.txt';proof.write_text('Synthetic unit geometry; NOT robot calibration, no field accuracy claim.\n')
    identity=np.eye(4);base=identity.copy();base[:3,3]=[.2,0,.4]
    config=dict(verified=True,test_only=True,reviewed_by='automated synthetic fixture',reviewed_at='2026-09-23',
                base_from_lidar=base.tolist(),imu_from_lidar=identity.tolist(),body_polygon=[[-.3,-.3],[.3,-.3],[.3,.3],[-.3,.3]],
                obstacle_height_min=.1,obstacle_height_max=1.5,range_min=.3,range_max=15.,
                max_pose_step_m=.2,max_pose_step_rad=.2,imu_acceleration_unit='g',imu_gyro_unit='rad_s',point_timestamp_unit='absolute_ns',
                evidence={key:dict(path=str(proof),sha256=hashlib.sha256(proof.read_bytes()).hexdigest()) for key in CALIBRATION_EVIDENCE})
    calibration=root/'synthetic_calibration.yaml';calibration.write_text(yaml.safe_dump(config))
    obs=Observer();processes=[];logs=[];scans=[];odoms=[]
    report={'hardware_used':False,'synthetic_input':True,'field_accuracy_verified':False,'passed':False}
    def spawn(name,command):
        log=(root/(name+'.log')).open('w');logs.append(log)
        proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(proc);return proc
    try:
        adapter=spawn('adapter',['ros2','run','towergo_real','lio_adapter','--ros-args','-p','calibration:='+str(calibration)])
        slam=spawn('slam',['ros2','run','slam_toolbox','sync_slam_toolbox_node','--ros-args','--params-file',
                         str(Path(get_package_share_directory('towergo_real'))/'config/slam_real.yaml')])
        odom_pub=obs.node.create_publisher(Odometry,'/towergo/lio/odometry_raw',10)
        cloud_pub=obs.node.create_publisher(PointCloud2,'/towergo/lio/cloud_body',qos_profile_sensor_data)
        obs.node.create_subscription(LaserScan,'/scan',lambda m:scans.append(m),qos_profile_sensor_data)
        obs.node.create_subscription(Odometry,'/odom',lambda m:odoms.append(m),10)
        time.sleep(3)
        for k in range(120):
            if adapter.poll() is not None or slam.poll() is not None:raise RuntimeError('Child exited')
            x=.6*(1-abs(k-60)/60)
            stamp=obs.node.get_clock().now().to_msg()
            pose=Odometry();pose.header.stamp=stamp;pose.header.frame_id='camera_init';pose.child_frame_id='body'
            pose.pose.pose.position.x=x+.2;pose.pose.pose.position.z=.4;pose.pose.pose.orientation.w=1.
            odom_pub.publish(pose)
            # Four walls, expressed in the IMU/LiDAR frame at this frame end.
            a=np.linspace(-3,3,1200)
            points=np.concatenate([np.c_[a,np.full(len(a),3)],np.c_[a,np.full(len(a),-3)],
                                   np.c_[np.full(len(a),3),a],np.c_[np.full(len(a),-3),a]])
            xyz=np.c_[points[:,0]-x-.2,points[:,1],np.full(len(points),.1)].astype('<f4')
            cloud=PointCloud2();cloud.header.stamp=stamp;cloud.header.frame_id='body';cloud.height=1;cloud.width=len(xyz)
            cloud.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
            cloud.point_step=12;cloud.row_step=cloud.width*12;cloud.is_dense=True;cloud.data=array('B',xyz.tobytes())
            cloud_pub.publish(cloud);time.sleep(.1)
        assert len(scans)>100 and len(odoms)>100, (len(scans),len(odoms))
        assert all(m.header.frame_id=='scan_front' for m in scans)
        assert abs(odoms[60].pose.pose.position.x-.6)<.05
        latest=scans[-1];assert abs(latest.ranges[360]-(3-odoms[-1].pose.pose.position.x-.2))<.05
        assert obs.map is not None and obs.map.data, 'SLAM did not publish map'
        report['scan_frames']=len(scans);report['odom_frames']=len(odoms);report['ray_origin_checked']=True
        report['save']=save_map(obs,root/'saved_map')
        # A reset must terminate the adapter instead of silently extending the old map.
        pose.header.stamp.sec-=10;odom_pub.publish(pose)
        deadline=time.monotonic()+3
        while not obs.mapping_fault and time.monotonic()<deadline:time.sleep(.05)
        assert obs.mapping_fault,'LIO reset was not latched'
        report['reset_ends_session']=True
    finally:
        for p in reversed(processes):stop_process(p)
        for log in logs:log.close()
        obs.close()
    verify_map(SimpleNamespace(output=str(root/'saved_map')))
    report['ros_map_reload']=True;report['posegraph_restore']=True;report['passed']=True
    (root/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':main()
