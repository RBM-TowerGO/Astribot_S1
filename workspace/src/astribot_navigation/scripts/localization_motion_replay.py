#!/usr/bin/env python3
"""Two real AMCL instances, recorded scan/odom, isolated localhost Domain 127.
No SDK, cmd_vel, navigation actions or recorded map->odom transforms are replayed.
Initial estimate is the first recorded AMCL estimate, NOT external ground truth.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import yaml


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run',required=True);ap.add_argument('--output',required=True);a=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='127' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('Requires localhost Domain 127')
    import rclpy
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped,TransformStamped
    from tf2_msgs.msg import TFMessage
    from rosgraph_msgs.msg import Clock
    from lifecycle_msgs.srv import ChangeState
    from astribot_nav_sim.grid_world import GridWorld
    from real_navigation.localization import MapMatcher
    run=Path(a.run);out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    old=yaml.safe_load((run/'nav2.yaml').read_text())
    old=old.get('towergo',{}).get('nav',old)
    # Session files have ordinary node names under a namespace rewrite.
    params=old['amcl']['ros__parameters'];mapfile=old['map_server']['ros__parameters']['yaml_filename']
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(run/'diagnostic_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    topics={'/towergo/nav/scan':LaserScan,'/towergo/nav/odom':Odometry,'/towergo/nav/tf_static':TFMessage,'/towergo/nav/amcl_pose':PoseWithCovarianceStamped}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    def ns(stamp):return stamp.sec*1000000000+stamp.nanosec
    events=[];initial=None;statics=None
    while r.has_next():
        topic,b,received=r.read_next();m=deserialize_message(b,topics[topic])
        if topic.endswith('tf_static'):statics=m
        elif topic.endswith('amcl_pose'):
            if initial is None:initial=m
        else:events.append((ns(m.header.stamp),0 if topic.endswith('odom') else 1,m))
    if initial is None or statics is None:raise RuntimeError('Incomplete evidence')
    start=ns(initial.header.stamp);events=sorted((e for e in events if e[0]>=start),key=lambda e:(e[0],e[1]))
    origin=next(t.transform.translation for t in statics.transforms if t.child_frame_id==next(e[2].header.frame_id for e in events if e[1]==1))
    matcher=MapMatcher(GridWorld(mapfile),(origin.x,origin.y));scan_by_stamp={e[0]:e[2] for e in events if e[1]==1}
    records={'baseline':[],'candidate':[]};processes=[];logs=[]
    rclpy.init();n=rclpy.create_node('motion_model_replay_probe')
    clock=n.create_publisher(Clock,'/clock',10);tf=n.create_publisher(TFMessage,'/motion_ab/tf',100)
    static=n.create_publisher(TFMessage,'/motion_ab/tf_static',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    scanpub=n.create_publisher(LaserScan,'/motion_ab/scan',qos_profile_sensor_data)
    def setclock(t):
        c=Clock();c.clock.sec=t//1000000000;c.clock.nanosec=t%1000000000;clock.publish(c)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.001)
    def launch(name,package,exe,p,remaps):
        path=out/(name+'.yaml');path.write_text(yaml.safe_dump({'/**':{'ros__parameters':p}}))
        log=(out/(name+'.log')).open('w');logs.append(log)
        cmd=['/opt/ros/humble/lib/'+package+'/'+exe,'--ros-args','-r','__node:='+name,'--params-file',str(path),'-r','/tf:=/motion_ab/tf','-r','/tf_static:=/motion_ab/tf_static']
        for k,v in remaps.items():cmd+=['-r',k+':='+v]
        processes.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT))
    def activate(name):
        client=n.create_client(ChangeState,'/'+name+'/change_state')
        if not client.wait_for_service(timeout_sec=15):raise RuntimeError('Lifecycle service missing '+name)
        for ident in (1,3):
            req=ChangeState.Request();req.transition.id=ident;f=client.call_async(req);deadline=time.monotonic()+15
            while not f.done():
                if time.monotonic()>deadline:raise RuntimeError('Lifecycle timeout '+name)
                setclock(start);spin(.01)
            if not f.result().success:raise RuntimeError('Lifecycle rejected '+name)
    try:
        launch('motion_map','nav2_map_server','map_server',dict(use_sim_time=True,yaml_filename=mapfile,frame_id='tg_nav_map',topic_name='/motion_ab/map'),{})
        pubs={}
        for key,model in [('baseline','nav2_amcl::DifferentialMotionModel'),('candidate','towergo::LioPlanarMotionModel')]:
            p=dict(params,use_sim_time=True,tf_broadcast=False,scan_topic='/motion_ab/scan',map_topic='/motion_ab/map',robot_model_type=model)
            launch('motion_'+key,'nav2_amcl','amcl',p,dict(initialpose='/motion_ab/'+key+'/initialpose',amcl_pose='/motion_ab/'+key+'/pose',map='/motion_ab/map'))
            pubs[key]=n.create_publisher(PoseWithCovarianceStamped,'/motion_ab/'+key+'/initialpose',10)
            def received(m,k=key):
                q=m.pose.pose.orientation;pos=m.pose.pose.position;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                row=dict(stamp=ns(m.header.stamp),pose=[pos.x,pos.y,yaw],cov=[m.pose.covariance[i] for i in (0,7,35)])
                scan=scan_by_stamp.get(row['stamp'])
                if scan is not None:
                    ranges=np.asarray(scan.ranges);angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
                    valid=np.isfinite(ranges)&(ranges>=scan.range_min)&(ranges<=min(12.,scan.range_max))
                    pts=np.column_stack((ranges[valid]*np.cos(angles[valid])+origin.x,ranges[valid]*np.sin(angles[valid])+origin.y))
                    row['quality']=matcher.evaluate(pts,row['pose'])
                records[k].append(row)
            n.create_subscription(PoseWithCovarianceStamped,'/motion_ab/'+key+'/pose',received,100)
        for name in ('motion_map','motion_baseline','motion_candidate'):activate(name)
        setclock(start);static.publish(statics)
        # Supply the first measured odom before submitting one identical initial pose.
        first=next(e[2] for e in events if e[1]==0)
        def odom_tf(m):
            tr=TransformStamped();tr.header=m.header;tr.child_frame_id=m.child_frame_id
            tr.transform.translation.x=m.pose.pose.position.x;tr.transform.translation.y=m.pose.pose.position.y;tr.transform.translation.z=m.pose.pose.position.z
            tr.transform.rotation=m.pose.pose.orientation;tf.publish(TFMessage(transforms=[tr]))
        odom_tf(first);spin(.3)
        for pub in pubs.values():pub.publish(initial)
        spin(.3);last=start;bucket=-1;scans=0
        for ts,kind,m in events:
            setclock(ts)
            if kind==0:odom_tf(m)
            else:
                spin(.002);scanpub.publish(m);scans+=1
            spin(min(.03,max(0,(ts-last)/1e9/3)));last=ts
            b=(ts-start)//30000000000
            if b!=bucket:print('recorded_seconds',round((ts-start)/1e9),'poses',{k:len(v) for k,v in records.items()},flush=True);bucket=b
            if any(p.poll() is not None for p in processes):raise RuntimeError('AMCL exited')
        spin(.5)
        summary={}
        for key,rows in records.items():
            qs=[v['quality'] for v in rows if 'quality' in v]
            summary[key]=dict(updates=len(rows),matched_scans=len(qs),covariance_max=np.max([v['cov'] for v in rows],axis=0).tolist(),
                              covariance_p95=np.percentile([v['cov'] for v in rows],95,axis=0).tolist(),
                              good_fraction=float(np.mean([v['good'] for v in qs])),match_fraction_p10=float(np.percentile([v['match_fraction'] for v in qs],10)))
        meta=dict(kind='recorded_scan_odom_real_amcl_ab',run=str(run),map=mapfile,map_yaml_sha256=hashlib.sha256(Path(mapfile).read_bytes()).hexdigest(),seconds=(last-start)/1e9,input_scans=scans,initial_estimate='first_recorded_AMCL_not_ground_truth')
        (out/'results.json').write_text(json.dumps(dict(**meta,records=records)))
        (out/'summary.json').write_text(json.dumps(dict(**meta,summary=summary),indent=2));print(json.dumps(summary),flush=True)
    finally:
        for p in processes:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        for log in logs:log.close()
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
