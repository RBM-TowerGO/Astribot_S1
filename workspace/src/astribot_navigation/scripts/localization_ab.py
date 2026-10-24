#!/usr/bin/env python3
"""Isolated real-AMCL A/B on deterministic map raycasts; NEVER loads the SDK.
600 simulated seconds stationary + turn + translation. Synthetic odom noise is
explicit: this validates policy, not the missing Oct 24 raw failure recording.
"""
import argparse,json,math,os,subprocess,time,signal
from pathlib import Path
import numpy as np
import yaml

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--map',required=True);ap.add_argument('--output',required=True)
    ap.add_argument('--static-seconds',type=float,default=600);a=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='126' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('This experiment requires isolated localhost ROS Domain 126')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseWithCovarianceStamped,TransformStamped
    from sensor_msgs.msg import LaserScan
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from lifecycle_msgs.srv import ChangeState
    from astribot_nav_sim.grid_world import GridWorld
    from real_navigation.localization import MapMatcher
    out=Path(a.output);out.mkdir(parents=True,exist_ok=True);g=GridWorld(a.map)
    truth=g.initial_pose();truth[2]=0.;matcher=MapMatcher(g);angles=np.linspace(-math.pi,math.pi,360,endpoint=False)
    roots=Path(__file__).resolve().parents[1]
    params=yaml.safe_load((roots/'astribot_nav_bringup/config/nav2.yaml').read_text())['amcl']['ros__parameters']
    params.update(use_sim_time=True,tf_broadcast=False,global_frame_id='map',odom_frame_id='odom',base_frame_id='base',scan_topic='/ab/scan',map_topic='/ab/map')
    procs=[];logs=[];rclpy.init();n=rclpy.create_node('localization_ab_probe');records={'baseline':[],'candidate':[]}
    clock=n.create_publisher(Clock,'/clock',10);tf=n.create_publisher(TFMessage,'/ab/tf',100)
    scans=n.create_publisher(LaserScan,'/ab/scan',qos_profile_sensor_data)
    def stamp(t):
        from builtin_interfaces.msg import Time
        return Time(sec=int(t),nanosec=int((t-int(t))*1e9))
    def publish_clock(t):m=Clock();m.clock=stamp(t);clock.publish(m)
    def run(name,exe,p,remap):
        path=out/(name+'.yaml');path.write_text(yaml.safe_dump({'/**':{'ros__parameters':p}}))
        log=(out/(name+'.log')).open('w');logs.append(log)
        cmd=['nice','-n','10',exe,'--ros-args','-r','__node:='+name,'--params-file',str(path),'-r','/tf:=/ab/tf','-r','/tf_static:=/ab/tf_static']
        for k,v in remap.items():cmd+=['-r',k+':='+v]
        procs.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT))
    def transition(name,ident):
        c=n.create_client(ChangeState,'/'+name+'/change_state');deadline=time.monotonic()+15
        while not c.wait_for_service(timeout_sec=.1):
            if time.monotonic()>deadline:raise RuntimeError('lifecycle unavailable '+name)
            publish_clock(100.);rclpy.spin_once(n,timeout_sec=.01)
        req=ChangeState.Request();req.transition.id=ident;f=c.call_async(req)
        while not f.done():
            publish_clock(100.);rclpy.spin_once(n,timeout_sec=.02)
            if time.monotonic()>deadline:raise RuntimeError('lifecycle timed out '+name)
        if not f.result().success:raise RuntimeError('lifecycle rejected '+name)
    try:
        run('ab_map','/opt/ros/humble/lib/nav2_map_server/map_server',dict(use_sim_time=True,yaml_filename=a.map,topic_name='/ab/map',frame_id='map'),{})
        pubs={}
        for key,threshold in [('baseline',0.),('candidate',.05)]:
            p=dict(params,update_min_d=threshold,update_min_a=threshold)
            run('ab_'+key,'/opt/ros/humble/lib/nav2_amcl/amcl',p,{'initialpose':'/ab/'+key+'/initialpose','amcl_pose':'/ab/'+key+'/pose','map':'/ab/map'})
            pubs[key]=n.create_publisher(PoseWithCovarianceStamped,'/ab/'+key+'/initialpose',10)
            def received(m,k=key):
                q=m.pose.pose.orientation;p=m.pose.pose.position
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                records[k].append(dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,pose=[p.x,p.y,yaw],cov=[m.pose.covariance[i] for i in [0,7,35]]))
            n.create_subscription(PoseWithCovarianceStamped,'/ab/'+key+'/pose',received,10)
        for name in ('ab_map','ab_baseline','ab_candidate'):
            transition(name,1);transition(name,3)
        initial=PoseWithCovarianceStamped();initial.header.frame_id='map';initial.header.stamp=stamp(100.)
        initial.pose.pose.position.x=float(truth[0]);initial.pose.pose.position.y=float(truth[1]);initial.pose.pose.orientation.w=1.
        initial.pose.covariance[0]=initial.pose.covariance[7]=.15**2;initial.pose.covariance[35]=math.radians(5)**2
        # Warm up TF before a single initial pose, then continuously exercise the filters.
        truth_log=[];last_progress=-1;last_static=None
        for i in range(int((a.static_seconds+100)*10)+20):
            t=100+i*.1;elapsed=max(0,(i-20)*.1);pose=truth.copy()
            if elapsed>a.static_seconds:pose[2]=min((elapsed-a.static_seconds)*.02,math.pi/2)
            if elapsed>a.static_seconds+80:pose[0]+=(elapsed-a.static_seconds-80)*.02
            noisy=[pose[0]+.004*math.sin(elapsed*2.3),pose[1]+.004*math.sin(elapsed*1.7),pose[2]+.002*math.sin(elapsed*2.1)]
            publish_clock(t);tr=TransformStamped();tr.header.frame_id='odom';tr.child_frame_id='base';tr.header.stamp=stamp(t)
            tr.transform.translation.x=float(noisy[0]);tr.transform.translation.y=float(noisy[1]);tr.transform.rotation.z=math.sin(noisy[2]/2);tr.transform.rotation.w=math.cos(noisy[2]/2)
            tf.publish(TFMessage(transforms=[tr]))
            if i==15:
                for pub in pubs.values():pub.publish(initial)
            if i>=20:
                if elapsed<=a.static_seconds and last_static is not None:ranges=last_static
                else:ranges=g.raycast(pose[:2],angles+pose[2]);last_static=ranges
                scan=LaserScan();scan.header.frame_id='base';scan.header.stamp=stamp(t);scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/360;scan.angle_max=math.pi-scan.angle_increment;scan.range_min=.15;scan.range_max=12.;scan.ranges=ranges.astype(float).tolist();scans.publish(scan)
                truth_log.append(dict(stamp=t,pose=pose.tolist()))
            end=time.monotonic()+.025
            while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.001)
            bucket=int(elapsed)//60
            if bucket!=last_progress:print('simulated_seconds',round(elapsed), 'poses',{k:len(v) for k,v in records.items()},flush=True);last_progress=bucket
            if any(p.poll() is not None for p in procs):raise RuntimeError('AMCL experiment process exited')
        result=dict(kind='synthetic_map_raycast_with_real_amcl',static_seconds=a.static_seconds,truth=truth_log,records=records)
        (out/'results.json').write_text(json.dumps(result))
        summary={}
        for k,rs in records.items():
            errors=[];static=[]
            for row in rs:
                idx=min(len(truth_log)-1,max(0,int(round((row['stamp']-102)/.1))))
                p=truth_log[idx]['pose'];v=row['pose'];e=[math.dist(p[:2],v[:2]),abs(math.atan2(math.sin(p[2]-v[2]),math.cos(p[2]-v[2])))]
                errors.append(e)
                if row['stamp']<=102+a.static_seconds:static.append(e)
            summary[k]=dict(updates=len(rs),static_max=np.max(static,axis=0).tolist() if static else None,all_max=np.max(errors,axis=0).tolist() if errors else None)
        (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
    finally:
        for p in procs:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in procs:
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        for f in logs:f.close()
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
