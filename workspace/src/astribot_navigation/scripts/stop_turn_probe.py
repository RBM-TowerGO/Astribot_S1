#!/usr/bin/env python3
"""Actual Nav2 planner/controller/goal plugins with an isolated ideal base.
Only localhost Domain 128; no SDK import or connection. This is not braking data.
"""
import json,math,os,signal,subprocess,time,argparse
from pathlib import Path
import yaml


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--map');ap.add_argument('--start',nargs=3,type=float,default=[2.,2.,0.]);ap.add_argument('--goal',nargs=3,type=float,default=[5.,2.,0.]);ap.add_argument('--plan-only',action='store_true');a=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='128' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('Requires isolated localhost Domain 128')
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid,Odometry
    from nav2_msgs.action import ComputePathToPose,FollowPath
    from geometry_msgs.msg import PoseStamped,TransformStamped,Twist
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import String
    out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    rclpy.init();n=rclpy.create_node('stop_turn_probe');procs=[];logs=[];rows=[];stages=[]
    pose=list(a.start);command=[0.,0.];last_command=0.;last=time.monotonic();idle_since=last
    drop_feedback=False
    sdk=n.create_publisher(String,'/executor_status',10);tf=n.create_publisher(TFMessage,'/tf',100);odom=n.create_publisher(Odometry,'/odom',10)
    maps=n.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.05;grid.info.width=160;grid.info.height=120;grid.info.origin.orientation.w=1.
    grid.data=[0]*19200
    for y in range(120):
        for x in range(160):
            if x in (0,159) or y in (0,119) or (60<=x<=80 and 20<=y<=50):grid.data[y*160+x]=100
    if a.map:
        import numpy as np
        from PIL import Image
        metadata=yaml.safe_load(Path(a.map).read_text());pixels=np.asarray(Image.open(Path(a.map).parent/metadata['image']).convert('L'),dtype=float)/255.
        occupied=pixels if metadata.get('negate',0) else 1.-pixels
        cells=np.full(pixels.shape,-1,dtype=int);cells[occupied<metadata['free_thresh']]=0;cells[occupied>metadata['occupied_thresh']]=100
        grid.info.height,grid.info.width=pixels.shape;grid.info.resolution=metadata['resolution']
        grid.info.origin.position.x=float(metadata['origin'][0]);grid.info.origin.position.y=float(metadata['origin'][1])
        assert metadata['origin'][2]==0,'Probe supports axis-aligned maps'
        grid.data=np.flipud(cells).ravel().tolist()
    def commanded(m):
        nonlocal command,last_command
        command=[m.linear.x,m.angular.z];last_command=time.monotonic()
        assert abs(command[0])<1e-10 or abs(command[1])<1e-10,'Controller issued a moving arc'
    def phase(m):
        s=json.loads(m.data)
        if not stages or s['stage']!=stages[-1]['stage']:stages.append(dict(time=time.monotonic(),pose=list(pose),**s))
    n.create_subscription(Twist,'/cmd_vel',commanded,10);n.create_subscription(String,'/route_execution',phase,10)
    def tick():
        nonlocal last,idle_since
        now=time.monotonic();dt=min(now-last,.05);last=now
        v,w=command if now-last_command<.3 else [0.,0.]
        pose[0]+=v*dt*math.cos(pose[2]);pose[1]+=v*dt*math.sin(pose[2]);pose[2]+=w*dt
        if abs(v)+abs(w)>1e-8:idle_since=now
        stamp=n.get_clock().now().to_msg()
        tr=TransformStamped();tr.header.frame_id='map';tr.header.stamp=stamp;tr.child_frame_id='base_link'
        tr.transform.translation.x=pose[0];tr.transform.translation.y=pose[1];tr.transform.rotation.z=math.sin(pose[2]/2);tr.transform.rotation.w=math.cos(pose[2]/2)
        tf.publish(TFMessage(transforms=[tr]))
        m=Odometry();m.header=tr.header;m.child_frame_id='base_link';m.pose.pose.position.x=pose[0];m.pose.pose.position.y=pose[1];m.pose.pose.orientation=tr.transform.rotation;m.twist.twist.linear.x=v;m.twist.twist.angular.z=w;odom.publish(m)
        if not drop_feedback:sdk.publish(String(data=json.dumps(dict(sent=now,task='isolated-test',rights=True,fault=None,holding=False,stationary=now-idle_since>=.15,command=[v,w]))))
        rows.append(dict(time=now,pose=list(pose),command=[v,w]))
    n.create_timer(.02,tick)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.005)
    def await_future(f,timeout=15):
        end=time.monotonic()+timeout
        while not f.done():
            if time.monotonic()>end:raise RuntimeError('Action/service timeout')
            spin(.01)
        return f.result()
    cost=dict(global_frame='map',robot_base_frame='base_link',update_frequency=10.,publish_frequency=2.,resolution=.05,
        width=8,height=6,rolling_window=False,track_unknown_space=True,transform_tolerance=.3,
        footprint='[[0.335,0.33],[0.335,-0.33],[-0.335,-0.33],[-0.335,0.33]]',footprint_padding=.02,
        plugins=['static_layer','inflation_layer'],static_layer=dict(plugin='nav2_costmap_2d::StaticLayer',map_topic='/map',map_subscribe_transient_local=True),
        inflation_layer=dict(plugin='nav2_costmap_2d::InflationLayer',inflation_radius=.2,cost_scaling_factor=5.))
    def launch(name,package,p,extra):
        path=out/(name+'.yaml');data={name:{'ros__parameters':p},**extra};path.write_text(yaml.safe_dump(data))
        log=(out/(name+'.log')).open('w');logs.append(log)
        procs.append(subprocess.Popen(['/opt/ros/humble/lib/'+package+'/'+name,'--ros-args','--params-file',str(path)],stdout=log,stderr=subprocess.STDOUT))
    def activate(name):
        client=n.create_client(ChangeState,'/'+name+'/change_state')
        if not client.wait_for_service(timeout_sec=15):raise RuntimeError('Missing lifecycle '+name)
        for ident in (1,3):
            req=ChangeState.Request();req.transition.id=ident;f=client.call_async(req)
            if not await_future(f).success:raise RuntimeError('Lifecycle refused '+name)
    def goal(x,y,yaw):
        p=PoseStamped();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg();p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2);return p
    try:
        launch('planner_server','nav2_planner',dict(planner_plugins=['GridBased'],GridBased=dict(plugin='towergo::StopTurnPlanner')),{'global_costmap':{'global_costmap':{'ros__parameters':cost}}})
        launch('controller_server','nav2_controller',dict(controller_plugins=['FollowPath'],controller_frequency=20.,failure_tolerance=.3,
            FollowPath=dict(plugin='towergo::StopTurnController',desired_linear_vel=.3),goal_checker_plugins=['goal_checker'],goal_checker=dict(plugin='towergo::StopTurnGoalChecker'),
            progress_checker_plugin='progress_checker',progress_checker=dict(plugin='nav2_controller::SimpleProgressChecker',required_movement_radius=.02,movement_time_allowance=120.),odom_topic='/odom'),{'local_costmap':{'local_costmap':{'ros__parameters':cost}}})
        maps.publish(grid);spin(.3);activate('planner_server');activate('controller_server');spin(.5)
        plan=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');follow=ActionClient(n,FollowPath,'/follow_path')
        assert plan.wait_for_server(timeout_sec=10) and follow.wait_for_server(timeout_sec=10)
        req=ComputePathToPose.Goal();req.goal=goal(*a.goal);req.start=goal(*pose);req.use_start=True;req.planner_id='GridBased'
        handle=await_future(plan.send_goal_async(req));assert handle.accepted
        response=await_future(handle.get_result_async());assert response.status==4
        path=response.result.path
        if not a.plan_only:assert len(path.poses)>=3,'Obstacle was not routed around'
        vertices=[[p.pose.position.x,p.pose.position.y] for p in path.poses]
        (out/'path.json').write_text(json.dumps(vertices))
        print('planned_corners',vertices,flush=True)
        if a.plan_only:
            summary=dict(status='passed',kind='plan_only_no_motion',map=a.map,start=a.start,goal=a.goal,vertices=vertices,straight_segments=len(vertices)-1)
            (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True);return
        req=FollowPath.Goal();req.path=path;req.controller_id='FollowPath';req.goal_checker_id='goal_checker'
        h=await_future(follow.send_goal_async(req));assert h.accepted
        result=h.get_result_async();deadline=time.monotonic()+180;progress=time.monotonic()
        while not result.done():
            spin(.05)
            if time.monotonic()>deadline:raise RuntimeError('Route did not finish')
            if time.monotonic()-progress>=15:
                progress=time.monotonic();print('pose',pose,'stage',stages[-1] if stages else None,flush=True)
        response=result.result();assert response.status==4, str(response)
        spin(.3)
        position=math.dist(pose[:2],[5,2]);yaw=abs(math.atan2(math.sin(pose[2]),math.cos(pose[2])))
        assert position<=.15 and yaw<=.08
        assert sum(s['stage']=='straight' for s in stages)==len(vertices)-1,'Unexpected extra stop/realign inside ideal straight segment'
        assert all(s['stationary_confirmed'] for s in stages if s['stage'] in ('straight','turning','final_turn'))
        assert any(s['stage']=='stopping_at_corner' for s in stages)
        summary=dict(status='passed',kind='real_nav2_with_ideal_base_no_sdk',corners=vertices,position_error_m=position,yaw_error_rad=yaw,samples=len(rows),mixed_velocity_samples=sum(abs(r['command'][0])>1e-8 and abs(r['command'][1])>1e-8 for r in rows),straight_segments=sum(s['stage']=='straight' for s in stages),stages=list(stages))
        # Failure tests use new explicit actions, never the physical executor.
        def turn_action(yaw):
            req=FollowPath.Goal();req.path=type(path)();req.path.header.frame_id='map'
            req.path.poses=[goal(*pose),goal(pose[0],pose[1],yaw)];req.controller_id='FollowPath';req.goal_checker_id='goal_checker'
            handle=await_future(follow.send_goal_async(req));assert handle.accepted
            deadline=time.monotonic()+10
            while abs(command[1])<1e-4:
                spin(.05)
                if time.monotonic()>deadline:raise RuntimeError('Turn did not start')
            return handle
        h=turn_action(pose[2]+.7);drop_feedback=True
        aborted=await_future(h.get_result_async(),5);assert aborted.status==6
        spin(.4);assert abs(command[0])+abs(command[1])<1e-8
        drop_feedback=False;before=list(pose);spin(.8)
        assert math.dist(before,pose)<1e-6,'Feedback recovery resumed an old action'
        h=turn_action(pose[2]+.7);await_future(h.cancel_goal_async())
        canceled=await_future(h.get_result_async(),5);assert canceled.status==5
        spin(.4);assert abs(command[0])+abs(command[1])<1e-8
        summary.update(stale_feedback_aborts=True,feedback_recovery_does_not_resume=True,manual_cancel_stops=True)
        (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
    finally:
        (out/'trace.json').write_text(json.dumps(rows))
        for p in procs:
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in procs:
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        for f in logs:f.close()
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
