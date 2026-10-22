"""Offline acceptance runner. Ground truth is consumed only by this evaluator."""
import argparse, json, math, os, signal, subprocess, threading, time, hashlib
from pathlib import Path
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from astribot_nav_api.navigator import Navigator
from .world import goals,geometry,wall_distances,wrap

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def pose(m): return np.array([m.position.x,m.position.y,yaw(m.orientation)])
def wait_until(predicate,seconds=60.):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if predicate():return True
        time.sleep(.05)
    return False

def completed(future,seconds=15.):
    if not wait_until(future.done,seconds): raise TimeoutError('ROS request timed out')
    return future.result()

class Evaluator(Navigator):
    def __init__(self):
        super().__init__(use_sim_time=True)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.truth=[];self.estimate=[];self.map=None;self.route_done=False;self.collisions=0;self.actual_velocity=(0.,0.);self.last_truth_stamp=0.;self.tf_misses=0
        self.create_subscription(Odometry,'/ground_truth/odom',self.on_truth,50)
        self.create_subscription(Bool,'/ground_truth/collision',self.on_collision,10)
        self.create_subscription(Bool,'/experiment/route_done',lambda m:setattr(self,'route_done',m.data),10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,'/map',lambda m:setattr(self,'map',m),qos)
        self.initial=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.command=self.create_publisher(Twist,'/cmd_vel_nav',10)
        self.create_timer(.2,self.sample)
    def on_truth(self,m):
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        self.truth.append([t,*pose(m.pose.pose)])
        self.last_truth_stamp=t;self.actual_velocity=(m.twist.twist.linear.x,m.twist.twist.angular.z)
    def on_collision(self,m): self.collisions+=int(m.data)
    def sample(self):
        if len(self.truth)<30:return
        row=self.truth[-26] # Allow TF producers to catch up; query the exact sample time.
        try:
            tf=self.buffer.lookup_transform('map','base_link',Time(seconds=row[0]))
            e=np.array([tf.transform.translation.x,tf.transform.translation.y,yaw(tf.transform.rotation)])
            self.estimate.append([row[0],*e,*row[1:]])
        except Exception:self.tf_misses+=1
    def initialize(self):
        m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
        m.pose.pose.position.x=.3;m.pose.pose.orientation.z=math.sin(math.radians(10)/2);m.pose.pose.orientation.w=math.cos(math.radians(10)/2)
        m.pose.covariance[0]=.25;m.pose.covariance[7]=.25;m.pose.covariance[35]=math.radians(15)**2
        self.initial.publish(m)
    def switch(self,name,value):
        client=self.create_client(SetBool,'/experiment/'+name)
        if not client.wait_for_service(timeout_sec=10.):raise TimeoutError(name)
        req=SetBool.Request();req.data=value;result=completed(client.call_async(req));self.destroy_client(client)
        if not result.success:raise RuntimeError(name+' failed')
    def wait_active(self):
        from lifecycle_msgs.srv import GetState
        for name in ('amcl','planner_server','controller_server','bt_navigator'):
            client=self.create_client(GetState,'/'+name+'/get_state')
            if not client.wait_for_service(timeout_sec=30.):raise RuntimeError(name+' lifecycle service unavailable')
            end=time.monotonic()+60.
            while time.monotonic()<end:
                result=completed(client.call_async(GetState.Request()),5.)
                if result.current_state.id==3:break
                time.sleep(.1)
            else:raise RuntimeError(name+' did not become active')
            self.destroy_client(client)
    def metrics(self,scenario):
        result={'trajectory_samples':len(self.estimate),'tf_misses':self.tf_misses,'collisions':self.collisions}
        if self.estimate:
            a=np.array(self.estimate)
            result['position_rmse_m']=float(np.sqrt(np.mean(np.sum((a[:,1:3]-a[:,4:6])**2,axis=1))))
        if self.map is not None:
            m=self.map;cells=np.array(m.data).reshape(m.info.height,m.info.width);iy,ix=np.where(cells>=65)
            a=yaw(m.info.origin.orientation);pts=np.stack(((ix+.5)*m.info.resolution,(iy+.5)*m.info.resolution),axis=1)
            pts=pts@np.array([[math.cos(a),math.sin(a)],[-math.sin(a),math.cos(a)]])+np.array([m.info.origin.position.x,m.info.origin.position.y])
            if len(pts):result['wall_error_p95_m']=float(np.percentile(wall_distances(pts,geometry(scenario)),95))
            result['observed_cells']=int(np.sum(cells>=0))
        return result

class Run:
    def __init__(self,args):
        self.args=args;self.output=Path(args.output).resolve()
        if self.output.exists() and any(self.output.iterdir()):raise RuntimeError('Use a new empty output directory')
        self.output.mkdir(parents=True,exist_ok=True)
        self.node=Evaluator();self.thread=threading.Thread(target=rclpy.spin,args=(self.node,),daemon=True);self.thread.start()
        self.log=(self.output/'launch.log').open('w');self.process=None;self.recorder=None
        self.report={'source_sha256':self.source_hash(),'image_id':os.environ.get('TOWERGO_IMAGE_ID','unknown'),'mode':args.mode,'scenario':args.scenario,'seed':args.seed,'scan_source':args.scan_source,'noise':args.noise,'rate':args.rate,'sdk_used':False,'passed':False}
        versions=Path('/towergo/dependency-versions.txt')
        if versions.exists():(self.output/'dependency-versions.txt').write_text(versions.read_text())
    @staticmethod
    def source_hash():
        root=Path('/towergo/src/astribot_navigation');h=hashlib.sha256()
        for p in sorted(root.rglob('*')):
            if p.is_file() and '__pycache__' not in str(p) and '.pytest_cache' not in str(p):
                h.update(str(p.relative_to(root)).encode());h.update(p.read_bytes())
        return h.hexdigest()
    def start(self):
        mode='mapping' if self.args.mode=='mapping' else 'navigation'
        cmd=['ros2','launch','astribot_nav_bringup','offline_'+mode+'.launch.py',f'scenario:={self.args.scenario}',f'seed:={self.args.seed}',f'scan_source:={self.args.scan_source}',f'noise:={str(self.args.noise).lower()}',f'rate:={self.args.rate}']
        if mode=='navigation':cmd+=['map:='+str(Path(self.args.map).resolve())]
        self.report['command']=cmd
        if self.args.mode=='mapping' and self.args.record:
            self.recorder=subprocess.Popen(['ros2','bag','record','--use-sim-time','-s','mcap','-o',str(self.output/'raw_bag'),'/livox/lidar_front','/livox/lidar_back','/odom','/tf','/tf_static','/ground_truth/odom'],stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
            time.sleep(2.)
        self.process=subprocess.Popen(cmd,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        if not wait_until(lambda:bool(self.node.truth),30):raise RuntimeError('Simulator did not publish truth')
    def mapping(self):
        if not wait_until(lambda:self.node.route_done or self.process.poll() is not None,240/self.args.rate+120):raise TimeoutError('Mapping route did not finish')
        if not self.node.route_done:raise RuntimeError('Launch exited before mapping completed')
        time.sleep(3)
        if self.node.map is None:raise RuntimeError('No SLAM map')
        path=self.output/'map'
        subprocess.run(['ros2','run','nav2_map_server','map_saver_cli','-f',str(path),'--ros-args','-p','use_sim_time:=true','-p','save_map_timeout:=20.0'],check=True,timeout=45,stdout=self.log,stderr=subprocess.STDOUT)
        from slam_toolbox.srv import SerializePoseGraph
        client=self.node.create_client(SerializePoseGraph,'/slam_toolbox/serialize_map')
        if not client.wait_for_service(timeout_sec=15.):raise RuntimeError('serialize_map unavailable')
        req=SerializePoseGraph.Request();req.filename=str(self.output/'posegraph');response=completed(client.call_async(req),30.)
        self.report['serialize_result']=response.result
        if response.result!=0:raise RuntimeError('Pose graph serialization failed')
        import yaml
        config=yaml.safe_load(path.with_suffix('.yaml').read_text());image=path.parent/config['image']
        if not image.is_file():raise RuntimeError('Saved map image missing')
        self.report.update(self.node.metrics(self.args.scenario));self.report['map_yaml']=str(path.with_suffix('.yaml'))
        audit_path=self.output/'map-audit.json'
        from .map_audit import audit
        self.report['map_audit']=audit(path.with_suffix('.yaml'),self.args.scenario)
        audit_path.write_text(json.dumps(self.report['map_audit'],indent=2))
        self.report['passed']=self.report['map_audit']['passed'] and self.report.get('position_rmse_m',999)<=.15 and self.report.get('wall_error_p95_m',999)<=.15 and self.node.collisions==0
    def localize(self):
        if not self.node.client.wait_for_server(timeout_sec=60.):raise RuntimeError('Nav2 action server unavailable')
        self.node.initialize()
        if not wait_until(lambda:bool(self.node.estimate),30.):raise RuntimeError('No localization TF after initial pose')
        self.node.wait_active()
        # Sensors and lifecycle nodes must be ready before the timed motion test.
        self.node.initialize();time.sleep(.5)
        self.node.estimate.clear()
        # Actual simulated motion, independently driven, for the 15 s localization test.
        start=self.node.last_truth_stamp;converged=None;wall_start=time.monotonic()
        while self.node.last_truth_stamp-start<15.:
            if time.monotonic()-wall_start>30. or (self.process and self.process.poll() is not None):raise RuntimeError('Clock stopped or launch exited during localization')
            m=Twist();m.angular.z=.1;self.node.command.publish(m)
            if self.node.estimate:
                row=self.node.estimate[-1]
                if row[0]>=start and math.hypot(row[1]-row[4],row[2]-row[5])<=.15 and abs(wrap(row[3]-row[6]))<=math.radians(5):
                    converged=row[0]-start;break
            time.sleep(.02)
        self.node.command.publish(Twist());time.sleep(.3)
        self.report['localization_converged_seconds']=converged
        if converged is None:raise RuntimeError('Localization did not converge within 15 simulated seconds')
        self.node.wait_active()
    def navigation(self):
        self.localize();results=[]
        for goal in goals(self.args.scenario):
            self.node.goal_handle=None;self.node.result_future=None
            handle=completed(self.node.send(*goal),30.)
            if not handle.accepted:raise RuntimeError('Goal rejected')
            if not wait_until(lambda:self.node.result_future is not None,5.):raise RuntimeError('No action result future')
            result=completed(self.node.result_future,180/self.args.rate+60.)
            if not self.node.truth:raise RuntimeError('No ground truth')
            truth=self.node.truth[-1][1:]
            position_error=math.hypot(truth[0]-goal[0],truth[1]-goal[1]);angle_error=abs(wrap(truth[2]-goal[2]))
            item={'goal':goal,'status':result.status,'position_error_m':position_error,'yaw_error_deg':math.degrees(angle_error)}
            item['passed']=result.status==4 and position_error<=.2 and angle_error<=math.radians(10)
            results.append(item)
            self.report['goals']=results
        self.report.update(self.node.metrics(self.args.scenario));self.report['passed']=all(x['passed'] for x in results) and self.node.collisions==0
    def faults(self):
        self.localize();out={}
        handle=completed(self.node.send(*goals(self.args.scenario)[0]),30.)
        if not handle.accepted:raise RuntimeError('Fault test goal rejected')
        if not wait_until(lambda:abs(self.node.actual_velocity[0])+abs(self.node.actual_velocity[1])>.01,20.):raise RuntimeError('No motion before cancellation')
        begin=self.node.last_truth_stamp;cancel=self.node.cancel()
        if cancel is None:raise RuntimeError('No cancellable goal')
        response=completed(cancel)
        stopped=wait_until(lambda:sum(abs(v) for v in self.node.actual_velocity)<.001,5.)
        out['cancel']={'accepted':bool(response.goals_canceling),'stop_seconds':self.node.last_truth_stamp-begin,'passed':stopped and self.node.last_truth_stamp-begin<=.5}
        if self.node.result_future:completed(self.node.result_future,15.)
        # Controlled synthetic commands establish movement before sensor loss.
        for _ in range(20):m=Twist();m.angular.z=.1;self.node.command.publish(m);time.sleep(.02)
        begin=self.node.last_truth_stamp;self.node.switch('sensors_enabled',False)
        for _ in range(50):m=Twist();m.angular.z=.1;self.node.command.publish(m);time.sleep(.02)
        out['sensor_loss']={'passed':sum(abs(v) for v in self.node.actual_velocity)<.001,'elapsed_sim_seconds':self.node.last_truth_stamp-begin}
        self.node.command.publish(Twist());self.node.switch('sensors_enabled',True)
        self.node.switch('pause',True);stamp=self.node.last_truth_stamp;time.sleep(.6)
        frozen=stamp==self.node.last_truth_stamp
        self.node.switch('pause',False);time.sleep(.6)
        out['pause_resume']={'passed':frozen and sum(abs(v) for v in self.node.actual_velocity)<.001}
        self.node.switch('block_path',True)
        self.node.goal_handle=None;self.node.result_future=None
        handle=completed(self.node.send(3.,0.,0.),30.)
        if handle.accepted:
            if not wait_until(lambda:self.node.result_future and self.node.result_future.done(),60/self.args.rate+30):
                f=self.node.cancel()
                if f:completed(f)
                out['blocked']={'passed':False,'reason':'Nav2 did not fail within bound; cancelled by evaluator'}
            else:
                result=self.node.result_future.result();out['blocked']={'passed':result.status==6,'status':result.status}
        else:out['blocked']={'passed':True,'reason':'Rejected blocked goal'}
        self.report['faults']=out;self.report['passed']=all(v['passed'] for v in out.values()) and self.node.collisions==0
    def close(self):
        if self.recorder:
            os.killpg(self.recorder.pid,signal.SIGINT);self.recorder.wait(timeout=20)
        if self.process:
            os.killpg(self.process.pid,signal.SIGINT)
            try:self.process.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(self.process.pid,signal.SIGTERM);self.process.wait(timeout=10)
        self.report['collisions']=self.node.collisions
        (self.output/'report.json').write_text(json.dumps(self.report,indent=2,allow_nan=False))
        np.savetxt(self.output/'truth.csv',np.array(self.node.truth),delimiter=',',header='time,x,y,yaw')
        np.savetxt(self.output/'estimated.csv',np.array(self.node.estimate),delimiter=',',header='time,x,y,yaw,truth_x,truth_y,truth_yaw')
        self.node.destroy_node();rclpy.shutdown();self.thread.join(timeout=3);self.log.close()

def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['mapping','navigation','faults']);p.add_argument('--scenario',choices=['room','corridor','obstacles'],default='room');p.add_argument('--seed',type=int,default=42);p.add_argument('--scan-source',choices=['front','dual'],default='front');p.add_argument('--noise',action='store_true');p.add_argument('--record',action='store_true');p.add_argument('--rate',type=float,default=2.);p.add_argument('--map');p.add_argument('--output',required=True);args=p.parse_args()
    if args.mode!='mapping' and not args.map:p.error('--map is required for navigation and faults')
    if args.rate<=0 or args.rate>8:p.error('--rate must be in (0,8]')
    if os.environ.get('ROS_DOMAIN_ID')!='125':raise RuntimeError('Offline ROS_DOMAIN_ID=125 required')
    rclpy.init(args=[]);run=Run(args)
    try:
        run.start();getattr(run,args.mode)()
    except Exception as e:
        run.report['error']=str(e);run.report['passed']=False
    finally:run.close()
    print(json.dumps(run.report,indent=2))
    if not run.report['passed']:raise SystemExit(1)
