"""Real-only web action coordinator. All mutations execute on the ROS thread."""
import json
import math
import queue
import secrets
import threading
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path as NavPath
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import SetParametersAtomically
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from astribot_nav_sim.grid_world import GridWorld
from .geometry import safe_pose
from .core import Lease, speed, controller_parameters, limited
from .sensors import Sensors,NS,FRAME


class Web(Node):
    def __init__(self,config,filename,mode,run):
        super().__init__('navigation_web',namespace=NS.rstrip('/'))
        self.config,self.mode,self.run=config,mode,Path(run)
        self.grid=GridWorld(filename);self.token=secrets.token_urlsafe(32)
        self.lease=Lease();self.lock=threading.RLock();self.requests=queue.Queue(maxsize=32)
        self.stop_request=threading.Event();self.request_epoch=0
        self.sensors=Sensors(self,config);self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.action=ActionClient(self,NavigateToPose,NS+'navigate_to_pose')
        self.parameters=self.create_client(SetParametersAtomically,NS+'controller_server/set_parameters_atomically')
        self.lifecycle=self.create_client(GetState,NS+'bt_navigator/get_state')
        self.initial=self.create_publisher(PoseWithCovarianceStamped,NS+'initialpose',1)
        self.control=self.create_publisher(String,NS+'executor_control',1)
        self.create_subscription(String,NS+'executor_status',self.executor_status,1)
        self.create_subscription(PoseWithCovarianceStamped,NS+'amcl_pose',self.amcl,10)
        self.create_subscription(Twist,NS+'cmd_vel_nav',self.command,1)
        for topic,key in [('plan','plan'),('lookahead_collision_arc','local_plan')]:
            self.create_subscription(NavPath,NS+topic,lambda m,k=key:self.path(m,k),1)
        self.sequence=0;self.goal_handle=None;self.executor_state={};self.executor_at=-1e20
        self.cmd=(0.,0.);self.cmd_at=-1e20;self.amcl_at=-1e20;self.amcl_cov=math.inf
        self.amcl_stamp=-1e20;self.initial_stamp=math.inf
        self.active_lifecycle=False;self.lifecycle_at=-1e20;self.lifecycle_pending=False
        self.phase='idle';self.phase_at=0.;self.params_ok=False;self.goal_sent=False
        self.initial_message=None;self.initial_at=0.;self.initial_last=0.
        self.still_since=None;self.result_pending=None;self.pending_goal=None
        self.state=dict(status='真机观察模式：等待传感器' if mode=='observe' else '真机：等待传感器与执行器',
                        mode=mode,replay=bool(self.get_parameter('use_sim_time').value),ready=False,initialized=False,busy=False,error=None,pose=None,start=None,goal=None,
                        velocity=[0.,0.],command_velocity=[0.,0.],requested_speed=.05,plan=[],local_plan=[],trajectory=[],
                        obstacles=[],distance_remaining=None,diagnostics=[],task=None,owner=None,result=None,
                        footprint=config['footprint'],max_speed=config['max_speed'])
        self.log=(self.run/'navigation.jsonl').open('a',buffering=1)
        self.sensor_epoch=0
        self.last_sample=0.;self.last_lifecycle=0.
        self.create_timer(.05,self.tick)

    def event(self,text):
        self.state['diagnostics']=(self.state['diagnostics']+[text])[-40:]
        self.log.write(json.dumps(dict(time=time.time(),event=text,task=self.state['task']),ensure_ascii=False)+'\n')

    def send(self,op,**kwargs):
        self.sequence+=1
        self.control.publish(String(data=json.dumps(dict(op=op,sequence=self.sequence,sent=time.monotonic(),**kwargs))))

    def executor_status(self,msg):
        try:
            data=json.loads(msg.data)
            if 0<=time.monotonic()-data['sent']<.2:
                self.executor_state=data;self.executor_at=time.monotonic()
        except (ValueError,KeyError,TypeError):pass

    def amcl(self,msg):
        now=self.get_clock().now().nanoseconds*1e-9
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not -.05<=now-stamp<=.5:return
        self.amcl_stamp=stamp;self.amcl_at=time.monotonic()
        c=msg.pose.covariance
        self.amcl_cov=max(c[0],c[7],c[35]) if all(math.isfinite(c[i]) and c[i]>=0 for i in (0,7,35)) else math.inf

    def command(self,msg):
        try:
            if any(v!=0 for v in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y)):
                raise ValueError('不允许横移或其他轴的指令')
            self.cmd=limited(msg.linear.x,msg.angular.z,self.state['requested_speed'])
            self.cmd_at=time.monotonic()
        except ValueError as exc:self.stop(str(exc))

    def path(self,msg,key):
        if msg.header.frame_id not in (FRAME['map'],FRAME['odom']):return
        try:
            t=self.tf.lookup_transform(FRAME['map'],msg.header.frame_id,rclpy.time.Time())
            q=t.transform.rotation;r=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();p=t.transform.translation
            pts=np.array([[v.pose.position.x,v.pose.position.y,v.pose.position.z] for v in msg.poses])
            self.state[key]=((pts@r.T)+[p.x,p.y,p.z])[:,:2].tolist() if len(pts) else []
        except Exception:pass

    def valid(self,task):
        return self.lease.valid(task,time.monotonic())

    def stop(self,reason='用户取消',result='canceled'):
        idle_cancel=(result=='canceled' and self.phase=='idle' and not self.lease.active and not self.state['busy'])
        task=self.lease.stop()
        self.send('stop')
        if self.goal_handle:
            self.goal_handle.cancel_goal_async();self.goal_handle=None
        self.params_ok=False;self.cmd=(0.,0.);self.cmd_at=-1e20
        self.initial_message=None
        self.phase='stopping';self.phase_at=time.monotonic();self.still_since=None
        self.result_pending=result;self.state.update(status='正在停车：'+reason,command_velocity=[0.,0.],error=reason if result=='fault' else None)
        if idle_cancel:
            self.phase='idle'
            self.state.update(busy=False,result=result,status='已取消 · 当前没有活动导航任务',error=None)
        self.event(reason)
        # Task remains visible for audit, but the lease is irrevocably invalidated.

    def localize(self,obj):
        if not self.sensors.healthy():raise ValueError('请先等待双雷达、IMU 和里程计就绪')
        if self.state['busy'] or self.phase=='stopping':raise ValueError('请等待停车完成')
        self.lease.claim(obj['owner'],time.monotonic())
        values=self.values(obj,check_footprint=False)
        msg=PoseWithCovarianceStamped();msg.header.frame_id=FRAME['map']
        msg.pose.pose.position.x,msg.pose.pose.position.y=values[:2]
        msg.pose.pose.orientation.z=math.sin(values[2]/2);msg.pose.pose.orientation.w=math.cos(values[2]/2)
        msg.pose.covariance[0]=msg.pose.covariance[7]=.25;msg.pose.covariance[35]=.07
        self.state.update(initialized=False,error=None,start=values,trajectory=[],status='正在初始化定位；机器人保持静止')
        self.initial_message=msg;self.initial_at=time.monotonic();self.initial_last=0.
        self.initial_stamp=self.sensors.last_pose[0]
        self.initial_message.header.stamp=rclpy.time.Time(nanoseconds=int(self.initial_stamp*1e9)).to_msg()

    def values(self,obj,check_footprint=True):
        values=[float(obj[k]) for k in ('x','y','yaw')]
        if not all(math.isfinite(v) for v in values):raise ValueError('选点必须为有限坐标')
        cell=self.grid.cells([values[:2]])[0];x,y=cell
        if not 0<=x<self.grid.width or not 0<=y<self.grid.height or not self.grid.free[y,x]:
            raise ValueError('选点位于障碍、未知区域或地图之外')
        if check_footprint and not safe_pose(self.grid,values,self.config['footprint'],self.config['footprint_padding']):
            raise ValueError('当前朝向下整机轮廓与地图障碍或未知区域相交')
        return values

    def begin(self,obj):
        if self.mode not in ('hardware','mock'):raise ValueError('观察模式不发送运动任务；需先完成现场验证')
        if not self.state['ready'] or not self.state['initialized'] or self.phase!='idle':raise ValueError('定位、执行器或任务尚未就绪')
        value=speed(obj['speed']);params=controller_parameters(value);values=self.values(obj)
        if value>self.config['max_speed']:raise ValueError('超过配置的导航最高速度')
        if not self.parameters.service_is_ready():raise ValueError('控制器参数服务尚未就绪')
        task=self.lease.begin(obj['owner'],time.monotonic())
        self.state.update(task=task,owner=obj['owner'],busy=True,goal=values,requested_speed=value,result=None,error=None,status='正在确认导航速度')
        self.phase='preparing';self.phase_at=time.monotonic();self.params_ok=False;self.goal_sent=False
        self.cmd=(0.,0.);self.cmd_at=-1e20
        self.send('prepare',task=task,speed=value)
        request=SetParametersAtomically.Request();request.parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in params.items()]
        future=self.parameters.call_async(request)
        def applied(f):
            if not self.valid(task):return
            try:
                result=f.result().result
                if not result.successful:raise RuntimeError(result.reason)
                self.params_ok=True
            except Exception as exc:self.stop('控制器限速失败：'+str(exc),'fault')
        future.add_done_callback(applied)

    def submit_goal(self):
        task=self.lease.task;values=self.state['goal']
        msg=NavigateToPose.Goal();msg.pose.header.frame_id=FRAME['map'];msg.pose.header.stamp=self.get_clock().now().to_msg()
        msg.pose.pose.position.x,msg.pose.pose.position.y=values[:2]
        msg.pose.pose.orientation.z=math.sin(values[2]/2);msg.pose.pose.orientation.w=math.cos(values[2]/2)
        self.goal_sent=True;self.phase='accepting';self.phase_at=time.monotonic()
        def feedback(m):
            if self.valid(task):self.state['distance_remaining']=float(m.feedback.distance_remaining)
        future=self.action.send_goal_async(msg,feedback_callback=feedback)
        def accepted(f):
            try:
                handle=f.result()
                if not self.valid(task):
                    if handle.accepted:handle.cancel_goal_async()
                    return
                if not handle.accepted:raise RuntimeError('Nav2 拒绝目标')
                self.goal_handle=handle;self.phase='running';self.cmd_at=time.monotonic()
                self.state['status']='真机导航中'
                def finished(done):
                    if not self.valid(task):return
                    try:
                        result=done.result().status
                        if result==4:
                            pose=self.state['pose'];goal=self.state['goal']
                            if pose is None or math.dist(pose[:2],goal[:2])>.15 or abs(math.atan2(math.sin(pose[2]-goal[2]),math.cos(pose[2]-goal[2])))>.08:
                                raise RuntimeError('Nav2 返回成功但当前位姿不满足到点判据')
                        self.stop('目标已到达，等待底盘静止' if result==4 else 'Nav2 任务结束', 'succeeded' if result==4 else 'failed')
                    except Exception as exc:self.stop(str(exc),'fault')
                handle.get_result_async().add_done_callback(finished)
            except Exception as exc:
                if self.valid(task):self.stop(str(exc),'fault')
        future.add_done_callback(accepted)

    def tick(self):
        with self.lock:
            self._tick()

    def _tick(self):
        now=time.monotonic();ros_now=self.get_clock().now().nanoseconds*1e-9
        epoch=self.sensors.geometry.continuity_epoch
        if epoch!=self.sensor_epoch:
            self.sensor_epoch=epoch
            if self.lease.active:self.stop('定位数据中断，任务已取消','fault')
            self.initial_message=None;self.amcl_at=-1e20;self.amcl_stamp=-1e20
            self.state.update(initialized=False,pose=None,status='定位数据已恢复，请重新初始化位置与朝向')
            self.event('定位历史发生中断：保留里程计原点，禁止自动恢复任务')
        if self.stop_request.is_set():self.stop_request.clear();self.stop()
        try:
            while True:
                name,obj,epoch=self.requests.get_nowait()
                if epoch!=self.request_epoch:continue
                try:
                    if name=='localize':self.localize(obj)
                    elif name=='goal':self.begin(obj)
                except Exception as exc:self.state['error']=str(exc);self.event(str(exc))
        except queue.Empty:pass
        if now-self.last_lifecycle>1 and not self.lifecycle_pending and self.lifecycle.service_is_ready():
            self.last_lifecycle=now;self.lifecycle_pending=True
            def reply(f):
                self.lifecycle_pending=False
                try:self.active_lifecycle=f.result().current_state.id==3;self.lifecycle_at=time.monotonic()
                except Exception:self.active_lifecycle=False
            self.lifecycle.call_async(GetState.Request()).add_done_callback(reply)
        tf_ok=False
        try:
            t=self.tf.lookup_transform(FRAME['map'],FRAME['base'],rclpy.time.Time())
            stamp=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
            tf_ok=-.35<=ros_now-stamp<=.5
            q=t.transform.rotation;p=t.transform.translation
            pose=[p.x,p.y,Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')[2]]
            self.state['pose']=pose
            if tf_ok and (not self.state['trajectory'] or math.dist(pose[:2],self.state['trajectory'][-1][:2])>.02):
                self.state['trajectory']=(self.state['trajectory']+[pose])[-10000:]
        except Exception:pass
        self.state['sensors_ready']=self.sensors.healthy()
        healthy=self.state['sensors_ready'] and tf_ok and now-self.amcl_at<.5 and self.amcl_cov<.25
        execution=(self.mode=='observe' or (now-self.executor_at<.3 and not self.executor_state.get('fault') and self.executor_state.get('rights')))
        self.state['ready_checks']=dict(sensors=self.state['sensors_ready'],tf=tf_ok,amcl_age=round(now-self.amcl_at,3),amcl_covariance=self.amcl_cov if math.isfinite(self.amcl_cov) else None,executor=bool(execution),lifecycle=self.active_lifecycle,lifecycle_age=round(now-self.lifecycle_at,3))
        self.state['ready']=bool(healthy and execution and self.active_lifecycle and now-self.lifecycle_at<3)
        self.state['velocity']=self.sensors.velocity
        self.state['localization_covariance']=self.amcl_cov if math.isfinite(self.amcl_cov) else None
        if self.mode=='mock':self.state['mode_label']='录包回放 · 内存执行器（不控制机器人）'
        elif self.state['replay']:self.state['mode_label']='录包回放 · 只读观察'
        else:self.state['mode_label']='真机观察' if self.mode=='observe' else '真机控制'
        self.state['sensor_health']={k:round(now-v,3) for k,v in self.sensors.received.items()}
        self.state['executor']=self.executor_state
        self.state['owner']=self.lease.owner;self.state['phase']=self.phase
        if self.initial_message:
            if now-self.initial_at>30:
                self.initial_message=None;self.state['error']='定位初始化超时，请检查雷达与地图对齐'
            elif self.amcl_stamp>self.initial_stamp and healthy and now-self.initial_at>=2:
                self.initial_message=None;self.state.update(initialized=True,error=None,status='定位就绪；观察模式' if self.mode=='observe' else '定位就绪，可以开始导航')
            elif now-self.initial_last>.5 and self.amcl_stamp<=self.initial_stamp:
                self.initial.publish(self.initial_message);self.initial_last=now
        if self.lease.active:
            if not self.valid(self.lease.task):self.stop('控制浏览器失联','fault')
            elif not self.state['ready']:self.stop('定位、传感器或执行器失效','fault')
            elif self.phase in ('preparing','accepting'):
                self.send('command',task=self.lease.task,v=0.,w=0.)
                if self.phase=='stopping' and now-self.phase_at>5:self.stop('导航准备超时','fault')
                elif self.phase=='preparing' and self.params_ok and self.executor_state.get('task')==self.lease.task and self.executor_state.get('speed')==self.state['requested_speed']:
                    self.submit_goal()
            elif self.phase=='running':
                if now-self.cmd_at>=.3:self.stop('Nav2 指令超时','fault')
                else:
                    self.send('command',task=self.lease.task,v=self.cmd[0],w=self.cmd[1]);self.state['command_velocity']=list(self.cmd)
        if self.phase=='stopping':
            still=self.sensors.healthy() and self.sensors.planar_speed<.005 and abs(self.sensors.velocity[1])<.01
            if still:
                if self.still_since is None:self.still_since=now
                elif now-self.still_since>=.5:
                    self.phase='idle';self.state.update(busy=False,result=self.result_pending,status='已静止 · '+str(self.result_pending))
                    if self.state['error']=='未能确认停车，请检查现场状态':self.state['error']=None
            else:self.still_since=None
            if self.phase=='stopping' and now-self.phase_at>5:self.state['error']='未能确认停车，请检查现场状态'
        if self.sensors.error:self.state['error']=self.sensors.error
        if now-self.last_sample>.2:
            self.last_sample=now
            self.update_obstacles()
            self.log.write(json.dumps(dict(time=time.time(),task=self.state['task'],phase=self.phase,pose=self.state['pose'],velocity=self.sensors.velocity,command=self.state['command_velocity'],result=self.state['result']),ensure_ascii=False)+'\n')

    def update_obstacles(self):
        points=[]
        for key,(stamp,cloud) in self.sensors.obstacles.items():
            if self.get_clock().now().nanoseconds*1e-9-stamp>.5:continue
            try:
                t=self.tf.lookup_transform(FRAME['map'],FRAME[key],rclpy.time.Time(seconds=stamp))
                q=t.transform.rotation;p=t.transform.translation
                r=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
                if cloud:points.extend((np.asarray(cloud)@r.T+[p.x,p.y,p.z])[:,:2].tolist())
            except Exception:pass
        self.state['obstacles']=points
        self.state['laser_scan']=[]
        self.state['laser_scan_status']='等待雷达扫描'
        self.state['laser_scan_age']=None
        scan=self.sensors.latest_scan
        if scan is not None:
            stamp,cloud=scan
            age=self.get_clock().now().nanoseconds*1e-9-stamp
            self.state['laser_scan_age']=round(age,3)
            if not -.05<=age<=.5:
                self.state['laser_scan_status']='扫描已过期，暂不叠加'
            elif self.sensors.error:
                self.state['laser_scan_status']='定位数据异常，暂不叠加'
            else:
                try:
                    t=self.tf.lookup_transform(FRAME['map'],FRAME['front'],rclpy.time.Time(seconds=stamp))
                    q=t.transform.rotation;p=t.transform.translation
                    r=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
                    if cloud:self.state['laser_scan']=(np.asarray(cloud)@r.T+[p.x,p.y,p.z])[:,:2].tolist()
                    self.state['laser_scan_status']='实时水平扫描 · '+str(len(cloud))+' 点'
                except Exception:
                    self.state['laser_scan_status']='等待扫描时刻的地图定位变换；请先初始化定位'


def server(node,port):
    html=Path(__file__).with_name('index.html').read_bytes()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def output(self,code,data,kind='application/json'):
            self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.end_headers()
            try:self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_GET(self):
            if self.path=='/':return self.output(200,html.replace(b'__TOKEN__',node.token.encode()),'text/html; charset=utf-8')
            if self.path=='/map.png':return self.output(200,node.grid.png,'image/png')
            if self.path=='/api/state':
                with node.lock:data=dict(node.state,map=node.grid.metadata())
                return self.output(200,json.dumps(data,ensure_ascii=False,allow_nan=False).encode())
            self.output(404,b'{}')
        def do_POST(self):
            if self.headers.get('X-Session-Token')!=node.token:
                return self.output(403,json.dumps({'error':'导航服务已重启或页面会话已失效，请刷新整个页面后重试'},ensure_ascii=False).encode())
            try:
                size=int(self.headers.get('Content-Length',0))
                if not 0<size<=2048:raise ValueError('Invalid request size')
                self.connection.settimeout(1)
                obj=json.loads(self.rfile.read(size));name=self.path.removeprefix('/api/')
                if name=='stop':
                    with node.lock:
                        node.request_epoch+=1;node.lease.stop();node.stop_request.set()
                elif name=='heartbeat':
                    with node.lock:
                        if not node.lease.heartbeat(obj['owner'],obj['task'],time.monotonic()):raise ValueError('控制许可已失效')
                elif name in ('localize','goal'):
                    if name=='goal':controller_parameters(obj['speed'])
                    with node.lock:node.requests.put_nowait((name,obj,node.request_epoch))
                else:raise ValueError('Unknown endpoint')
                self.output(202,b'{"queued":true}')
            except (ValueError,KeyError,TypeError,queue.Full) as exc:self.output(400,json.dumps({'error':str(exc)},ensure_ascii=False).encode())
    result=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    result.daemon_threads=True
    threading.Thread(target=result.serve_forever,daemon=True).start()
    return result
