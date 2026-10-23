"""Deterministic, offline-only kinematic world. No SDK dependency."""
import math, time, os
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMsg
from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Bool
from std_srvs.srv import SetBool
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from .world import geometry,route,raycast,advance,wrap,collides

MAX_LINEAR_SPEED = .1
MAX_ANGULAR_SPEED = .1

class Simulator(Node):
    def __init__(self):
        super().__init__('towergo_simulator')
        if os.environ.get('ROS_DOMAIN_ID')!='125':
            raise RuntimeError('Offline simulator requires ROS_DOMAIN_ID=125')
        for k,v in [('max_linear_speed', .1),('scenario','room'),('seed',42),('noise',False),('mapping',False),('rate',2.),('map_yaml','')]: self.declare_parameter(k,v)
        self.scenario=self.get_parameter('scenario').value
        self.segments=geometry(self.scenario)
        self.rng=np.random.default_rng(self.get_parameter('seed').value)
        self.noise=self.get_parameter('noise').value
        self.mapping=self.get_parameter('mapping').value
        self.path=route(self.scenario); self.waypoint=0
        self.truth=np.zeros(3); self.odom=np.zeros(3); self.tick_count=0
        self.grid = None
        self.navigation_enabled = True
        self.gate_received = 0.
        if self.get_parameter('map_yaml').value:
            from .grid_world import GridWorld
            self.grid = GridWorld(self.get_parameter('map_yaml').value)
            self.truth = self.grid.initial_pose(); self.odom = self.truth.copy()
            self.navigation_enabled = False
            self.create_subscription(PoseWithCovarianceStamped, '/sim/reset_pose', self.reset_pose, 1)
            self.create_subscription(Bool, '/sim/navigation_enabled', self.navigation_gate, 1)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        if not 0 < self.max_linear_speed <= .3:
            raise ValueError('Simulation max_linear_speed must be in (0, 0.3]')
        self.dt=.02; self.command=(0.,0.); self.command_time=-10.; self.wall_command=0.
        self.sensors_enabled=True; self.paused=False
        self.clock_pub=self.create_publisher(ClockMsg,'/clock',10)
        self.odom_pub=self.create_publisher(Odometry,'/odom',10)
        self.truth_pub=self.create_publisher(Odometry,'/ground_truth/odom',10)
        self.collision_pub=self.create_publisher(Bool,'/ground_truth/collision',10)
        self.done_pub=self.create_publisher(Bool,'/experiment/route_done',10)
        self.route_pub=self.create_publisher(Twist,'/cmd_vel_nav',10)
        self.lidars={side:self.create_publisher(__import__('sensor_msgs.msg',fromlist=['PointCloud2']).PointCloud2,'/livox/lidar_'+side,qos_profile_sensor_data) for side in ('front','back')}
        self.tf=TransformBroadcaster(self); self.static_tf=StaticTransformBroadcaster(self)
        transforms=[]
        for side,x,yaw in [('front',.3,0.),('back',-.3,math.pi)]:
            t=TransformStamped(); t.header.frame_id='base_link'; t.child_frame_id='lidar_'+side
            t.transform.translation.x=x; t.transform.translation.z=.3
            t.transform.rotation.z=math.sin(yaw/2); t.transform.rotation.w=math.cos(yaw/2); transforms.append(t)
        self.static_tf.sendTransform(transforms)
        self.create_subscription(Twist,'/cmd_vel',self.on_command,10)
        self.create_service(SetBool,'/experiment/sensors_enabled',self.sensor_switch)
        self.create_service(SetBool,'/experiment/pause',self.pause)
        self.create_service(SetBool,'/experiment/block_path',self.block)
        self.wall_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(self.dt/float(self.get_parameter('rate').value),self.step,clock=self.wall_clock)
    def sensor_switch(self,req,res):
        self.sensors_enabled=req.data; res.success=True; return res
    def pause(self,req,res):
        self.paused=req.data; self.command=(0.,0.); res.success=True; return res
    def block(self,req,res):
        if self.grid:
            res.success=False; res.message='Synthetic barriers are unavailable in recorded-map mode'; return res
        # Barrier around the start cell for an intentionally unreachable navigation goal.
        self.segments=geometry(self.scenario)
        if req.data:
            extra=np.array([[[.9,-1.9],[.9,1.]],[[.9,1.],[-1.9,1.]]])
            self.segments=np.concatenate([self.segments,extra])
        res.success=True; return res
    def navigation_gate(self, msg):
        self.navigation_enabled = msg.data
        self.gate_received = time.monotonic()
        if not msg.data: self.command = (0., 0.)
    def reset_pose(self, msg):
        p = msg.pose.pose
        q = p.orientation
        pose = np.array([p.position.x, p.position.y, math.atan2(2*q.w*q.z, 1-2*q.z*q.z)])
        if self.grid and not self.navigation_enabled and msg.header.frame_id == 'map' and self.grid.safe(pose):
            self.truth = pose; self.odom = pose.copy(); self.command = (0., 0.)
    def on_command(self,m):
        vals=[m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z]
        valid=all(math.isfinite(x) for x in vals) and all(abs(vals[i])<1e-9 for i in (1,2,3,4))
        self.command=(min(self.max_linear_speed,max(0.,m.linear.x)),
                      min(MAX_ANGULAR_SPEED,max(-MAX_ANGULAR_SPEED,m.angular.z))) if valid else (0.,0.)
        self.command_time=self.tick_count*self.dt; self.wall_command=time.monotonic()
        if not self.navigation_enabled: self.command=(0.,0.)
    def stamp(self):
        ns=int(round(self.tick_count*self.dt*1e9))
        return __import__('builtin_interfaces.msg',fromlist=['Time']).Time(sec=ns//10**9,nanosec=ns%10**9)
    def odometry(self,pose,v,w,frame):
        m=Odometry(); m.header.stamp=self.stamp(); m.header.frame_id=frame; m.child_frame_id='base_link'
        m.pose.pose.position.x=float(pose[0]); m.pose.pose.position.y=float(pose[1])
        m.pose.pose.orientation.z=math.sin(pose[2]/2); m.pose.pose.orientation.w=math.cos(pose[2]/2)
        m.twist.twist.linear.x=float(v); m.twist.twist.angular.z=float(w)
        for idx in (0,7,35): m.pose.covariance[idx]=.001 if self.noise else 1e-6; m.twist.covariance[idx]=.001
        return m
    def step(self):
        if self.paused:
            self.command=(0.,0.); return
        self.tick_count+=1
        self.clock_pub.publish(ClockMsg(clock=self.stamp()))
        v,w=self.command
        if self.tick_count*self.dt-self.command_time>.5 or time.monotonic()-self.wall_command>.5: v=w=0.
        if not self.navigation_enabled or (self.grid and time.monotonic()-self.gate_received > .5): v=w=0.
        proposed=advance(self.truth,v,w,self.dt)
        hit=not self.grid.safe(proposed) if self.grid else collides(proposed,self.segments)
        if not hit: self.truth=proposed
        else: v=w=0.
        bias=1.005 if self.noise else 1.
        self.odom=advance(self.odom,v*bias,w*bias,self.dt)
        self.collision_pub.publish(Bool(data=hit))
        self.truth_pub.publish(self.odometry(self.truth,v,w,'world'))
        odom=self.odometry(self.odom,v*bias,w*bias,'odom'); self.odom_pub.publish(odom)
        t=TransformStamped(); t.header=odom.header; t.child_frame_id='base_link'
        t.transform.translation.x=float(self.odom[0]); t.transform.translation.y=float(self.odom[1]); t.transform.rotation=odom.pose.pose.orientation
        self.tf.sendTransform(t)
        if self.mapping and self.tick_count%5==0:
            # Scripted route follows measured odometry, never the published ground truth.
            cmd=Twist()
            if self.waypoint<len(self.path):
                dx=self.path[self.waypoint][0]-self.odom[0]; dy=self.path[self.waypoint][1]-self.odom[1]
                if math.hypot(dx,dy)<.12: self.waypoint+=1
                else:
                    err=wrap(math.atan2(dy,dx)-self.odom[2])
                    cmd.angular.z=max(-MAX_ANGULAR_SPEED,min(MAX_ANGULAR_SPEED,err))
                    cmd.linear.x=MAX_LINEAR_SPEED if abs(err)<.2 else 0.
            self.route_pub.publish(cmd); self.done_pub.publish(Bool(data=self.waypoint==len(self.path)))
        if self.sensors_enabled and self.tick_count%5==0:
            angles=np.linspace(-math.pi,math.pi,720,endpoint=False)
            for side,x,yaw in [('front',.3,0.),('back',-.3,math.pi)]:
                a=self.truth[2]; origin=self.truth[:2]+x*np.array([math.cos(a),math.sin(a)])
                distances=self.grid.raycast(origin,angles+a+yaw) if self.grid else raycast(origin,angles+a+yaw,self.segments)
                if self.noise: distances=distances+self.rng.normal(0,.01,len(distances))
                ok=np.isfinite(distances)&(distances>.15)
                points=np.stack((distances[ok]*np.cos(angles[ok]),distances[ok]*np.sin(angles[ok]),np.zeros(ok.sum())),axis=1)
                h=Header(stamp=self.stamp(),frame_id='lidar_'+side)
                self.lidars[side].publish(point_cloud2.create_cloud_xyz32(h,points.tolist()))

def main():
    rclpy.init(); n=Simulator()
    try: rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
