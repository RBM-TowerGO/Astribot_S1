"""Read-only sensor bridge. Does not import SDK or call device services."""
import math
import time
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from .geometry import Geometry, matrix, scan_points

NS='/towergo/nav/'
FRAME={'map':'tg_nav_map','odom':'tg_nav_odom','base':'tg_nav_base','front':'tg_nav_front','back':'tg_nav_back'}


class Sensors:
    def __init__(self,node,config):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu,PointCloud2,LaserScan
        from std_msgs.msg import Bool
        from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
        from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
        self.node,self.config=node,config
        self.geometry=Geometry(config)
        self.latest_scan=None
        self.recovering_until=0.
        self.received={};self.stamps={};self.frames={};self.error=None;self.last_pose=None
        self.planar_speed=0.;self.velocity=[0.,0.];self.pending=[];self.scans={};self.obstacles={}
        self.odom_pub=node.create_publisher(Odometry,NS+'odom',10)
        self.scan_pub=node.create_publisher(LaserScan,NS+'scan',qos_profile_sensor_data)
        self.cloud_pub={k:node.create_publisher(PointCloud2,NS+'obstacles/'+k,qos_profile_sensor_data) for k in ('front','back')}
        self.tf=TransformBroadcaster(node);self.static=StaticTransformBroadcaster(node)
        self.static_transforms=[]
        for rear,key in [(False,'front'),(True,'back')]:
            t=np.eye(4);t[:3,3]=self.geometry.base_lidar[:3,3]
            if rear:t[:3,3]+=self.geometry.base_lidar[:3,:3] @ np.asarray(config['rear_origin_in_front'])
            self.static_transforms.append(self.tf_msg(t,FRAME['base'],FRAME[key],rclpy.time.Time().to_msg()))
        self.static.sendTransform(self.static_transforms)
        node.create_subscription(Imu,'/livox/imu_front',self.imu,qos_profile_sensor_data)
        node.create_subscription(Odometry,NS+'lio/odometry',self.pose,10)
        for key,topic in [('front','/livox/lidar_front'),('back','/livox/lidar_back')]:
            node.create_subscription(PointCloud2,topic,lambda m,k=key:self.cloud(m,k),qos_profile_sensor_data)
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Bool,NS+'lio/fault',lambda m:self.fail('双雷达适配器故障') if m.data else None,q)

    def fail(self,reason):
        self.error=reason

    def tf_msg(self,t,parent,child,stamp):
        from geometry_msgs.msg import TransformStamped
        out=TransformStamped();out.header.stamp=stamp;out.header.frame_id=parent;out.child_frame_id=child
        out.transform.translation.x,out.transform.translation.y,out.transform.translation.z=map(float,t[:3,3])
        q=Rotation.from_matrix(t[:3,:3]).as_quat()
        out.transform.rotation.x,out.transform.rotation.y,out.transform.rotation.z,out.transform.rotation.w=map(float,q)
        return out

    def seen(self,key,msg,history=False):
        frame=msg.header.frame_id
        if key in self.frames and self.frames[key]!=frame:
            self.fail('传感器坐标系改变：'+key);return None
        self.frames[key]=frame
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        now=self.node.get_clock().now().nanoseconds*1e-9
        fresh=-.05<=now-stamp<=.5
        if not fresh and not (history and now-stamp>.5):
            return None
        if key in self.stamps and stamp<=self.stamps[key]:
            return None
        if fresh:self.received[key]=time.monotonic()
        self.stamps[key]=stamp
        return stamp

    def imu(self,msg):
        if self.seen('imu',msg) is not None:
            a=msg.linear_acceleration;self.geometry.imu([a.x,a.y,a.z])

    def pose(self,msg):
        from nav_msgs.msg import Odometry
        try:
            stamp=self.seen('lio',msg,history=True)
            if stamp is None or self.error:return
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            epoch=self.geometry.continuity_epoch
            t=self.geometry.pose(stamp,matrix([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]))
            if t is None:return
            if epoch!=self.geometry.continuity_epoch:
                self.last_pose=None;self.latest_scan=None;self.pending=[];self.scans={};self.obstacles={}
                self.received.pop('scan',None)
                self.recovering_until=time.monotonic()+.5
            if self.last_pose:
                old_stamp,old=self.last_pose;dt=stamp-old_stamp
                v=t[:3,:3].T @ (t[:3,3]-old[:3,3])/dt
                w=Rotation.from_matrix(old[:3,:3].T@t[:3,:3]).as_rotvec()/dt
                self.velocity=[float(v[0]),float(w[2])];self.planar_speed=float(np.linalg.norm(v[:2]))
            if self.last_pose is None:self.static.sendTransform(self.static_transforms)
            self.last_pose=(stamp,t)
            out=Odometry();out.header.stamp=msg.header.stamp;out.header.frame_id=FRAME['odom'];out.child_frame_id=FRAME['base']
            out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,t[:3,3])
            q=Rotation.from_matrix(t[:3,:3]).as_quat()
            out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,q)
            out.twist.twist.linear.x,out.twist.twist.angular.z=self.velocity
            # Conservative nonzero estimates; no claim of calibrated covariance.
            out.pose.covariance=(np.eye(6)*.05).ravel().tolist();out.twist.covariance=(np.eye(6)*.05).ravel().tolist()
            self.odom_pub.publish(out);self.tf.sendTransform(self.tf_msg(t,FRAME['odom'],FRAME['base'],msg.header.stamp))
            self.flush()
        except Exception as exc:self.fail(str(exc))

    def cloud(self,msg,key):
        if self.seen(key,msg) is None or self.error:return
        self.pending.append((key,msg,time.monotonic()))
        self.pending=self.pending[-12:];self.flush()

    def flush(self):
        from mapping_slice import raw_points
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header
        from sensor_msgs.msg import LaserScan
        remaining=[]
        for key,msg,received in self.pending:
            try:
                raw,stamps,rejected=raw_points(msg)
                if not len(raw):continue
                stamp=float(stamps.max())*1e-9
            except Exception as exc:
                self.fail(str(exc));continue
            pose=self.geometry.at(stamp)
            if pose is None:
                if time.monotonic()-received<.3:remaining.append((key,msg,received))
                continue
            try:
                deskewed=self.geometry.deskew(raw,stamps,pose)
                if deskewed is None:continue
                points,ranges=self.geometry.cloud(deskewed,pose,key=='back')
                source_stamp=rclpy.time.Time(nanoseconds=int(stamps.max())).to_msg()
                header=Header();header.stamp=source_stamp;header.frame_id=FRAME[key]
                self.cloud_pub[key].publish(point_cloud2.create_cloud_xyz32(header,points.tolist()))
                self.obstacles[key]=(stamp,points[::max(1,len(points)//600)].tolist())
                self.scans[key]=(stamp,ranges)
                if len(self.scans)==2 and abs(self.scans['front'][0]-self.scans['back'][0])<=.1:
                    # Fused scan is for localization only; clearing uses independent real origins.
                    a,b=self.scans['front'][1],self.scans['back'][1]
                    merged=np.fmin(a,b)
                    scan=LaserScan();scan.header.stamp=source_stamp;scan.header.frame_id=FRAME['front']
                    scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/720;scan.angle_max=-math.pi+719*scan.angle_increment
                    scan.range_min=.15;scan.range_max=20.;scan.scan_time=.1;scan.ranges=merged.tolist()
                    self.latest_scan=(stamp,scan_points(merged).tolist())
                    self.scan_pub.publish(scan);self.received['scan']=time.monotonic()
            except Exception as exc:self.fail(str(exc))
        self.pending=remaining

    def healthy(self):
        now=time.monotonic()
        for topic in ('/livox/lidar_front','/livox/lidar_back','/livox/imu_front',NS+'lio/odometry'):
            if self.node.count_publishers(topic)!=1:return False
        return not self.error and now>=self.recovering_until and all(now-self.received.get(k,-1e20)<.5 for k in ('front','back','imu','lio','scan'))
