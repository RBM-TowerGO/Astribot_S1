"""Read-only sensor bridge. Does not import SDK or call device services."""
import math
import time
import threading
from collections import deque
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from .geometry import Geometry, matrix, scan_points
from .pipeline import Metrics, CloudWorker, match_scans
from .core import sensor_fresh
from .stability import Window

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
        self.geometry_lock=threading.Lock();self.snapshot_key=None;self.cached_snapshot=None
        self.metrics=Metrics();self.arrivals={}
        self.settling=Window(.25)
        from mapping_slice import raw_points
        self.worker=CloudWorker(self.geometry_snapshot,raw_points,self.metrics)
        node.create_timer(.01,self.flush)
        self.latest_scan=None
        self.latest_scan_source_stamp=None;self.latest_scan_stamp_ns=None
        self.recovering_until=0.
        self.received={};self.stamps={};self.frames={};self.error=None;self.last_pose=None;self.last_pose_stamp_ns=None
        self.planar_speed=0.;self.velocity=[0.,0.];self.scans={};self.obstacles={}
        self.scan_history={k:deque(maxlen=6) for k in ('front','back')}
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
        from diagnostic_msgs.msg import DiagnosticArray
        node.create_subscription(DiagnosticArray,NS+'lio/diagnostics',self.fusion_diagnostics,qos_profile_sensor_data)
        for key,topic in [('front','/livox/lidar_front'),('back','/livox/lidar_back')]:
            node.create_subscription(PointCloud2,topic,lambda m,k=key:self.cloud(m,k),qos_profile_sensor_data)
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Bool,NS+'lio/fault',lambda m:self.fail('双雷达适配器故障') if m.data else None,q)

    def fusion_diagnostics(self,msg):
        for status in msg.status:
            for value in status.values:
                try:self.metrics.add('fusion_'+value.key,float(value.value))
                except ValueError:pass

    def geometry_snapshot(self):
        with self.geometry_lock:
            key=(self.geometry.continuity_epoch,self.geometry.poses[-1][0] if self.geometry.poses else None)
            if self.cached_snapshot is None or key!=self.snapshot_key:
                self.cached_snapshot=self.geometry.snapshot();self.snapshot_key=key
            return self.cached_snapshot

    def close(self):
        self.worker.close()

    def measure(self,key,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        now=self.node.get_clock().now().nanoseconds*1e-9
        received=time.monotonic()
        self.metrics.add(key+'_source_age_s',now-stamp)
        if key in self.arrivals:self.metrics.add(key+'_receive_interval_s',received-self.arrivals[key])
        self.arrivals[key]=received

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
        if hasattr(self,'metrics'):self.measure(key,msg)
        frame=msg.header.frame_id
        if key in self.frames and self.frames[key]!=frame:
            self.fail('传感器坐标系改变：'+key);return None
        self.frames[key]=frame
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        now=self.node.get_clock().now().nanoseconds*1e-9
        fresh=-.05<=now-stamp<=.5
        if not fresh and not (history and now-stamp>.5):
            return None
        if key in self.stamps and stamp<self.stamps[key]:
            self.fail("传感器时间倒退："+key);return None
        if key in self.stamps and stamp==self.stamps[key]:return None
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
            with self.geometry_lock:
                t=self.geometry.pose(stamp,matrix([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]))
            self.worker.wake()
            if t is None:return
            if epoch!=self.geometry.continuity_epoch:
                self.settling.rows.clear()
                self.last_pose=None;self.latest_scan=None;self.scans={};self.obstacles={}
                self.latest_scan_source_stamp=None;self.latest_scan_stamp_ns=None
                for history in self.scan_history.values():history.clear()
                self.received.pop('scan',None)
                self.recovering_until=time.monotonic()+.5
            if self.last_pose:
                old_stamp,old=self.last_pose;dt=stamp-old_stamp
                v=t[:3,:3].T @ (t[:3,3]-old[:3,3])/dt
                w=Rotation.from_matrix(old[:3,:3].T@t[:3,:3]).as_rotvec()/dt
                self.velocity=[float(v[0]),float(w[2])];self.planar_speed=float(np.linalg.norm(v[:2]))
                self.settling.add(time.monotonic(),[self.planar_speed,abs(self.velocity[1])])
            if self.last_pose is None:self.static.sendTransform(self.static_transforms)
            self.last_pose=(stamp,t)
            self.last_pose_stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            out=Odometry();out.header.stamp=msg.header.stamp;out.header.frame_id=FRAME['odom'];out.child_frame_id=FRAME['base']
            out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,t[:3,3])
            q=Rotation.from_matrix(t[:3,:3]).as_quat()
            out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,q)
            out.twist.twist.linear.x,out.twist.twist.angular.z=self.velocity
            # Conservative nonzero estimates; no claim of calibrated covariance.
            out.pose.covariance=(np.eye(6)*.05).ravel().tolist();out.twist.covariance=(np.eye(6)*.05).ravel().tolist()
            self.odom_pub.publish(out);self.tf.sendTransform(self.tf_msg(t,FRAME['odom'],FRAME['base'],msg.header.stamp))
            self.metrics.add('odom_tf_source_age_s',self.node.get_clock().now().nanoseconds*1e-9-stamp)
        except Exception as exc:self.fail(str(exc))

    def cloud(self,msg,key):
        if self.seen(key,msg) is None or self.error:return
        self.worker.submit(key,msg,self.geometry.continuity_epoch)

    def flush(self):
        started=time.monotonic()
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header
        from sensor_msgs.msg import LaserScan
        results,error=self.worker.take()
        if error:self.fail(error)
        if self.error:return
        now=self.node.get_clock().now().nanoseconds*1e-9
        for key,stamp,points,ranges,epoch,received in results:
            if epoch!=self.geometry.continuity_epoch or not -.05<=now-stamp<=.5:
                self.metrics.count(key+'_stale_output_discarded');continue
            try:
                source_stamp=rclpy.time.Time(nanoseconds=int(round(stamp*1e9))).to_msg()
                header=Header();header.stamp=source_stamp;header.frame_id=FRAME[key]
                self.cloud_pub[key].publish(point_cloud2.create_cloud_xyz32(header,points))
                self.obstacles[key]=(stamp,points[::max(1,len(points)//600)].tolist())
                self.scans[key]=(stamp,ranges)
                self.scan_history[key].append((stamp,ranges))
                self.metrics.add(key+'_output_source_age_s',now-stamp)
                if len(self.scans)!=2:continue
                # Match actual acquisition times, not whichever callbacks
                # happened to finish last. Keep only bounded recent history.
                pair=match_scans(self.scan_history,now,self.latest_scan_source_stamp)
                if pair is None:
                    self.metrics.count('scan_pair_unmatched');continue
                front,back=pair
                # Reproject both scans to one measured reference time before fusion.
                # Project the measured endpoints into the latest recorded LIO pose.
                # This uses a real transform, never extrapolation or wall-clock time.
                # Retain the OLDEST contributing acquisition for health gates;
                # a newer projection frame must never renew old observations.
                source_stamp=min(front[0],back[0])
                reference_stamp=self.last_pose[0]
                reference_ns=self.last_pose_stamp_ns
                if max(front[0],back[0])>reference_stamp:continue
                if self.latest_scan and reference_stamp<=self.latest_scan[0]:continue
                reference=self.geometry.at(reference_stamp)
                if reference is None:
                    self.metrics.count('scan_reference_pose_missing');continue
                origin=self.geometry.base_lidar[:3,3]
                merged=np.full(720,np.nan)
                for scan_stamp,scan_ranges in (front,back):
                    pose=self.geometry.at(scan_stamp)
                    if pose is None:
                        self.metrics.count('scan_source_pose_missing');break
                    base=scan_points(scan_ranges)+origin
                    world=base@pose[:3,:3].T+pose[:3,3]
                    local=(world-reference[:3,3])@reference[:3,:3]-origin
                    bins=np.floor((np.arctan2(local[:,1],local[:,0])+math.pi)/(2*math.pi)*720+.5).astype(int)%720
                    values=np.full(720,np.inf)
                    np.minimum.at(values,bins,np.linalg.norm(local[:,:2],axis=1))
                    values[~np.isfinite(values)]=np.nan
                    merged=np.fmin(merged,values)
                else:
                    scan=LaserScan();scan.header.stamp=rclpy.time.Time(nanoseconds=reference_ns).to_msg();scan.header.frame_id=FRAME['front']
                    scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/720;scan.angle_max=-math.pi+719*scan.angle_increment
                    scan.range_min=.15;scan.range_max=20.;scan.scan_time=.1;scan.ranges=merged.tolist()
                    self.latest_scan=(reference_stamp,scan_points(merged).tolist())
                    self.latest_scan_source_stamp=source_stamp;self.latest_scan_stamp_ns=reference_ns
                    self.scan_pub.publish(scan);self.received['scan']=time.monotonic();self.stamps['scan']=source_stamp
                    self.metrics.add('scan_source_age_s',now-source_stamp)
                    self.metrics.add('scan_reference_age_s',now-reference_stamp)
            except Exception as exc:self.fail(str(exc))

        self.metrics.add('flush_ms',(time.monotonic()-started)*1000)

    def healthy(self):
        now=time.monotonic()
        for topic in ('/livox/lidar_front','/livox/lidar_back','/livox/imu_front',NS+'lio/odometry'):
            if self.node.count_publishers(topic)!=1:return False
        ros_now=self.node.get_clock().now().nanoseconds*1e-9
        return not self.error and now>=self.recovering_until and all(sensor_fresh(k,now-self.received.get(k,-1e20),ros_now-self.stamps.get(k,-1e20)) for k in ('front','back','imu','lio','scan'))
