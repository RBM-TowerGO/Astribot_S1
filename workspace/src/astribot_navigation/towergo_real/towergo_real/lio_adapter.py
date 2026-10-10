"""One public TF owner, timestamp-matched deskewed scans, no control imports."""
from collections import OrderedDict
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .contracts import calibration
from .geometry import PoseSession, points_from_cloud, scan_from_body, transform


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry, Path
    from sensor_msgs.msg import PointCloud2, LaserScan
    from std_msgs.msg import Bool
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

    rclpy.init(args=args)
    class LioAdapter(Node):
        def __init__(self):
            super().__init__('towergo_lio_adapter')
            self.config = calibration(self.declare_parameter('calibration', '').value)
            self.session = PoseSession(self.config)
            self.failed = False
            self.poses, self.clouds = OrderedDict(), OrderedDict()
            self.last_receipt = None
            self.tf = TransformBroadcaster(self)
            self.static = StaticTransformBroadcaster(self)
            self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
            self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
            self.path_pub = self.create_publisher(Path, '/towergo/trajectory', 1)
            self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.fault_pub = self.create_publisher(Bool, '/towergo/mapping_fault', latched)
            self.create_subscription(Bool, '/towergo/mapping_fault', self.external_fault, latched)
            self.create_subscription(Odometry, '/towergo/lio/odometry_raw', self.pose, 10)
            self.create_subscription(PointCloud2, '/towergo/lio/cloud_body', self.cloud, qos_profile_sensor_data)
            self.path = Path()
            self.path.header.frame_id = 'odom'
            base_lidar = np.asarray(self.config['base_from_lidar'])
            base_imu = base_lidar @ np.linalg.inv(np.asarray(self.config['imu_from_lidar']))
            scan = np.eye(4); scan[:3, 3] = base_lidar[:3, 3]
            self.static.sendTransform([self.tf_msg(t, 'base_link', name, self.get_clock().now().to_msg())
                                       for t, name in [(base_lidar, 'lidar_front'), (base_imu, 'imu_front'), (scan, 'scan_front')]])
            self.create_timer(.5, self.health)

        def tf_msg(self, matrix, parent, child, stamp):
            msg = TransformStamped()
            msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, parent, child
            t, q = matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_quat()
            msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, t)
            msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = map(float, q)
            return msg

        def diagnostic(self, message, level):
            out = DiagnosticArray(); out.header.stamp = self.get_clock().now().to_msg()
            s = DiagnosticStatus(); s.name = 'towergo/lio_adapter'; s.level = bytes([level]); s.message = message
            out.status = [s]; self.diag_pub.publish(out)

        def fail(self, reason):
            if self.failed:
                return
            self.failed = True
            self.poses.clear(); self.clouds.clear()
            msg = Bool(); msg.data = True; self.fault_pub.publish(msg)
            self.diagnostic(reason+'; session ended, restart into a new run directory', 2)

        def external_fault(self, msg):
            if msg.data:
                self.fail('Sensor/LIO session fault')

        def health(self):
            if self.count_publishers('/odom') > 1 or self.count_publishers('/scan') > 1:
                self.fail('Competing /odom or /scan publisher')
            # Offline bag pauses are allowed; live loss must terminate this map.
            sim = self.get_parameter('use_sim_time').value
            if not sim and self.last_receipt is not None and time.monotonic()-self.last_receipt > 2.:
                self.fail('No LIO odometry for 2 seconds')

        def pose(self, msg):
            if self.failed:
                return
            try:
                if (msg.header.frame_id, msg.child_frame_id) != ('camera_init', 'body'):
                    raise ValueError('Unexpected FAST-LIO frame contract')
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                # The vendor Humble rclpy predates MessageInfo callbacks.
                publishers = self.get_publishers_info_by_topic('/towergo/lio/odometry_raw')
                if len(publishers) != 1:
                    raise ValueError('Expected exactly one raw LIO odometry publisher')
                t, v = self.session.accept(transform([p.x,p.y,p.z], [q.x,q.y,q.z,q.w]), stamp,
                                           bytes(publishers[0].endpoint_gid))
                self.last_receipt = time.monotonic()
                self.tf.sendTransform(self.tf_msg(t, 'odom', 'base_link', msg.header.stamp))
                out = Odometry(); out.header = msg.header; out.header.frame_id = 'odom'; out.child_frame_id = 'base_link'
                out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, t[:3, 3])
                rot = Rotation.from_matrix(t[:3, :3]).as_quat()
                out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = map(float, rot)
                out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = map(float, v[:3])
                out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = map(float, v[3:])
                # Upstream covariance/twist is not calibrated at this lever arm.
                # Explicitly conservative uncertainty avoids advertising perfect odometry.
                out.pose.covariance = (np.eye(6)*1e3).ravel().tolist()
                out.twist.covariance = (np.eye(6)*1e3).ravel().tolist()
                self.odom_pub.publish(out)
                from geometry_msgs.msg import PoseStamped
                ps = PoseStamped(); ps.header = out.header; ps.pose = out.pose.pose
                self.path.header.stamp = out.header.stamp; self.path.poses.append(ps)
                self.path.poses = self.path.poses[-3000:]
                if len(self.path.poses) % 10 == 0:
                    self.path_pub.publish(self.path)
                self.poses[stamp] = out.header.stamp
                if len(self.poses) > 50:
                    self.poses.popitem(last=False)
                self.flush(stamp)
            except Exception as exc:
                self.fail(str(exc))

        def cloud(self, msg):
            if self.failed:
                return
            if msg.header.frame_id != 'body':
                self.fail('Expected current deskewed cloud in FAST-LIO IMU body frame')
                return
            stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            self.clouds[stamp] = msg
            if len(self.clouds) > 10:
                self.clouds.popitem(last=False)
                self.diagnostic('Dropped cloud without matching odometry timestamp', 1)
            self.flush(stamp)

        def flush(self, stamp):
            if stamp not in self.poses or stamp not in self.clouds:
                return
            msg = self.clouds.pop(stamp)
            try:
                ranges = scan_from_body(points_from_cloud(msg), self.config)
                scan = LaserScan(); scan.header.stamp = self.poses.pop(stamp); scan.header.frame_id = 'scan_front'
                scan.angle_min = -math.pi; scan.angle_increment = 2*math.pi/720
                scan.angle_max = scan.angle_min+719*scan.angle_increment
                scan.scan_time = .1; scan.time_increment = 0.  # already deskewed to frame end
                scan.range_min, scan.range_max = float(self.config['range_min']), float(self.config['range_max'])
                scan.ranges = ranges.astype(np.float32).tolist()
                self.scan_pub.publish(scan)
                self.diagnostic('Matched pose and deskewed scan', 0)
            except Exception as exc:
                self.diagnostic('Dropped scan: '+str(exc), 2)

    node = None
    try:
        node = LioAdapter()
        while rclpy.ok() and not node.failed:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
