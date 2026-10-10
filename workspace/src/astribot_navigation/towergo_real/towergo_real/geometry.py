"""Measured-frame geometry shared by offline tests and the ROS adapter."""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def transform(position, quaternion):
    p, q = np.asarray(position, float), np.asarray(quaternion, float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError('Invalid pose')
    if abs(np.linalg.norm(q)-1.) > .01:
        raise ValueError('Invalid quaternion norm')
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(q).as_matrix()
    t[:3, 3] = p
    return t


def apply(t, points):
    return points @ t[:3, :3].T + t[:3, 3]


def points_from_cloud(msg):
    fields = {f.name: f for f in msg.fields}
    if len(fields) != len(msg.fields) or not all(k in fields for k in ('x', 'y', 'z')):
        raise ValueError('Missing/duplicate XYZ fields')
    if msg.height < 1 or msg.width < 1 or msg.row_step < msg.width*msg.point_step or len(msg.data) < msg.height*msg.row_step:
        raise ValueError('Truncated cloud')
    arrays = []
    for k in ('x', 'y', 'z'):
        f = fields[k]
        if f.datatype != 7 or f.count != 1 or f.offset+4 > msg.point_step:
            raise ValueError('XYZ must be float32')
        arrays.append(np.ndarray((msg.height, msg.width), dtype=('>' if msg.is_bigendian else '<')+'f4',
                                 buffer=msg.data, offset=f.offset,
                                 strides=(msg.row_step, msg.point_step)).ravel())
    return np.stack(arrays, axis=1)


def inside_polygon(xy, polygon):
    inside = np.zeros(len(xy), bool)
    boundary = inside.copy()
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        edge = b-a
        cross = (xy[:, 0]-a[0])*edge[1] - (xy[:, 1]-a[1])*edge[0]
        dot = (xy-a) @ edge
        boundary |= (np.abs(cross) < 1e-8) & (dot >= 0) & (dot <= edge@edge)
        if b[1] != a[1]:
            inside ^= ((a[1] > xy[:, 1]) != (b[1] > xy[:, 1])) & (
                xy[:, 0] < (b[0]-a[0])*(xy[:, 1]-a[1])/(b[1]-a[1])+a[0])
    return inside | boundary


def scan_from_body(points, config, bins=720):
    base_lidar = np.asarray(config['base_from_lidar'], float)
    imu_lidar = np.asarray(config['imu_from_lidar'], float)
    base_imu = base_lidar @ np.linalg.inv(imu_lidar)
    finite = points[np.isfinite(points).all(axis=1)]
    base = apply(base_imu, finite)
    keep = (base[:, 2] >= config['obstacle_height_min']) & (base[:, 2] <= config['obstacle_height_max'])
    keep &= ~inside_polygon(base[:, :2], np.asarray(config['body_polygon'], float))
    # scan_front axes align with base_link; origin stays at the real front LiDAR.
    scan = base[keep] - base_lidar[:3, 3]
    ranges = np.linalg.norm(scan[:, :2], axis=1)
    valid = (ranges >= config['range_min']) & (ranges <= config['range_max'])
    angles = np.arctan2(scan[valid, 1], scan[valid, 0])
    indices = np.floor((angles+math.pi)/(2*math.pi)*bins+.5).astype(int) % bins
    result = np.full(bins, np.inf)
    np.minimum.at(result, indices, ranges[valid])
    # Unobserved bins are NaN, not max-range rays that clear unknown space.
    result[~np.isfinite(result)] = np.nan
    return result


class PoseSession:
    def __init__(self, config):
        self.config = config
        self.imu_from_base = np.asarray(config['imu_from_lidar']) @ np.linalg.inv(np.asarray(config['base_from_lidar']))
        self.odom_from_world = None
        self.previous = None
        self.previous_stamp = None
        self.publisher = None
        self.failed = False

    def accept(self, raw_pose, stamp_ns, publisher=None):
        if self.failed:
            raise ValueError('Mapping session is faulted')
        try:
            if stamp_ns <= 0 or (self.previous_stamp is not None and stamp_ns <= self.previous_stamp):
                raise ValueError('LIO timestamp reset/repeated')
            if self.publisher is not None and publisher != self.publisher:
                raise ValueError('LIO publisher changed; start a new mapping session')
            self.publisher = publisher
            world_base = raw_pose @ self.imu_from_base
            if self.odom_from_world is None:
                # The verified v1 capture starts stationary on flat ground.
                self.odom_from_world = np.linalg.inv(world_base)
            pose = self.odom_from_world @ world_base
            velocity = np.zeros(6)
            if self.previous is not None:
                delta = np.linalg.inv(self.previous) @ pose
                distance = np.linalg.norm(delta[:3, 3])
                angle = np.linalg.norm(Rotation.from_matrix(delta[:3, :3]).as_rotvec())
                if distance > self.config['max_pose_step_m'] or angle > self.config['max_pose_step_rad']:
                    raise ValueError('Discontinuous LIO trajectory')
                dt = (stamp_ns-self.previous_stamp)*1e-9
                velocity[:3] = pose[:3, :3].T @ (pose[:3, 3]-self.previous[:3, 3])/dt
                velocity[3:] = Rotation.from_matrix(self.previous[:3, :3].T@pose[:3, :3]).as_rotvec()/dt
            self.previous, self.previous_stamp = pose, stamp_ns
            return pose, velocity
        except Exception:
            self.failed = True
            raise
