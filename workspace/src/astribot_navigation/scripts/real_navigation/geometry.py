"""Gravity-level odometry and timestamp-matched dual LiDAR projection."""
from collections import deque
import copy
from mapping_slice import Poses
import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def rotate_unit_quaternions(quaternions,points):
    """Vectorized rotation; Poses.interpolate already returns unit quaternions."""
    q=np.asarray(quaternions)
    twice=2*np.cross(q[:,:3],points)
    return points+q[:,3,None]*twice+np.cross(q[:,:3],twice)


def matrix(p,q):
    p,q=np.asarray(p,float),np.asarray(q,float)
    if p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(p).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.01:
        raise ValueError('Invalid pose')
    out=np.eye(4);out[:3,:3]=Rotation.from_quat(q).as_matrix();out[:3,3]=p
    return out


class PoseSnapshot:
    """Immutable interpolation arrays shared by workers until the next LIO pose."""
    def __init__(self,history):
        self.gap=history.gap
        self.ts=np.array([r[0] for r in history.data],dtype=np.int64)
        self.positions=np.asarray([r[1] for r in history.data])
        self.quaternions=np.asarray([r[2] for r in history.data])

    def interpolate(self,ns):
        ok=np.zeros(len(ns),bool)
        if len(self.ts)<2:return ok,np.empty((0,3)),np.empty((0,4))
        idx=np.clip(np.searchsorted(self.ts,ns,side='right')-1,0,len(self.ts)-2)
        ok=(ns>=self.ts[0])&(ns<=self.ts[-1])&(self.ts[idx+1]-self.ts[idx]<=self.gap)
        idx=idx[ok];a=((ns[ok]-self.ts[idx])/(self.ts[idx+1]-self.ts[idx]))[:,None]
        p=self.positions[idx]*(1-a)+self.positions[idx+1]*a
        q0=self.quaternions[idx];q1=self.quaternions[idx+1].copy()
        dot=np.sum(q0*q1,axis=1);q1[dot<0]*=-1;dot=np.abs(dot).clip(0,1)
        theta=np.arccos(dot)[:,None];den=np.sin(theta);close=dot>.9995;den[close]=1
        q=(np.sin((1-a)*theta)*q0+np.sin(a*theta)*q1)/den
        q[close]=(q0*(1-a)+q1*a)[close];q/=np.linalg.norm(q,axis=1)[:,None]
        return ok,p,q


class Geometry:
    def __init__(self, config):
        self.config=config
        self.base_lidar=np.asarray(config['base_from_lidar'])
        self.imu_lidar=np.asarray(config['imu_from_lidar'])
        self.imu_base=self.imu_lidar @ np.linalg.inv(self.base_lidar)
        self.origin=None
        self.poses=deque(maxlen=256)
        self.history=Poses(300000000)
        self.history.data=deque(maxlen=256)
        self.gravity=[]
        self.fault=None
        self.continuity_epoch=0

    def imu(self, xyz):
        if len(self.gravity)<100 and np.isfinite(xyz).all() and np.linalg.norm(xyz)>.1:
            self.gravity.append(xyz)

    def pose(self, stamp, raw):
        if self.poses and stamp <= self.poses[-1][0]:
            raise ValueError('LIO 时间倒退；需重启会话')
        gap=bool(self.poses and stamp-self.poses[-1][0]>.5)
        if self.origin is None:
            if len(self.gravity)<100:
                return None
            g=np.mean(self.gravity,axis=0)
            if np.max(np.std(np.asarray(self.gravity),axis=0)) > .15*np.linalg.norm(g):
                raise ValueError('IMU 静止初始化不稳定')
            # Acceleration is in IMU coordinates; align gravity to +Z, preserve initial base heading.
            z=raw[:3,:3] @ (g/np.linalg.norm(g))
            forward=(raw @ self.imu_base)[:3,0]
            x=forward-z*np.dot(forward,z); x/=np.linalg.norm(x)
            y=np.cross(z,x)
            self.origin=np.eye(4);self.origin[:3,:3]=np.stack([x,y,z])
            self.origin[:3,3]=-self.origin[:3,:3] @ (raw @ self.imu_base)[:3,3]
        base=self.origin @ raw @ self.imu_base
        if self.poses:
            prev=self.poses[-1][1]
            if np.linalg.norm(base[:3,3]-prev[:3,3])>.3 or Rotation.from_matrix(prev[:3,:3].T@base[:3,:3]).magnitude()>.3:
                raise ValueError('LIO 位姿跳变')
        if gap:
            # Preserve the odometry origin and jump checks, but never interpolate
            # across missing history. Consumers invalidate any active task.
            self.poses.clear()
            self.history=Poses(300000000)
            self.history.data=deque(maxlen=256)
            self.continuity_epoch+=1
        self.poses.append((stamp,base))
        self.history.add(int(round(stamp*1e9)),base[:3,3],Rotation.from_matrix(base[:3,:3]).as_quat(),'tg_nav_odom')
        return base

    def snapshot(self):
        result=copy.copy(self)
        result.poses=deque(self.poses,maxlen=256)
        result.history=PoseSnapshot(self.history)
        return result

    def at(self, stamp):
        good,positions,quaternions=self.history.interpolate(np.array([int(round(stamp*1e9))],dtype=np.int64))
        if not good.all():return None
        return matrix(positions[0],quaternions[0])

    def deskew(self, points, stamps, reference):
        good,positions,quaternions=self.history.interpolate(stamps)
        if not good.all():
            return None
        base=points @ self.base_lidar[:3,:3].T+self.base_lidar[:3,3]
        world=rotate_unit_quaternions(quaternions,base)+positions
        current_base=(world-reference[:3,3])@reference[:3,:3]
        return (current_base-self.base_lidar[:3,3])@self.base_lidar[:3,:3]

    def cloud(self, points, pose, rear=False):
        from towergo_real.geometry import inside_polygon
        points=np.asarray(points)
        points=points[np.isfinite(points).all(axis=1)]
        base=points @ self.base_lidar[:3,:3].T+self.base_lidar[:3,3]
        radius=np.linalg.norm(base[:,:2],axis=1)
        self_points=inside_polygon(base[:,:2],np.asarray(self.config['self_filter']))
        # Only the confirmed structure volumes; keep obstacles above/below and
        # beside them. Apply in body coordinates before gravity/world projection.
        for box in self.config.get('self_filter_boxes',[]):
            self_points |= np.all((base>=box['min'])&(base<=box['max']),axis=1)
        base=base[(radius>=.15)&(radius<=20)&~self_points]
        world=base @ pose[:3,:3].T+pose[:3,3]
        keep=(world[:,2]>=self.config['obstacle_min_height'])&(world[:,2]<=self.config['obstacle_max_height'])
        origin=self.base_lidar[:3,3].copy()
        if rear:
            origin += self.base_lidar[:3,:3] @ np.asarray(self.config['rear_origin_in_front'])
        obstacle=base[keep]-origin
        # Height referenced to initial base plane; source map slice height is a config snapshot.
        selected=abs(world[:,2]-self.config['slice_height'])<=self.config['slice_thickness']/2
        scan=base[selected]-self.base_lidar[:3,3]
        ranges=np.linalg.norm(scan[:,:2],axis=1)
        angles=np.arctan2(scan[:,1],scan[:,0])
        bins=np.floor((angles+math.pi)/(2*math.pi)*720+.5).astype(int)%720
        result=np.full(720,np.inf)
        np.minimum.at(result,bins,ranges)
        result[~np.isfinite(result)]=np.nan
        return obstacle.astype(np.float32),result


def safe_pose(grid, pose, footprint, padding=.06):
    """Conservative convex polygon / occupied-cell SAT including cell corners."""
    a=pose[2];r=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    polygon=(np.asarray(footprint)@r.T+pose[:2]-grid.origin[:2])@grid.rotation
    low=np.floor((polygon.min(axis=0)-padding)/grid.resolution).astype(int)
    high=np.floor((polygon.max(axis=0)+padding)/grid.resolution).astype(int)
    if min(low)<0 or high[0]>=grid.width or high[1]>=grid.height:return False
    edges=np.roll(polygon,-1,axis=0)-polygon
    axes=[np.array([1.,0.]),np.array([0.,1.])]+[np.array([-e[1],e[0]])/np.linalg.norm(e) for e in edges]
    half=grid.resolution/2
    for y in range(low[1],high[1]+1):
        for x in range(low[0],high[0]+1):
            if grid.free[y,x]:continue
            centre=np.array([x+.5,y+.5])*grid.resolution
            separated=False
            for axis in axes:
                proj=polygon@axis;c=centre@axis;radius=half*np.abs(axis).sum()
                if c+radius<proj.min()-padding or c-radius>proj.max()+padding:
                    separated=True;break
            if not separated:return False
    return True


def scan_points(ranges, minimum=.15, maximum=20.):
    """Finite horizontal scan endpoints, in the scan frame; no inferred returns."""
    ranges=np.asarray(ranges,float)
    angles=-math.pi+np.arange(len(ranges))*2*math.pi/len(ranges)
    valid=np.isfinite(ranges)&(ranges>=minimum)&(ranges<=maximum)
    return np.column_stack((ranges[valid]*np.cos(angles[valid]),
                            ranges[valid]*np.sin(angles[valid]),np.zeros(valid.sum())))
