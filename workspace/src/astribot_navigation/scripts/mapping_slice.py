"""Horizontal occupancy from real per-sensor rays. No ROS/SDK imports.

Coordinates: raw points already front-LiDAR; extrinsic_R/T maps them to IMU.
Slice frame levels measured initial gravity and starts at the first front origin.
Static occupied cells are sticky: a ray at a different height must not erase them.
"""
import base64
from collections import deque
import io
import math
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def stamp_ns(m):
    return m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec


def defaults(offset=0., thickness=.1):
    if not math.isfinite(offset) or not -.5 <= offset <= 2:
        raise ValueError('Slice offset must be finite and in [-0.5, 2] m')
    if not math.isfinite(thickness) or not .02 <= thickness <= .5:
        raise ValueError('Slice thickness must be in [0.02, 0.5] m')
    return dict(offset=offset, thickness=thickness, resolution=.05,
                max_range=20., min_range=.15, max_cells=4194304,
                max_rays=1024, max_pose_gap_ns=300000000,
                imu_from_front_R=np.eye(3).tolist(), imu_from_front_T=[-.011,-.02329,.04412],
                back_origin_front=[.001,-.496,.084],
                # Vendor front extrinsic; body rectangle from chassis mesh bounds.
                base_from_front=[[.696665,.717397,0,.17393],[-.717397,.696665,0,.16893],
                                 [0,0,1,.054],[0,0,0,1]],
                body_xy=[-.3231,.3069,-.311,.311], body_padding=.02,
                gravity_samples=100, frame='slice_map')


def raw_points(m):
    fields={f.name:f for f in m.fields}
    n=m.width*m.height
    if not 0<n<=100000 or m.row_step<m.width*m.point_step or len(m.data)<m.height*m.row_step:
        raise ValueError('Invalid slice cloud layout')
    def field(name, kind, datatype):
        f=fields[name]; dt=np.dtype(('>' if m.is_bigendian else '<')+kind)
        if f.datatype!=datatype or f.count!=1 or f.offset<0 or f.offset+dt.itemsize>m.point_step:
            raise ValueError('Invalid slice field: '+name)
        return np.ndarray((m.height,m.width), dtype=dt, buffer=m.data, offset=f.offset,
                          strides=(m.row_step,m.point_step)).ravel()
    p=np.column_stack([field(k,'f4',7) for k in ('x','y','z')]).astype(float)
    t=field('timestamp','f8',8)
    header=stamp_ns(m)
    if not np.isfinite(t).all() or np.any(t<header-1000) or np.any(t>header+200000000):
        raise ValueError('Invalid slice absolute point timestamp')
    ns=np.maximum(t.astype(np.int64),header)
    tag=field('tag','u1',2); line=field('line','u1',2)
    good=np.isfinite(p).all(axis=1)&np.isin(tag&0x30,[0,0x10])&(line<4)
    return p[good],ns[good],n-int(good.sum())


class Poses:
    def __init__(self, gap):
        self.data=deque(maxlen=512); self.gap=gap; self.first=None; self.frame=None

    def add(self, ns, p, q, frame):
        p=np.asarray(p,float);q=np.asarray(q,float)
        if not np.isfinite(p).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.01:
            raise ValueError('Invalid LIO pose')
        if ns<=0 or (self.data and ns<=self.data[-1][0]) or (self.frame and frame!=self.frame):
            raise ValueError('Non-monotonic LIO pose or changed frame')
        self.frame=frame; row=(ns,p,q/np.linalg.norm(q));self.data.append(row)
        if self.first is None:self.first=row

    def interpolate(self, ns):
        n=len(ns); ok=np.zeros(n,bool)
        if len(self.data)<2:return ok,np.empty((0,3)),np.empty((0,4))
        ts=np.array([r[0] for r in self.data],dtype=np.int64)
        idx=np.searchsorted(ts,ns,side='right')-1;idx=np.clip(idx,0,len(ts)-2)
        dt=ts[idx+1]-ts[idx]
        ok=(ns>=ts[0])&(ns<=ts[-1])&(dt<=self.gap)
        idx=idx[ok];a=((ns[ok]-ts[idx])/(ts[idx+1]-ts[idx]))[:,None]
        pos=np.array([r[1] for r in self.data]); quat=np.array([r[2] for r in self.data])
        p=pos[idx]*(1-a)+pos[idx+1]*a
        q0=quat[idx];q1=quat[idx+1].copy();dot=np.sum(q0*q1,axis=1)
        q1[dot<0]*=-1;dot=np.abs(dot).clip(0,1)
        theta=np.arccos(dot)[:,None];den=np.sin(theta)
        close=(dot>.9995)
        den[close]=1
        q=(np.sin((1-a)*theta)*q0+np.sin(a*theta)*q1)/den
        q[close]=(q0*(1-a)+q1*a)[close]
        q/=np.linalg.norm(q,axis=1)[:,None]
        return ok,p,q


class Grid:
    def __init__(self,c):
        self.c=c;self.r=c['resolution'];self.origin=np.array([0,0],dtype=np.int64)
        self.score=np.zeros((0,0),np.int16);self.hits=np.zeros((0,0),np.uint8)
        self.occupied=np.zeros((0,0),bool)

    def ensure(self,cells):
        if not len(cells):return
        lo=cells.min(axis=0)-1;hi=cells.max(axis=0)+2
        if self.score.size:
            lo=np.minimum(lo,self.origin);hi=np.maximum(hi,self.origin+np.array(self.score.shape[::-1]))
        shape=(hi-lo)[::-1]
        if int(shape[0])*int(shape[1])>self.c['max_cells']:
            raise RuntimeError('Slice map exceeds 4,194,304 cells; recorded data retained')
        if self.score.size and np.array_equal(lo,self.origin) and tuple(shape)==self.score.shape:return
        # Grow in tiles to avoid reallocating for every marginal ray.
        lo=(lo//64)*64;hi=((hi+63)//64)*64;shape=(hi-lo)[::-1]
        if int(shape[0])*int(shape[1])>self.c['max_cells']:
            raise RuntimeError('Slice map allocation limit reached')
        old=self.origin-lo;ys=slice(old[1],old[1]+self.score.shape[0]);xs=slice(old[0],old[0]+self.score.shape[1])
        for name,dtype in [('score',np.int16),('hits',np.uint8),('occupied',bool)]:
            a=np.zeros(shape,dtype);a[ys,xs]=getattr(self,name);setattr(self,name,a)
        self.origin=lo

    def update(self,origins,ends):
        if not len(ends):return 0,0
        low=self.c['offset']-self.c['thickness']/2;high=low+self.c['thickness']
        hit=(ends[:,2]>=low)&(ends[:,2]<=high)
        hc=np.floor(ends[hit,:2]/self.r).astype(np.int64)
        # Only observed ray segments inside the physical slab can clear space.
        d=ends-origins;dz=d[:,2];flat=np.abs(dz)<1e-10
        safe=np.where(flat,1,dz)
        a=(low-origins[:,2])/safe;b=(high-origins[:,2])/safe
        enter=np.maximum(0,np.minimum(a,b));leave=np.minimum(1,np.maximum(a,b))
        enter[flat]=0;leave[flat]=1
        valid=(leave>enter)&(~flat|((origins[:,2]>=low)&(origins[:,2]<=high)))
        ids=np.flatnonzero(valid)
        # Bound free-space work, never subsample obstacle endpoints.
        if len(ids)>self.c['max_rays']:
            ids=ids[np.linspace(0,len(ids)-1,self.c['max_rays'],dtype=int)]
        free=[]
        for chunk in np.array_split(ids,max(1,math.ceil(len(ids)/128))):
            if not len(chunk):continue
            start=origins[chunk,:2]+d[chunk,:2]*enter[chunk,None]
            end=origins[chunk,:2]+d[chunk,:2]*leave[chunk,None]
            length=np.linalg.norm(end-start,axis=1)
            steps=np.maximum(1,np.ceil(length/(self.r*.45)).astype(int))
            k=np.arange(steps.max()+1)[None,:];f=k/steps[:,None]
            sample=start[:,None,:]+(end-start)[:,None,:]*f[:,:,None]
            # Keep a whole cell away from hit endpoints to avoid grazing erosion.
            good=(k<=steps[:,None])&((1-f)*length[:,None]>self.r*1.5)
            free.append(np.floor(sample[good]/self.r).astype(np.int64))
        fc=np.concatenate(free) if free else np.empty((0,2),np.int64)
        cells=np.concatenate((hc,fc));self.ensure(cells)
        if not self.score.size:return 0,0
        def indices(x):
            # Exact row-major indices within checked bounds, not a collision-prone hash.
            x=x-self.origin
            ids=np.unique(x[:,1]*self.score.shape[1]+x[:,0])
            return ids//self.score.shape[1],ids%self.score.shape[1]
        fy,fx=indices(fc);hy,hx=indices(hc)
        self.score[fy,fx]=np.maximum(-20,self.score[fy,fx]-1)
        self.score[hy,hx]=np.minimum(20,self.score[hy,hx]+5)
        self.hits[hy,hx]=np.minimum(255,self.hits[hy,hx].astype(int)+1)
        self.occupied[hy,hx]|=(self.hits[hy,hx]>=2)&(self.score[hy,hx]>=4)
        return len(hx),len(fx)

    def raster(self):
        a=np.full(self.score.shape,-1,np.int8)
        a[self.score<=-2]=0;a[self.occupied]=100
        return a


def png(grid):
    from PIL import Image
    a=grid.raster();pixels=np.full(a.shape,205,np.uint8)
    pixels[a==0]=254;pixels[a==100]=0
    output=io.BytesIO();Image.fromarray(pixels[::-1]).save(output,format='PNG')
    return output.getvalue()


class Engine:
    def __init__(self,c):
        self.c=c;self.poses=Poses(c['max_pose_gap_ns']);self.grid=Grid(c)
        self.acc=[];self.level=None;self.anchor=None;self.pending=deque()
        self.trajectory=[];self.last_path=-1;self.latest_ns=0
        self.stats=dict(front_points=0,back_points=0,front_hits=0,back_hits=0,
                        invalid_points=0,self_filtered=0,range_filtered=0,pose_unmatched=0,
                        queue_dropped_points=0,frames=0,hit_cells=0,free_cells=0,max_frame_ms=0.)
        self.last_sensor=[0,0]

    def imu(self,m):
        if len(self.acc)<self.c['gravity_samples']:
            v=m.linear_acceleration;a=[v.x,v.y,v.z]
            if np.isfinite(a).all():self.acc.append(a)
        self.initialize()

    def pose(self,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        self.poses.add(stamp_ns(m),[p.x,p.y,p.z],[q.x,q.y,q.z,q.w],m.header.frame_id)
        self.initialize();self.drain()
        if self.level is not None and stamp_ns(m)-self.last_path>=100000000:
            xyz=self.level@(np.array([p.x,p.y,p.z])-self.anchor)
            quat=(Rotation.from_matrix(self.level)*Rotation.from_quat([q.x,q.y,q.z,q.w])).as_quat()
            self.trajectory.append([stamp_ns(m),*xyz,*quat]);self.last_path=stamp_ns(m)
            if len(self.trajectory)>10000:self.trajectory=self.trajectory[::2]

    def initialize(self):
        if self.level is not None or self.poses.first is None or len(self.acc)<self.c['gravity_samples']:return
        _,p,q=self.poses.first;rot=Rotation.from_quat(q).as_matrix()
        up=rot@np.mean(self.acc,axis=0)
        if np.linalg.norm(up)<.1:raise ValueError('Cannot determine initial gravity')
        up/=np.linalg.norm(up)
        # Preserve as much original yaw as possible while leveling the frame.
        self.level=Rotation.align_vectors([[0,0,1]],[up])[0].as_matrix()
        self.anchor=p+rot@np.asarray(self.c['imu_from_front_T'])

    def cloud(self,m,sensor):
        stamp=stamp_ns(m)
        if stamp<=self.last_sensor[sensor]:raise ValueError('Repeated/backwards slice cloud')
        self.last_sensor[sensor]=stamp
        p,ns,bad=raw_points(m);self.stats['invalid_points']+=bad
        self.pending.append((sensor,p,ns))
        while len(self.pending)>40:
            _,x,_=self.pending.popleft();self.stats['queue_dropped_points']+=len(x)
        self.drain()

    def drain(self,final=False):
        if self.level is None:return
        remaining=deque()
        while self.pending:
            sensor,p,ns=self.pending.popleft()
            if not len(ns):continue
            if not final and (not self.poses.data or ns.max()>self.poses.data[-1][0]):
                remaining.append((sensor,p,ns));continue
            ok,pos,q=self.poses.interpolate(ns);self.stats['pose_unmatched']+=int((~ok).sum())
            if not ok.any():continue
            begin=time.perf_counter()
            self.integrate(sensor,p[ok],pos,q)
            self.stats['max_frame_ms']=max(self.stats['max_frame_ms'],(time.perf_counter()-begin)*1000)
            self.latest_ns=max(self.latest_ns,int(ns[ok].max()))
        self.pending=remaining

    def integrate(self,sensor,p,pos,q):
        key='back' if sensor else 'front';self.stats[key+'_points']+=len(p)
        origin=np.array(self.c['back_origin_front'] if sensor else [0,0,0])
        base=np.asarray(self.c['base_from_front']);b=p@base[:3,:3].T+base[:3,3]
        x0,x1,y0,y1=self.c['body_xy'];pad=self.c['body_padding']
        body=(b[:,0]>=x0-pad)&(b[:,0]<=x1+pad)&(b[:,1]>=y0-pad)&(b[:,1]<=y1+pad)
        r=np.linalg.norm(p-origin,axis=1)
        within=(r>=self.c['min_range'])&(r<=self.c['max_range'])
        self.stats['self_filtered']+=int(body.sum());self.stats['range_filtered']+=int((~within).sum())
        good=~body&within;p=p[good];pos=pos[good];q=q[good]
        if not len(p):return
        ir=np.asarray(self.c['imu_from_front_R']);it=np.asarray(self.c['imu_from_front_T'])
        rot=Rotation.from_quat(q)
        ends=(rot.apply(p@ir.T+it)+pos-self.anchor)@self.level.T
        origins=(rot.apply(np.broadcast_to(ir@origin+it,p.shape))+pos-self.anchor)@self.level.T
        h,f=self.grid.update(origins,ends)
        self.stats[key+'_hits']+=h;self.stats['hit_cells']+=h;self.stats['free_cells']+=f;self.stats['frames']+=1

    def snapshot(self):
        a=self.grid.raster()
        result=dict(config=self.c,statistics=self.stats.copy(),frame=self.c['frame'],
                    source_frame=self.poses.frame,stamp_ns=self.latest_ns,
                    trajectory=[p[1:4] for p in self.trajectory],
                    pose=({'xyz':self.trajectory[-1][1:4],'quaternion_xyzw':self.trajectory[-1][4:]} if self.trajectory else None),
                    ready=bool(a.size and np.any(a==100) and np.any(a==0)))
        if self.level is not None:
            result['map_from_world_R']=self.level.tolist();result['world_front_anchor']=self.anchor.tolist()
        if a.size:
            result['grid']=dict(width=a.shape[1],height=a.shape[0],resolution=self.grid.r,
                                origin=(self.grid.origin*self.grid.r).tolist(),
                                occupied=int((a==100).sum()),free=int((a==0).sum()),unknown=int((a==-1).sum()),
                                png_base64=base64.b64encode(png(self.grid)).decode())
        return result
