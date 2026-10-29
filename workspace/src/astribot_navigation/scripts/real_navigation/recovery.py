"""Bounded retreat state machine and conservative swept clearance, no SDK/ROS."""
import math
import numpy as np
from .geometry import safe_pose


def recoverable_navigation_failure(status,reason,attempts):
    return status==6 and attempts<2 and any(text in reason for text in
        ('预测到前方碰撞','避障余量','可行轨迹','未取得进展','转弯半径'))


def backup_clearance(grid,pose,footprint,padding,distance,clouds):
    # Extra space covers remaining SDK target lead plus one control interval.
    sweep=distance+.15
    for d in np.linspace(0.,sweep,max(2,int(math.ceil(sweep/.01))+1)):
        p=[pose[0]-d*math.cos(pose[2]),pose[1]-d*math.sin(pose[2]),pose[2]]
        if not safe_pose(grid,p,footprint,padding):return False,'后退扫掠区域在地图上有障碍或未知格'
    poly=np.asarray(footprint,float)
    low=poly.min(axis=0)-padding;high=poly.max(axis=0)+padding;low[0]-=sweep
    for points in clouds:
        points=np.asarray(points,float)
        if points.ndim!=2 or points.shape[1]!=3 or len(points)<20 or not np.isfinite(points).all():
            return False,'缺少有效的完整实时障碍点云'
        if np.any(np.all((points[:,:2]>=low)&(points[:,:2]<=high),axis=1)):
            return False,'实时雷达检测到后退扫掠区域有障碍'
    if len(clouds)!=2:return False,'需要前后雷达共同确认回退空间'
    return True,None


class Retreat:
    def __init__(self,now,distance=.15,speed=.03):
        self.started=now;self.distance=distance;self.speed=speed
        self.phase='stopping';self.since=None;self.origin=None;self.progress=0.

    def step(self,now,feedback,stationary):
        if now-self.started>=12:raise RuntimeError('回退恢复超过 12 秒，停止并等待人工检查')
        if len(feedback)!=3 or not all(math.isfinite(x) for x in feedback):
            raise RuntimeError('回退恢复缺少有效底盘反馈')
        if self.phase in ('stopping','settling'):
            if not stationary:self.since=None
            elif self.since is None:self.since=now
            elif now-self.since>=.3:
                if self.phase=='stopping':
                    self.origin=list(feedback);self.phase='backing';self.since=None
                else:self.phase='replanning'
            return 0.
        if self.phase=='backing':
            self.progress=max(0.,self.origin[0]-feedback[0])
            if (feedback[0]-self.origin[0]>.02 or abs(feedback[1]-self.origin[1])>.03
                    or abs(feedback[2]-self.origin[2])>.05):
                raise RuntimeError('回退过程中底盘方向或侧向偏差异常')
            if self.progress>=self.distance:
                self.phase='settling';self.since=None;return 0.
            return -self.speed
        return 0.

    def snapshot(self):
        return dict(phase=self.phase,distance=self.distance,progress=round(self.progress,3),speed=self.speed,timeout=12)
