"""AMCL confidence: separate units, temporal confirmation, bounded reassessment."""
import math
import numpy as np


class Confidence:
    # Retain the old per-axis limits; hysteresis is temporal, not a higher limit.
    position_variance_limit = .25  # m^2, largest XY eigenvalue
    yaw_variance_limit = .25       # rad^2

    def __init__(self):
        self.reset()

    def reset(self):
        self.details = dict(valid=False, reason='等待新的 AMCL 协方差')
        self.bad_since = self.good_since = None
        self.blocked = True
        self.status = 'waiting'

    def observe(self, covariance):
        a = np.asarray(covariance, dtype=float).reshape(6, 6)[np.ix_([0,1,5],[0,1,5])]
        valid = bool(np.isfinite(a).all() and np.allclose(a,a.T,atol=1e-8) and
                     np.linalg.eigvalsh((a+a.T)/2).min() >= -1e-10)
        self.details = dict(valid=valid, covariance_xy_yaw=[float(v) if math.isfinite(v) else None for v in a.flat])
        if not valid:
            self.details['reason']='AMCL 协方差非有限值或非有效协方差矩阵'
            return
        position = max(0.,float(np.linalg.eigvalsh(a[:2,:2]).max()))
        yaw = max(0.,float(a[2,2]))
        reasons=[]
        if position >= self.position_variance_limit:reasons.append('位置不确定性持续偏大')
        if yaw >= self.yaw_variance_limit:reasons.append('朝向不确定性持续偏大')
        self.details.update(position_variance=position,yaw_variance=yaw,
                            sigma_x_m=math.sqrt(max(0.,a[0,0])),sigma_y_m=math.sqrt(max(0.,a[1,1])),
                            sigma_position_m=math.sqrt(position),sigma_yaw_deg=math.degrees(math.sqrt(yaw)),
                            reason='；'.join(reasons) or None)

    def tick(self, now, corroborated, initialized):
        valid=self.details['valid'];high=bool(self.details.get('reason'))
        if not valid:
            self.blocked=True;self.bad_since=self.good_since=None;self.status='invalid';return False
        if high:
            self.good_since=None
            if self.bad_since is None:self.bad_since=now
            # Grace only for a previously initialized estimate with fresh trusted matching.
            if not initialized or not corroborated or now-self.bad_since>=2:self.blocked=True
            self.status='blocked' if self.blocked else 'warning'
            return not self.blocked
        self.bad_since=None
        if not corroborated:
            self.good_since=None
            return not self.blocked
        if self.good_since is None:self.good_since=now
        if self.blocked and now-self.good_since>=2:self.blocked=False
        self.status='recovering' if self.blocked else 'trusted'
        return not self.blocked

    def snapshot(self):
        return dict(self.details,status=self.status,blocked=self.blocked,
                    position_variance_limit=self.position_variance_limit,yaw_variance_limit=self.yaw_variance_limit,
                    confirmation_seconds=2,recovery_seconds=2)


class StaticReassessment:
    """Three no-motion requests / ten seconds per confidence failure, no initialpose."""
    def __init__(self):self.reset()
    def reset(self):
        self.started=None;self.last=None;self.count=0;self.last_stamp=None;self.status='idle'
    def due(self, now, eligible, stamp):
        if self.started is not None and now-self.started>=10:self.status='manual_required'
        if not eligible or self.status=='manual_required':return False
        if self.started is None:self.started=now
        if self.count>=3:
            self.status='exhausted';return False
        return (self.last is None or now-self.last>=1) and stamp!=self.last_stamp
    def sent(self,now,stamp):
        self.count+=1;self.last=now;self.last_stamp=stamp;self.status='updating'
    def snapshot(self):return dict(status=self.status,requests=self.count,maximum_requests=3,timeout_seconds=10)
