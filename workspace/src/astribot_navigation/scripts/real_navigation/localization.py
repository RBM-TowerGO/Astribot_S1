"""Map alignment diagnostics and bounded recovery, independent of ROS/SDK.

All points and poses are measured at the same scan reference time. Unknown
cells and the interiors of thick walls are never successful endpoint matches.
"""
from collections import deque
import math
import time
import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt
from scipy.optimize import minimize


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def transform(points, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + pose[:2]


class MapMatcher:
    def __init__(self, grid, laser_origin=(0., 0.)):
        self.grid = grid
        self.origin = np.asarray(laser_origin, float)
        # Only occupied cells touching observed free space are wall boundaries.
        boundary = grid.occupied & binary_dilation(grid.free)
        self.distance = (distance_transform_edt(~boundary)*grid.resolution
                         if boundary.any() else np.full(grid.free.shape, 100.))

    def lookup(self, points):
        cells = self.grid.cells(points)
        x, y = cells.T
        inside = (x >= 0) & (y >= 0) & (x < self.grid.width) & (y < self.grid.height)
        known = np.zeros(len(cells), bool)
        occupied = known.copy()
        dist = np.full(len(cells), 2.)
        occupied[inside] = self.grid.occupied[y[inside], x[inside]]
        known[inside] = occupied[inside] | self.grid.free[y[inside], x[inside]]
        dist[inside] = self.distance[y[inside], x[inside]]
        dist[~known] = 2.
        return known, occupied, dist

    def evaluate(self, points, pose, rays=True):
        points = np.asarray(points, float).reshape((-1, 2))
        ranges = np.linalg.norm(points-self.origin, axis=1)
        points = points[np.isfinite(points).all(axis=1) & (ranges >= .15) & (ranges <= 12.)]
        if not len(points):
            return dict(good=False, center_known=False, reason='缺少有效扫描点', points=0,
                        score=4., known_fraction=0., match_fraction=0., median_distance_m=2., quadrants=0,
                        ray_conflict_fraction=1., ray_known_fraction=0.)
        world = transform(points, pose)
        known, occupied, d = self.lookup(world)
        ck, co, _ = self.lookup(np.asarray([pose[:2]]))
        center = bool(ck[0] and not co[0])
        a = np.arctan2((points-self.origin)[:, 1], (points-self.origin)[:, 0])
        quadrants = len(np.unique(np.floor((a+math.pi)/(math.pi/2)).astype(int) % 4))
        ray_conflict = 0.; ray_known = 1.; ray_occupied = 0.
        if rays:
            # Sample at half-cell spacing, including thin walls; omit endpoint
            # vicinity to tolerate discretization, never the whole ray interior.
            sampled = points[::max(1, len(points)//72)]
            delta = sampled-self.origin
            length = np.linalg.norm(delta, axis=1)
            steps = np.arange(self.grid.resolution/2, 12., self.grid.resolution/2)
            valid = (steps[None, :] < length[:, None]-.15)
            local = self.origin+delta[:, None, :]/length[:, None, None]*steps[None, :, None]
            k, o, _ = self.lookup(transform(local.reshape((-1, 2)), pose))
            k = k.reshape(valid.shape); o = o.reshape(valid.shape)
            ray_conflict = float(np.mean(np.any(o & valid, axis=1)))
            ray_known = float(np.sum(k & valid)/max(1, np.sum(valid)))
            ray_occupied = float(np.sum(o & valid)/max(1, np.sum(valid)))
        reasons = []
        if len(points) < 100: reasons.append('有效扫描点不足 100')
        if quadrants < 3: reasons.append('扫描方位覆盖不足')
        if not center: reasons.append('位置位于障碍、未知区域或地图外')
        if np.mean(known) < .7: reasons.append('已知区域覆盖不足 70%')
        if np.mean(d <= .2) < .6: reasons.append('墙面匹配不足 60%')
        if np.median(d) > .15: reasons.append('墙面距离中位数超过 15 cm')
        # Original mapping rays intersect isolated sticky cells frequently.
        # Bound occupied path length (observed original maximum 10.6%), not
        # the fraction of rays touching even one noisy cell.
        if ray_occupied > .12 or ray_known < .7: reasons.append('扫描射线与地图自由空间不一致')
        score = float(np.mean(np.minimum(d, 1.)**2) + .5*(1-np.mean(known)) +
                      2*ray_occupied + .05*ray_conflict + .5*(1-ray_known) + (0 if center else 2))
        return dict(good=not reasons, reason='；'.join(reasons) or None, center_known=center,
                    points=len(points), quadrants=quadrants, known_fraction=float(np.mean(known)),
                    match_fraction=float(np.mean(d <= .2)), median_distance_m=float(np.median(d)),
                    ray_conflict_fraction=ray_conflict, ray_known_fraction=ray_known, ray_occupied_fraction=ray_occupied, score=score)

    def search(self, frames, anchor, deadline, cancelled=lambda: False):
        """Coarse multi-hypothesis search, bounded refinement and five-frame proof."""
        if len(frames) < 5 or len({f[0] for f in frames}) < 5:
            return dict(ok=False, reason='需要五个不同时间戳的静止扫描')
        anchor = np.asarray(anchor, float)
        bounds = [(anchor[0]-1, anchor[0]+1), (anchor[1]-1, anchor[1]+1),
                  (anchor[2]-math.pi/6, anchor[2]+math.pi/6)]
        first = np.asarray(frames[0][1]); sparse = first[::max(1, len(first)//120)]
        def check():
            if cancelled() or time.monotonic() >= deadline:
                raise TimeoutError('局部搜索取消或超过恢复期限')
        def loss(p, points=sparse, rays=False):
            check()
            return self.evaluate(points, p, rays=rays)['score']
        try:
            candidates = []
            for yaw in np.linspace(*bounds[2], 13):
                for x in np.linspace(*bounds[0], 11):
                    for y in np.linspace(*bounds[1], 11):
                        p = np.array([x, y, yaw]); candidates.append((loss(p), p))
            candidates.sort(key=lambda v: v[0])
            seeds = []
            for _, p in candidates:
                if all(np.linalg.norm(p[:2]-q[:2]) >= .25 or abs(angle(p[2]-q[2])) >= math.radians(5) for q in seeds):
                    seeds.append(p)
                if len(seeds) >= 12: break
            refined = []
            for seed in seeds:
                local_bounds = [(max(lo, v-step), min(hi, v+step))
                                for (lo, hi), v, step in zip(bounds, seed, [.2, .2, math.radians(5)])]
                result = minimize(loss, seed, method='Powell', bounds=local_bounds,
                                  options=dict(maxiter=14, xtol=.005, ftol=.0001))
                check(); metrics = self.evaluate(first, result.x)
                refined.append((metrics['score'], result.x, metrics))
            refined.sort(key=lambda v: v[0]); best = refined[0]
            alternatives = [v for v in refined[1:] if np.linalg.norm(v[1][:2]-best[1][:2]) >= .25 or
                            abs(angle(v[1][2]-best[1][2])) >= math.radians(5)]
            # Refinement may converge every seed to the same solution. Preserve
            # separated coarse hypotheses instead of calling uniqueness ambiguous.
            for _, p in candidates:
                if np.linalg.norm(p[:2]-best[1][:2]) >= .25 or abs(angle(p[2]-best[1][2])) >= math.radians(5):
                    check(); metrics=self.evaluate(first,p)
                    alternatives.append((metrics['score'],p,metrics))
                    if len(alternatives)>=24:break
            alternatives.sort(key=lambda v:v[0])
            if not alternatives or best[0] > .8*alternatives[0][0] or alternatives[0][0] <= 1e-9:
                return dict(ok=False, reason='局部搜索存在歧义或缺少可比较次优解')
            if not best[2]['good']:
                return dict(ok=False, reason='最佳候选仍未通过扫描匹配检查', metrics=best[2])
            estimates = []
            for _, points in frames:
                small = [(max(lo, v-step), min(hi, v+step)) for (lo,hi),v,step in
                         zip(bounds,best[1],[.12,.12,math.radians(4)])]
                points = np.asarray(points)
                result = minimize(lambda p: loss(p, points, False), best[1], method='Powell', bounds=small,
                                  options=dict(maxiter=12, xtol=.003, ftol=.0001))
                check()
                if not self.evaluate(points, best[1])['good'] or not self.evaluate(points, result.x)['good']:
                    return dict(ok=False, reason='候选未通过五帧一致性检查')
                estimates.append(result.x)
            estimates = np.asarray(estimates)
            spread = max(np.linalg.norm(a[:2]-b[:2]) for a in estimates for b in estimates)
            yaw_spread = max(abs(angle(a[2]-b[2])) for a in estimates for b in estimates)
            if spread > .1 or yaw_spread > math.radians(3):
                return dict(ok=False, reason='五帧位置或朝向离散过大')
            return dict(ok=True, pose=best[1].tolist(), metrics=best[2], position_spread_m=float(spread),
                        yaw_spread_rad=float(yaw_spread), runner_up_score=float(alternatives[0][0]))
        except TimeoutError as exc:
            return dict(ok=False, reason=str(exc))


class QualityMonitor:
    def __init__(self):
        self.reset()

    def reset(self):
        self.last_stamp = None; self.bad_since = None; self.outside_since = None
        self.good_since = None; self.last_trusted = None; self.invalid = False
        self.stationary = deque(); self.last_received = None
        self.state = dict(status='unavailable', reason='等待扫描地图匹配', last_trusted_pose=None)

    def update(self, stamp, now, metrics, pose, stationary):
        if self.last_stamp is not None and stamp <= self.last_stamp: return self.state
        if self.last_received is not None and now-self.last_received > .8:
            self.good_since=None;self.bad_since=None;self.outside_since=None;self.stationary.clear()
        self.last_stamp = stamp; self.last_received = now
        if not stationary: self.stationary.clear()
        else:
            self.stationary.append((now, list(pose)))
            while len(self.stationary)>1 and self.stationary[1][0] <= now-10: self.stationary.popleft()
        drift = [0., 0.]
        if self.stationary:
            p = self.stationary[0][1]
            drift = [math.dist(p[:2], pose[:2]), abs(angle(p[2]-pose[2]))]
        drifting = drift[0] > .1 or drift[1] > math.radians(3)
        good = metrics['good'] and not drifting
        if good:
            if self.good_since is None: self.good_since = now
            self.bad_since = None
        else:
            self.good_since = None
            if self.bad_since is None: self.bad_since = now
        if metrics['center_known']: self.outside_since = None
        elif self.outside_since is None: self.outside_since = now
        if ((self.bad_since is not None and now-self.bad_since >= 2) or
            (self.outside_since is not None and now-self.outside_since >= 1) or drifting):
            self.invalid = True
        if good and not self.invalid and self.good_since is not None and now-self.good_since >= 2:
            self.last_trusted = list(pose)
        status = ('untrusted' if self.invalid else 'trusted' if good and self.last_trusted is not None else 'checking')
        self.state = dict(status=status, reason='静止时地图定位持续漂移' if drifting else metrics['reason'],
                          scan_stamp=stamp, metrics=metrics, stationary_drift_m=drift[0], stationary_drift_rad=drift[1],
                          last_trusted_pose=self.last_trusted)
        return self.state

    def usable(self, now):
        # Brief quality dips keep an already trusted estimate until debounce;
        # stale data never does. New initialization requires continuous quality.
        return (not self.invalid and self.last_trusted is not None and self.last_received is not None
                and 0 <= now-self.last_received < .8)


class Recovery:
    def __init__(self):
        self.generation=0; self.status='idle'; self.reason=None; self.frames=[]
        self.pose=None; self.anchor=None; self.started=0.; self.good_since=None; self.published_stamp=None

    def begin(self, anchor, now):
        self.cancel(); self.started=now; self.anchor=anchor; self.reason=None
        self.status='waiting_stationary' if anchor is not None else 'manual_required'
        if anchor is None:self.reason='没有最后可信位置，请手动选点'

    def cancel(self):
        self.generation+=1;self.status='idle';self.frames=[];self.pose=None
        self.good_since=None;self.published_stamp=None;self.reason=None

    def fail(self, reason):
        self.status='manual_required';self.reason=reason;self.frames=[];self.generation+=1

    def observe(self, now, stationary, ready):
        if self.status not in ('waiting_stationary','collecting','searching','verifying'):return
        if now-self.started >= 30:self.fail('定位恢复超时，请手动选点');return
        if self.status=='waiting_stationary':
            if stationary and ready:self.status='collecting'
        elif not stationary:self.fail('恢复期间静止反馈失效或检测到移动，请手动选点')
        elif not ready:self.fail('恢复期间传感器数据失效，请手动选点')

    def add(self, stamp, points):
        if self.status!='collecting':return False
        if self.frames and stamp<=self.frames[-1][0]:return False
        self.frames.append((stamp,np.asarray(points).copy()))
        if len(self.frames)==5:self.status='searching';return True
        return False

    def result(self, generation, result):
        if generation!=self.generation or self.status!='searching':return False
        if not result['ok']:self.fail(result['reason']);return False
        self.pose=result['pose'];return True

    def published(self, stamp):
        if self.status!='searching' or self.published_stamp is not None:return False
        self.published_stamp=stamp;self.status='verifying';self.good_since=None;return True

    def verify(self, now, good, amcl_stamp):
        if self.status!='verifying':return False
        if not good or amcl_stamp<=self.published_stamp:self.good_since=None;return False
        if self.good_since is None:self.good_since=now
        if now-self.good_since>=3:self.status='succeeded';return True
        return False

    def state(self):
        return dict(id=self.generation,status=self.status,reason=self.reason,
                    candidate=self.pose,frames=len(self.frames),source_stamp=self.published_stamp)


class UpdateExpectation:
    """AMCL need not publish a new pose until its configured motion threshold.

This never renews a source timestamp. Callers must independently require fresh
TF, current scan/map quality, healthy sensors and responsive AMCL lifecycle.
"""
    def __init__(self, distance=.05, yaw=.05):
        self.distance=distance;self.yaw=yaw;self.reset()

    def reset(self):
        self.stamp=None;self.odom=None;self.due_since=None

    def observe(self, stamp, odom):
        if odom is None or (self.stamp is not None and stamp<=self.stamp):return
        self.stamp=stamp;self.odom=list(odom);self.due_since=None

    def valid(self, now, odom):
        if self.odom is None or odom is None:return False
        due=(math.dist(odom[:2],self.odom[:2])>=self.distance or abs(angle(odom[2]-self.odom[2]))>=self.yaw)
        if due:
            if self.due_since is None:self.due_since=now
        else:self.due_since=None
        return self.due_since is None or now-self.due_since<.5
