"""Navigation contracts and deterministic lease controller, independent of ROS/SDK."""
import hashlib
import json
import math
from pathlib import Path
import secrets


SCAN_MAX_SOURCE_AGE = 1.0
BROWSER_LEASE_SECONDS = 2.0
LOCALIZATION_MAX_SOURCE_AGE = .8


def costmap_inflation_radius(config):
    """Smac's center-cost fast path must cover every padded footprint corner."""
    requested=float(config.get('inflation_radius',.85))
    padding=float(config['footprint_padding'])
    outer=max(math.hypot(abs(x)+padding,abs(y)+padding) for x,y in config['footprint'])
    # Cover cell discretization as well as the circumscribed footprint radius.
    minimum=math.ceil((outer+math.sqrt(2)*.05)/.05)*.05
    if not math.isfinite(requested) or not 0<=requested<=2 or minimum>2:
        raise ValueError('膨胀半径或机身尺寸超出支持范围')
    return max(requested,minimum)


def localization_fresh(receive_age,source_age):
    # AMCL follows the derived scan pipeline; receipt still has a strict watchdog.
    return 0<=receive_age<.5 and -.05<=source_age<=LOCALIZATION_MAX_SOURCE_AGE


def sensor_fresh(key,receive_age,source_age):
    # Derived scan projection may lag a few frames. Raw lidars, IMU and LIO
    # still require 0.5 s freshness, and every stream must keep arriving.
    maximum=SCAN_MAX_SOURCE_AGE if key=='scan' else .5
    return 0<=receive_age<.5 and -.05<=source_age<=maximum


class Convergence:
    """Require a continuous valid localization window, not time since clicking."""
    def __init__(self,duration=2.):
        self.duration=duration;self.since=None

    def reset(self):self.since=None

    def update(self,valid,now):
        if not valid:self.since=None
        elif self.since is None:self.since=now
        return self.since is not None and now-self.since>=self.duration


def speed(value):
    if isinstance(value, bool):
        raise ValueError('速度必须是数字')
    value = float(value)
    if not math.isfinite(value) or not 0 <= value <= .30:
        raise ValueError('速度必须在 0–0.30 m/s 范围内')
    return value


def limited(v, w, maximum):
    maximum = speed(maximum)
    if not all(math.isfinite(x) for x in (v, w)) or v < 0:
        raise ValueError('仅接受有限的前进和转向速度')
    if maximum == 0:
        return 0., 0.
    scale = min(1., maximum / v if v else 1., .1 / abs(w) if w else 1.)
    return v * scale, w * scale


def controller_parameters(value):
    value = speed(value)
    if value == 0:
        raise ValueError('速度为 0：保持静止，不启动导航')
    return {'FollowPath.desired_linear_vel': value,
            'FollowPath.regulated_linear_scaling_min_speed': min(.02, value),
            'FollowPath.min_approach_linear_velocity': min(.015, value),
            'progress_checker.required_movement_radius': min(.05, value * 10.)}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_config(config, hardware=False):
    import numpy as np
    for name in ('base_from_lidar', 'imu_from_lidar'):
        t = np.asarray(config[name], dtype=float)
        if t.shape != (4, 4) or not np.isfinite(t).all() or not np.allclose(t[3], [0,0,0,1]):
            raise ValueError('Invalid transform: ' + name)
        if not np.allclose(t[:3,:3].T @ t[:3,:3], np.eye(3), atol=1e-4) or abs(np.linalg.det(t[:3,:3])-1)>1e-4:
            raise ValueError('Transform must be a proper rotation: ' + name)
    poly = np.asarray(config['footprint'], float)
    if poly.ndim != 2 or poly.shape[1] != 2 or len(poly)<3 or not np.isfinite(poly).all():
        raise ValueError('Invalid footprint')
    area = abs(sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(poly, np.roll(poly,-1,axis=0))))/2
    edges=np.roll(poly,-1,axis=0)-poly
    crosses=np.cross(edges,np.roll(edges,-1,axis=0))
    if (np.linalg.norm(edges,axis=1)<1e-6).any() or not ((crosses>0).all() or (crosses<0).all()):
        raise ValueError('Footprint must be strictly convex with unique vertices')
    if area < .1:
        raise ValueError('Footprint is too small')
    if not 0 <= config['obstacle_min_height'] < config['obstacle_max_height'] <= 3:
        raise ValueError('Invalid obstacle height')
    if not 0 < config['slice_thickness'] <= .5:
        raise ValueError('Invalid slice thickness')
    for box in config.get('self_filter_boxes',[]):
        lower,upper=np.asarray(box['min'],float),np.asarray(box['max'],float)
        if lower.shape!=(3,) or upper.shape!=(3,) or not np.isfinite([lower,upper]).all() or not (lower<upper).all():
            raise ValueError('Invalid self filter box')
    if not hardware:
        return
    speed(config['max_speed'])
    if config['max_speed'] <= 0:
        raise ValueError('导航最高速度必须大于 0')
    for key, maximum in (('feedback_timeout', .5), ('max_position_error', .1), ('max_yaw_error', .2)):
        if not 0 < config[key] <= maximum:
            raise ValueError('Invalid control limit: ' + key)


class Lease:
    """Owner is per page, task IDs invalidate all late callbacks and heartbeats."""
    def __init__(self):
        self.owner = None
        self.task = None
        self.until = -math.inf
        self.active = False
        self.generation = 0

    def claim(self, owner, now):
        if not isinstance(owner, str) or not 16 <= len(owner) <= 128:
            raise ValueError('Invalid browser session')
        if self.owner and self.owner != owner and (self.active or now < self.until):
            raise ValueError('另一浏览器持有控制许可；当前页面只读')
        self.owner, self.until = owner, now + BROWSER_LEASE_SECONDS

    def begin(self, owner, now):
        self.claim(owner, now)
        if self.active:
            raise ValueError('已有导航任务')
        self.generation += 1
        self.task = secrets.token_hex(16)
        self.active = True
        return self.task

    def heartbeat(self, owner, task, now):
        if self.owner != owner or self.task != task or not self.active or now >= self.until:
            return False
        self.until = now + BROWSER_LEASE_SECONDS
        return True

    def valid(self, task, now):
        return self.active and task == self.task and now < self.until

    def stop(self):
        old = self.task
        self.active = False
        self.task = None
        self.until = -math.inf
        self.generation += 1
        return old


class Drive:
    """Feedback-governed local-coordinate targets; never integrates missed time."""
    def __init__(self, backend, config, now):
        self.backend, self.config = backend, config
        self.target = list(backend.desired())
        if len(self.target)!=3 or not all(math.isfinite(x) for x in self.target):
            raise ValueError('Invalid SDK desired position')
        self.task = None
        self.limit = 0.
        self.last_tick = now
        self.command_time = self.permit_time = -math.inf
        self.command_value = (0.,0.)
        self.fault = None
        self.moving = False
        self.target_sent = False
        self.tracking = None
        self.warning = None
        self.holding = False
        self.hold_since = None
        self.hold_deadline = None
        self.hold_sample = None
        self.seed_target = False

    def prepare(self, task, value):
        if self.fault or self.task or self.holding:
            raise ValueError('执行器未就绪')
        value = speed(value)
        if not 0 < value <= self.config['max_speed']:
            raise ValueError('速度超过配置的导航上限')
        self.target = list(self.backend.desired())
        self.target_sent = False
        self.seed_target = True
        self.warning = None
        self.task, self.limit = task, value
        self.command_time = self.permit_time = -math.inf
        self.command_value = (0.,0.)

    def command(self, task, v, w, now):
        if task != self.task or self.fault:
            return False
        self.command_value = limited(v,w,self.limit)
        self.command_time = now
        return True

    def backup_command(self,task,v,now):
        # Only the explicitly gated coordinator recovery uses signed velocity.
        # Ordinary navigation remains forward-only via command()/limited().
        if not self.task or task!=self.task or self.fault:return False
        if not math.isfinite(v) or not -.03<=v<=0:
            raise ValueError('回退恢复仅允许 0–0.03 m/s 的直线后退')
        self.command_value=(max(v,-self.limit),0.)
        self.command_time=now
        return True

    def permit(self, task, now):
        if task == self.task and not self.fault:
            self.permit_time = now

    def stop(self, reason=None):
        self.task = None
        self.holding = False
        self.command_value = (0.,0.)
        self.moving = False
        if reason:
            self.fault = reason
        if self.backend.has_control():
            try:
                self.backend.stop()
            except Exception as exc:
                self.fault = '停车请求失败：' + str(exc)
        if getattr(self.backend, 'requires_restart', False):
            self.fault = self.fault or '停车已锁定；需重新启动会话'

    def feedback(self, now):
        p,received=self.backend.feedback()
        if (len(p)!=3 or not all(math.isfinite(x) for x in p) or
                not -.02<=now-received<=self.config['feedback_timeout']):
            raise RuntimeError('底盘反馈无效或超时')
        return list(p),received

    def suspend(self, now, reason):
        """Cancel the task and request a measured-position hold; verify settling.
        This is a target request, not a claimed physical brake guarantee.
        """
        try:
            if self.fault: return False
            if not self.backend.has_control(): raise RuntimeError('底盘控制权丢失')
            p,received=self.feedback(now)
            self.task=None;self.command_value=(0.,0.);self.moving=False
            self.command_time=self.permit_time=-math.inf
            self.warning=reason+'；已取消任务，确认静止后请重新提交'
            if not self.holding:
                self.target=p
                if self.target_sent:
                    self.backend.send(list(p))
                self.holding=True;self.hold_since=now;self.hold_deadline=now+3.
                self.hold_sample=(list(p),received)
            return True
        except Exception as exc:
            self.stop(str(exc));return False

    def cancel_stationary(self, now, stationary):
        # Compatibility endpoint: settling is now verified here from SDK feedback.
        return self.suspend(now,'导航任务结束或取消')

    def tick(self, now):
        dt=now-self.last_tick;self.last_tick=now
        try:
            if self.fault:return (0.,0.)
            if not math.isfinite(dt) or dt<=0:raise RuntimeError('控制时钟无效')
            if not self.backend.has_control():raise RuntimeError('底盘控制权丢失')
            p,received=self.feedback(now)
            if self.holding:
                previous,previous_stamp=self.hold_sample
                close=(math.hypot(p[0]-previous[0],p[1]-previous[1])<.005
                       and abs(p[2]-previous[2])<.01)
                if not close:
                    self.hold_sample=(list(p),received);self.hold_since=now
                if self.target_sent:self.backend.send(list(self.target))
                if close and received!=previous_stamp and now-self.hold_since>=.15:
                    self.holding=False
                elif now>=self.hold_deadline:
                    raise RuntimeError('位置保持请求后未确认静止')
                return (0.,0.)
            if not self.task:return (0.,0.)
            if now-self.permit_time>=.5:
                self.suspend(now,'运动许可失效');return (0.,0.)
            if now-self.command_time>=.3:
                self.suspend(now,'导航指令超时');return (0.,0.)
            if dt>.1:
                self.warning='控制周期延迟 %.3f s；本周期不推进目标，不补发位移'%dt
                return (0.,0.)
            if self.seed_target:
                # Explicit new task discards a previous desired-position backlog.
                # Seed only after obtaining fresh native SDK feedback.
                self.target=list(p);self.seed_target=False
            v,w=self.command_value
            before=list(self.target)
            if (v,w)==(0.,0.):
                # Withdraw unexecuted lead instead of retaining a residual move.
                if self.target_sent:
                    self.target=list(p);self.backend.send(list(p))
                self.moving=False
                self.tracking=dict(target=list(self.target),feedback=p,position_error_m=math.hypot(self.target[0]-p[0],self.target[1]-p[1]),
                    yaw_error_rad=abs(self.target[2]-p[2]),feedback_receive_age_s=now-received,
                    tick_interval_s=dt,sampled_at=now,requested_command=[v,w],
                    applied_command=[0.,0.],governor_scale=1.)
                return (0.,0.)
            step=min(dt,.01)
            delta=[v*step,0.,w*step]
            limits=(self.config['max_position_error'],self.config['max_yaw_error'])
            gap=math.hypot(before[0]-p[0],before[1]-p[1])
            angle=abs(before[2]-p[2])
            # No proportional throttling for normal tracking lag. Advance at
            # the requested rate, clipped only to the remaining hard budget.
            scale=1.
            # Clip the increment to the remaining budget instead of rejecting
            # the entire tick at the boundary (which creates zero-speed pulses).
            if delta[0]:
                radius=max(gap,limits[0])
                dx=before[0]-p[0];dy=before[1]-p[1]
                edge=math.sqrt(max(0.,radius*radius-dy*dy))
                available=(edge-dx) if delta[0]>0 else (edge+dx)
                scale=min(scale,max(0.,available/abs(delta[0])))
            if delta[2]:
                radius=max(angle,limits[1]);offset=before[2]-p[2]
                available=radius-offset if delta[2]>0 else radius+offset
                scale=min(scale,max(0.,available/abs(delta[2])))
            candidate=[before[i]+delta[i]*scale for i in range(3)]
            applied=(v*scale,w*scale)
            self.warning=('底盘跟随偏差接近领先预算：位置 %.3f m、角度 %.3f rad；领先预算增量裁剪 %.0f%%'
                          %(gap,angle,100*scale)) if scale<.999 else None
            self.tracking=dict(target=list(candidate),feedback=list(p),
                position_error_m=math.hypot(candidate[0]-p[0],candidate[1]-p[1]),
                yaw_error_rad=abs(candidate[2]-p[2]),feedback_receive_age_s=now-received,
                tick_interval_s=dt,sampled_at=now,requested_command=[v,w],
                applied_command=list(applied),governor_scale=scale)
            self.target=candidate
            # Do not send an inherited target if no motion has been requested yet.
            if scale>0 or self.target_sent:
                self.backend.send(list(candidate));self.target_sent=True
            self.moving=bool(applied[0] or applied[1])
            return applied
        except Exception as exc:
            self.stop(str(exc));return (0.,0.)


def navigation_command(command, command_age, feedback_age):
    """Hold zero during a live Nav2 recovery; never reuse an old velocity."""
    if command_age<.3:return tuple(command)
    if feedback_age>=1.:raise RuntimeError('Nav2 动作反馈超时')
    if command_age>=15.:raise RuntimeError('Nav2 连续 15 秒没有速度输出')
    return (0.,0.)


def nav2_failure_reason(log_path,status,start_offset=0):
    """Use only this goal's log region; never attribute an older task's error."""
    import re
    generic='Nav2 导航未成功（动作状态 %s）'%status
    try:
        with Path(log_path).open('rb') as stream:
            end=stream.seek(0,2)
            stream.seek(max(start_offset,end-32768))
            lines=stream.read().decode('utf-8',errors='replace').splitlines()
        for line in reversed(lines):
            match=re.search(r'Timed out while waiting for action server to acknowledge goal request for (\w+)',line)
            if match:return generic+'：动作请求应答超时（'+match.group(1)+'）'
        for line in reversed(lines):
            for text,reason in [('no valid path found','没有满足机身边界与转弯半径的路径'),('Starting point in lethal space','起点不满足碰撞边界，请检查实际位置与定位'),('Planned route violates full footprint clearance','路径不满足完整机身避障余量'),('Failed to make progress','控制器持续未取得进展'),('No valid trajectories','控制器没有可行轨迹'),('detected collision ahead','控制器预测到前方碰撞'),('Collision detected ahead','控制器预测到前方碰撞')]:
                if text in line:return generic+'：'+reason
    except OSError:pass
    return generic
