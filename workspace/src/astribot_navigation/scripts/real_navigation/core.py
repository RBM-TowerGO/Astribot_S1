"""Navigation contracts and deterministic lease controller, independent of ROS/SDK."""
import hashlib
import json
import math
from pathlib import Path
import secrets


SCAN_MAX_SOURCE_AGE = .8
BROWSER_LEASE_SECONDS = 2.0
LOCALIZATION_MAX_SOURCE_AGE = .8


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
    """100 Hz latest-only local-coordinate integrator; never catches up."""
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

    def prepare(self, task, value):
        if self.fault or self.task:
            raise ValueError('执行器未就绪')
        value = speed(value)
        if not 0 < value <= self.config['max_speed']:
            raise ValueError('速度超过配置的导航上限')
        self.target = list(self.backend.desired())
        self.target_sent = False
        self.task, self.limit = task, value
        self.command_time = self.permit_time = -math.inf
        self.command_value = (0.,0.)

    def command(self, task, v, w, now):
        if task != self.task or self.fault:
            return False
        self.command_value = limited(v,w,self.limit)
        self.command_time = now
        return True

    def permit(self, task, now):
        if task == self.task and not self.fault:
            self.permit_time = now

    def stop(self, reason=None):
        self.task = None
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

    def cancel_stationary(self, now, stationary):
        """Release a task only when already stopped; otherwise use vendor stop."""
        try:
            p,received=self.backend.feedback()
            safe=(stationary and self.backend.has_control() and not self.fault and
                  len(p)==3 and all(math.isfinite(v) for v in p) and
                  -.02<=now-received<self.config['feedback_timeout'] and
                  math.hypot(self.target[0]-p[0],self.target[1]-p[1])<.005 and
                  abs(self.target[2]-p[2])<.01 and self.command_value==(0.,0.))
            if not safe:
                self.stop('数据失效时未确认静止，已执行保护停止');return False
            self.task=None;self.command_value=(0.,0.);self.moving=False
            self.command_time=self.permit_time=-math.inf
            return True
        except Exception as exc:
            self.stop('静止确认失败：'+str(exc));return False

    def tick(self, now):
        dt = now-self.last_tick
        self.last_tick = now
        try:
            if self.fault:
                return (0.,0.)
            if not 0 < dt <= .1:
                raise RuntimeError('控制周期停顿，不补发位移')
            if not self.backend.has_control():
                raise RuntimeError('底盘控制权丢失')
            p, received = self.backend.feedback()
            if len(p)!=3 or not all(math.isfinite(x) for x in p) or not -.02 <= now-received <= self.config['feedback_timeout']:
                raise RuntimeError('底盘反馈无效或超时')
            if not self.task:
                return (0.,0.)
            if now-self.permit_time >= .5:
                raise RuntimeError('运动许可失效')
            if now-self.command_time >= .3:
                raise RuntimeError('导航指令超时')
            v,w = self.command_value
            candidate = [self.target[0]+v*.01, self.target[1], self.target[2]+w*.01]
            position_error=math.hypot(candidate[0]-p[0],candidate[1]-p[1])
            yaw_error=abs(candidate[2]-p[2])
            self.tracking=dict(target=list(candidate),feedback=list(p),position_error_m=position_error,
                               yaw_error_rad=yaw_error,feedback_receive_age_s=now-received,
                               tick_interval_s=dt,sampled_at=now)
            if position_error > self.config['max_position_error'] or yaw_error > self.config['max_yaw_error']:
                raise RuntimeError('底盘目标超前量超过控制限制：位置 %.4f/%.4f m，角度 %.4f/%.4f rad' %
                                   (position_error,self.config['max_position_error'],yaw_error,self.config['max_yaw_error']))
            if (v,w)==(0.,0.):
                # Normal controller pause follows keyboard teleop: no increment,
                # resend only our last commanded target. This is NOT a brake.
                # Cancellation, stale inputs and faults use stop() separately.
                if self.target_sent:
                    self.backend.send(list(self.target))
                self.moving = False
                return (0.,0.)
            self.target = candidate
            self.backend.send(candidate)
            self.target_sent = True
            self.moving = True
            return v,w
        except Exception as exc:
            self.stop(str(exc))
            return (0.,0.)


def navigation_command(command, command_age, feedback_age):
    """Hold zero during a live Nav2 recovery; never reuse an old velocity."""
    if command_age<.3:return tuple(command)
    if feedback_age>=1.:raise RuntimeError('Nav2 动作反馈超时')
    if command_age>=15.:raise RuntimeError('Nav2 连续 15 秒没有速度输出')
    return (0.,0.)
