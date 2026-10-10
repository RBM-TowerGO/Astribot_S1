"""Deterministic control state machine; no ROS, SDK, or wall-clock imports."""
from dataclasses import dataclass
from enum import Enum
import math


class State(str, Enum):
    DISABLED = 'disabled'
    READY = 'ready'
    MOVING = 'moving'
    FAULT = 'fault'


@dataclass(frozen=True)
class Limits:
    max_v: float = .2
    max_w: float = .3
    accel_v: float = .2
    accel_w: float = .4
    command_timeout: float = .3
    max_command_age: float = .2
    max_cycle: float = .02
    feedback_timeout: float = .1
    max_position_error: float = .03
    max_yaw_error: float = .05

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError('All limits must be finite and positive')
        if self.max_v > .2 or self.max_w > .3 or self.command_timeout > .3:
            raise ValueError('Cannot exceed v1 hard limits')


def checked_twist(values, limits):
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('Command requires six finite components')
    if any(values[i] != 0 for i in (1, 2, 3, 4)):
        raise ValueError('Only linear.x and angular.z are supported')
    return (max(-limits.max_v, min(limits.max_v, values[0])),
            max(-limits.max_w, min(limits.max_w, values[5])))


class KeyboardLease:
    def __init__(self, duration=.2):
        self.duration = duration
        self.until = -math.inf
        self.velocity = (0., 0.)
        self.quit = False

    def feed(self, data, now):
        # A burst never represents a trajectory. A stop/unknown key wins a burst.
        keys = data.lower()
        self.quit = 'q' in keys or '\x03' in keys or '\x04' in keys
        table = {'w': (.1, 0.), 's': (-.1, 0.), 'a': (0., .2), 'd': (0., -.2)}
        if not keys or self.quit or any(k not in table for k in keys):
            self.velocity = (0., 0.)
            self.until = -math.inf
        else:
            self.velocity = table[keys[-1]]
            self.until = now + self.duration

    def sample(self, now):
        return self.velocity if now < self.until else (0., 0.)


class Controller:
    """Backend must implement position(), send(position), stop(), close().

    position() returns (xyz native chassis coordinates, monotonic receipt time).
    stop() is a separately validated backend operation, never rate-limited here.
    All calls must be serialized by the hosting process.
    """
    def __init__(self, limits=None):
        self.limits = limits or Limits()
        self.state = State.DISABLED
        self.reason = 'Hardware disabled'
        self.backend = None
        self.target = (0., 0.)
        self.velocity = (0., 0.)
        self.position_target = None
        self.last_received = None
        self.last_stamp = None
        self.last_tick = None
        self.enabled_ros_time = None
        self.zero_after_enable = False

    def enable(self, factory, now, ros_time):
        if self.backend is not None:
            raise RuntimeError('Disable and release the previous backend first')
        backend = None
        try:
            backend = factory()
            self.backend = backend
            now = now() if callable(now) else now
            ros_time = ros_time() if callable(ros_time) else ros_time
            p = self._feedback(now)
            if not backend.has_control():
                raise RuntimeError('No exclusive control rights')
            self.position_target = list(p)
            self.last_tick = now
            self.last_received = now
            self.last_stamp = None
            self.enabled_ros_time = ros_time
            self.zero_after_enable = False
            self.target = self.velocity = (0., 0.)
            self.state, self.reason = State.READY, 'Ready; waiting for a fresh key'
        except Exception as exc:
            self.fault('Enable failed: ' + str(exc))
            if backend is not None:
                backend.close()
            self.backend = None
            raise

    def _feedback(self, now):
        p, stamp = self.backend.position()
        if len(p) != 3 or not all(math.isfinite(x) for x in p):
            raise ValueError('Invalid measured chassis position')
        # A concurrent receipt can occur after the tick captured `now` but
        # before position() took the feedback lock; tolerate at most one cycle.
        if not math.isfinite(stamp) or not -self.limits.max_cycle <= now - stamp <= self.limits.feedback_timeout:
            raise ValueError('Measured chassis feedback is stale')
        return p

    def command(self, values, stamp, now, ros_time):
        if self.state not in (State.READY, State.MOVING):
            return False
        # DDS may deliver a packet already queued when enable finished. Discard
        # it without renewing the deadline; a new zero handshake is still required.
        if math.isfinite(stamp) and stamp <= self.enabled_ros_time:
            return False
        try:
            if not math.isfinite(stamp) or not 0 <= ros_time - stamp <= self.limits.max_command_age:
                raise ValueError('Stale/future command')
            if self.last_stamp is not None and stamp <= self.last_stamp:
                raise ValueError('Old/repeated command or clock reset')
            self.target = checked_twist(values, self.limits)
            if self.target == (0., 0.):
                self.zero_after_enable = True
            elif not self.zero_after_enable:
                raise ValueError('A fresh zero command is required after enabling')
            self.last_received, self.last_stamp = now, stamp
            # Zero is an explicit stop, not a slow deceleration to an old target.
            if self.target == (0., 0.) and self.state == State.MOVING:
                self.backend.stop()
                self.position_target = list(self._feedback(now))
                self.velocity = (0., 0.)
                self.state, self.reason = State.READY, 'Stopped by operator/input lease'
            return True
        except Exception as exc:
            self.fault(str(exc))
            return False

    def tick(self, now):
        if self.state not in (State.READY, State.MOVING):
            return
        try:
            dt = now - self.last_tick
            self.last_tick = now
            if not 0 < dt <= self.limits.max_cycle:
                raise RuntimeError('Control cycle overrun; no catch-up integration')
            if now - self.last_received > self.limits.command_timeout:
                raise RuntimeError('Command timeout')
            if not self.backend.has_control():
                raise RuntimeError('Control rights lost')
            p = self._feedback(now)
            error = [a-b for a, b in zip(self.position_target, p)]
            if math.hypot(*error[:2]) > self.limits.max_position_error or abs(error[2]) > self.limits.max_yaw_error:
                raise RuntimeError('Position following error')
            if self.target == (0., 0.):
                return
            def ramp(old, goal, accel):
                return old + max(-accel*dt, min(accel*dt, goal-old))
            self.velocity = (ramp(self.velocity[0], self.target[0], self.limits.accel_v),
                             ramp(self.velocity[1], self.target[1], self.limits.accel_w))
            # Contract is the SDK 202 local integrated-coordinate convention,
            # not a claim that these native values are world-frame odometry.
            self.position_target[0] += self.velocity[0] * dt
            self.position_target[2] += self.velocity[1] * dt
            self.backend.send(list(self.position_target))
            self.state, self.reason = State.MOVING, 'Driving'
        except Exception as exc:
            self.fault(str(exc))

    def fault(self, reason):
        was_active = self.state in (State.READY, State.MOVING)
        self.state, self.reason = State.FAULT, reason
        self.target = self.velocity = (0., 0.)
        if self.backend is not None and was_active:
            try:
                self.backend.stop()
            except Exception as exc:
                self.reason += '; stop failed: ' + str(exc)

    def disable(self):
        backend, self.backend = self.backend, None
        error = None
        if backend is not None:
            try:
                backend.stop()
            except Exception as exc:
                error = str(exc)
            finally:
                backend.close()
        self.target = self.velocity = (0., 0.)
        self.state = State.FAULT if error else State.DISABLED
        self.reason = 'Stop failed: ' + error if error else 'Disabled; resources released'
        return error is None


class FakeBackend:
    """Explicit software backend; no automatic hardware fallback."""
    def __init__(self, clock):
        self.clock = clock
        self.p = [0., 0., 0.]
        self.calls = []
        self.rights = True
        self.feedback_stamp = None

    def position(self):
        return list(self.p), self.clock() if self.feedback_stamp is None else self.feedback_stamp

    def has_control(self):
        return self.rights

    def send(self, p):
        self.calls.append(('send', list(p)))
        self.p = list(p)

    def stop(self):
        self.calls.append(('stop',))

    def close(self):
        self.calls.append(('close',))
