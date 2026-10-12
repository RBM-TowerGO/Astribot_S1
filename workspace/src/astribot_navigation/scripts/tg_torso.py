"""Small torso-height increments; no SDK import or connection at import time."""
import math
import time

HZ = 100.
DEFAULT_SPEED = .02         # m/s; pre-ramped targets, independently of chassis speed
MAX_SPEED = .02              # m/s; independent of chassis speed
MAX_JOINT_STEP = .003        # rad per 100 Hz command; reject IK branch jumps
MAX_TRACKING = .05          # rad between last command and measured joints
MAX_Z_TRACKING = .02        # metres between target and measured torso endpoint
LEAD_JOINT = .025           # pause before reaching the hard tracking limits
LEAD_Z = .01
FOLLOW_TIMEOUT = 2.         # seconds without meaningful convergence
FOLLOW_PROGRESS = .04      # improvement in max(error / lead budget); ignore jitter


class HeightLimit(RuntimeError):
    """A rejected step; may be followed by an input in the opposite direction."""


def vector(values, count, label):
    if len(values) != count:
        raise RuntimeError(label + '维数错误')
    result = [float(v) for v in values]
    if not all(math.isfinite(v) for v in result):
        raise RuntimeError(label + '含无效数值')
    return result


class TorsoHeight:
    def __init__(self, backend, clock=time.monotonic):
        self.backend, self.clock = backend, clock
        self.pose = self.joints = None
        self.wait_since = None
        self.wait_error = None
        self.blocked = False
        self.message = ''

    def reset(self):
        # Do not resend an old target after H/P or another controller changed it.
        self.pose = self.joints = None
        self.wait_since = None
        self.wait_error = None
        self.blocked = False

    def block(self, reason):
        self.blocked = True
        self.message = '腰部已暂停：' + reason.split('；')[0] + ' | 松键后重新按 R/F 可重试'
        print('\n[腰部暂停] ' + reason + '；遥操保持运行，X 退出', flush=True)

    def wait_for_feedback(self, now, current_joints, joint_error, z_error):
        # Track the worst normalized error so improvement of a non-limiting
        # axis cannot hide a stuck joint/height. Never renew on rebound/noise.
        error = max(joint_error / LEAD_JOINT, z_error / LEAD_Z)
        if self.wait_since is None:
            self.wait_since = now
            self.wait_error = error
        elif error <= self.wait_error - FOLLOW_PROGRESS:
            self.wait_since = now
            self.wait_error = error
        details = '关节误差 %.4f rad，高度误差 %.4f m' % (joint_error, z_error)
        if now - self.wait_since >= FOLLOW_TIMEOUT:
            self.block('腰部反馈连续 2 秒无明显收敛；%s；目标=%s；反馈=%s' %
                       (details, self.joints, current_joints))
            return
        self.message = '腰部等待反馈：暂停累加 | ' + details

    def tick(self, velocity, deadline, allowed=lambda: True):
        if not velocity:
            self.reset()
            return
        if not math.isfinite(velocity) or abs(velocity) > MAX_SPEED:
            raise RuntimeError('腰部速度必须有限且不超过 0.02 m/s')
        started = self.clock()
        if started >= deadline or not allowed():
            self.message = '腰部输入已超时：未发送新目标'
            self.reset()
            return
        if not self.backend.has_control():
            raise RuntimeError('腰部控制权丢失')
        self.backend.check()
        if self.blocked:
            return
        current_pose, current_joints = self.backend.torso_state()
        current_pose = vector(current_pose, 7, '躯干位姿')
        current_joints = vector(current_joints, 4, '躯干关节')
        if abs(sum(x*x for x in current_pose[3:]) - 1.) > .01:
            raise RuntimeError('躯干姿态四元数无效')
        if self.pose is None:
            # Continue the controller's last goal, not its lagging measurement.
            # Otherwise direct dispatch could jump on every key press/reversal.
            reference_pose, reference_joints = self.backend.torso_reference()
            self.pose = vector(reference_pose, 7, '躯干目标位姿')
            self.joints = vector(reference_joints, 4, '躯干目标关节')
            if abs(sum(x*x for x in self.pose[3:]) - 1.) > .01:
                raise RuntimeError('躯干目标姿态四元数无效')
        joint_error = max(abs(a-b) for a, b in zip(self.joints, current_joints))
        z_error = abs(self.pose[2] - current_pose[2])
        if joint_error > MAX_TRACKING or z_error > MAX_Z_TRACKING:
            self.block('腰部反馈滞后超过保护限值（0.05 rad / 0.02 m）；'
                       '关节误差 %.4f rad，高度误差 %.4f m；目标=%s；反馈=%s' %
                       (joint_error, z_error, self.joints, current_joints))
            return
        # Test each proposed step against the lead budget. Do not latch waiting
        # until error halves: steady tracking offsets can exceed that threshold.
        # Reverse input is checked against the same proposed-target budget.
        target = list(self.pose)
        target[2] += velocity / HZ
        try:
            result = vector(self.backend.solve_torso(target), 4, '躯干逆解')
            if max(abs(a-b) for a, b in zip(result, self.joints)) > MAX_JOINT_STEP:
                raise HeightLimit('单步关节变化过大；请反向调节')
            # Check the proposed command too: checking only the last command
            # permits overshoot by one IK step.
            next_joint_error = max(abs(a-b) for a, b in zip(result, current_joints))
            next_z_error = abs(target[2] - current_pose[2])
            if (next_joint_error > LEAD_JOINT or next_z_error > LEAD_Z):
                self.hold_for_feedback(started, deadline, allowed, current_joints, joint_error, z_error)
                return
        except HeightLimit as exc:
            self.message = '腰部受限：' + str(exc)
            return
        if self.send_checked(result, started, deadline, allowed):
            self.wait_since = self.wait_error = None
            self.pose, self.joints = target, result
            self.message = ('腰部升高' if velocity > 0 else '腰部降低') + ' | 躯干目标 z=%.3f m' % target[2]

    def hold_for_feedback(self, started, deadline, allowed, current_joints, joint_error, z_error):
        self.wait_for_feedback(started, current_joints, joint_error, z_error)
        if not self.blocked:
            # Refresh only the held goal, never accumulate while feedback lags.
            self.send_checked(self.joints, started, deadline, allowed)

    def send_checked(self, joints, started, deadline, allowed):
        try:
            self.backend.check_torso(joints)
        except HeightLimit as exc:
            self.message = '腰部受限：' + str(exc)
            return False
        # Apply the same checks to both fresh steps and repeated hold commands.
        now = self.clock()
        if now - started > .1:
            raise RuntimeError('腰部解算超过 0.1 秒；未发送新目标')
        if now >= deadline or not allowed():
            self.message = '腰部输入已超时：未发送新目标'
            self.reset()
            return False
        if not self.backend.has_control():
            raise RuntimeError('腰部控制权丢失')
        self.backend.check()
        self.backend.send_torso(list(joints))
        return True
