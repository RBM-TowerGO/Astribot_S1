"""SDK-only hold-to-run poses. Importing this file never connects to hardware."""
import json
import math
import threading
import time
from pathlib import Path

HOLD = .6
LEASE = .2
QUIET = .3
STOP_TIMEOUT = 2.
NAMES = ['astribot_torso', 'astribot_arm_left', 'astribot_gripper_left',
         'astribot_arm_right', 'astribot_gripper_right', 'astribot_head']
DOFS = [4, 7, 1, 7, 1, 2]


def load_pack(path):
    """No default pack angles: require a sourced, ordered upper-body route."""
    if path is None:
        return None
    data = json.loads(Path(path).read_text())
    if not isinstance(data, dict) or not isinstance(data.get('source'), str) or not data['source'].strip():
        raise ValueError('打包姿态配置必须注明 source 来源')
    steps = data.get('steps')
    if not isinstance(steps, list) or not 1 <= len(steps) <= 10:
        raise ValueError('打包姿态需要 1–10 个有序 steps')
    total = 0.
    for step in steps:
        if not isinstance(step, dict) or set(step) != {'duration', 'joints'}:
            raise ValueError('每步只允许 duration 和 joints')
        duration, joints = step['duration'], step['joints']
        if type(duration) not in (int, float) or not math.isfinite(duration) or not 3 <= duration <= 30:
            raise ValueError('每步 duration 必须在 3–30 秒之间')
        total += duration
        if not isinstance(joints, dict) or set(joints) != set(NAMES):
            raise ValueError('必须指定全部六组上身关节，不允许底盘目标')
        for name, count in zip(NAMES, DOFS):
            row = joints[name]
            if not isinstance(row, list) or len(row) != count or any(
                    type(v) not in (int, float) or not math.isfinite(v) for v in row):
                raise ValueError('关节维数或数值无效: ' + name)
    if total > 120:
        raise ValueError('打包轨迹总时长不得超过 120 秒')
    return steps


class PoseControl:
    """One worker, no queue. A stopped/finished action requires an input gap.

    The main loop and watchdog only stop; only the worker at entry or a later
    explicit driving request may restart. Never restart while a worker is alive.
    Native SDK calls that hold the GIL can still delay Python cancellation.
    """
    def __init__(self, backend, clock=time.monotonic):
        self.backend, self.clock = backend, clock
        self.worker = None
        self.cancel = threading.Event()
        self.done = threading.Event()
        self.end_watch = threading.Event()
        self.lock = threading.Lock()
        self.deadline = 0.
        self.started = 0.
        self.stopping = None
        self.error = None
        self.blocked = False
        self.needs_resume = False
        self.active = None
        self.message = ''
        self.watch = threading.Thread(target=self._watch, daemon=True)
        self.watch.start()

    def _watch(self):
        while not self.end_watch.wait(.02):
            if self.active and not self.cancel.is_set() and self.clock() >= self.deadline:
                try:
                    self.stop('姿态输入超时：已请求停止')
                except Exception as exc:
                    self.error = exc

    def stop(self, message):
        with self.lock:
            if self.active is None or self.cancel.is_set():
                return
            self.cancel.set()
            self.stopping = self.clock()
            self.message = message
            # Set cancellation before the SDK call; even a stop failure latches.
            if self.backend.has_control():
                self.backend.stop_pose()

    def start(self, name, deadline):
        if self.active or self.blocked or (self.worker and self.worker.is_alive()):
            raise RuntimeError('姿态任务尚未结束')
        self.deadline, self.started = deadline, self.clock()
        self.cancel.clear()
        self.done.clear()
        self.error, self.stopping = None, None
        self.active, self.blocked, self.needs_resume = name, True, True
        self.message = '初始姿态执行中' if name == 'home' else '打包姿态执行中'

        def move():
            try:
                if self.cancel.is_set() or self.clock() >= self.deadline:
                    return
                self.backend.prepare_pose()
                # A stop may race preparation. Do not enqueue a late movement.
                if not self.cancel.is_set() and self.clock() < self.deadline:
                    self.backend.execute_pose(name, self.cancel)
            except Exception as exc:
                self.error = exc
            finally:
                # Also stop after completion, or cancellation during preparation.
                try:
                    if self.backend.has_control():
                        self.backend.stop_pose()
                except Exception as exc:
                    self.error = exc
                self.done.set()
        self.worker = threading.Thread(target=move, daemon=True)
        self.worker.start()

    def tick(self, requested, selected, deadline, last_input, now):
        if self.error:
            raise RuntimeError('姿态控制失败: %s' % self.error)
        if self.active:
            if selected != self.active or now >= deadline:
                self.stop('姿态停止：等待松键后重新输入')
            elif not self.cancel.is_set():
                self.deadline = deadline
            if now - self.started > 150:
                self.stop('姿态执行超时')
                raise RuntimeError('姿态执行超过 150 秒')
            if self.done.is_set() and not self.worker.is_alive():
                self.active = None
                self.message = '姿态任务已结束；松键后重新输入'
            elif self.stopping is not None and now - self.stopping > STOP_TIMEOUT:
                raise RuntimeError('姿态任务停止后 2 秒未退出，禁止恢复底盘')
            return True
        if self.blocked:
            if now - last_input >= QUIET:
                self.blocked = False
            return True
        if selected:
            if not self.backend.pose_available(selected):
                self.message = '打包姿态未配置：P 不执行动作'
            elif requested:
                self.start(requested, deadline)
            else:
                self.message = '持续长按 H/P 确认姿态（0.6 秒连续重复）'
            return True
        return False

    def resume(self):
        if self.active or self.blocked or (self.worker and self.worker.is_alive()):
            raise RuntimeError('姿态尚未结束，拒绝恢复')
        if self.needs_resume:
            self.backend.prepare_pose()
            self.needs_resume = False

    def close(self):
        try:
            self.stop('退出：请求停止姿态')
            if self.worker:
                self.worker.join(STOP_TIMEOUT)
                if self.worker.is_alive():
                    raise RuntimeError('姿态线程未退出；保持停止并释放控制权')
            if self.error:
                raise RuntimeError('姿态控制失败: %s' % self.error)
        finally:
            self.end_watch.set()
            self.watch.join(.2)
