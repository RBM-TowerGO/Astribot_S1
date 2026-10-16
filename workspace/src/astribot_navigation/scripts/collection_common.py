"""Collection contracts and asynchronous events. Importing never loads ROS/SDK."""
import json
import queue
import threading
import time
from pathlib import Path

CAMERAS = ('head_rgbd', 'torso_rgbd', 'left_wrist_rgbd', 'right_wrist_rgbd', 'head_stereo')
STREAMS = [(c, k) for c in CAMERAS for k in (('color',) if c == 'head_stereo' else ('color', 'depth'))]
CAMERA_TOPICS = {}
for c, k in STREAMS:
    base = '/astribot_camera/%s/%s' % (c, k)
    CAMERA_TOPICS[base + '_compress' + ('/compressed' if k == 'color' else '')] = 'sensor_msgs/msg/CompressedImage'
    CAMERA_TOPICS[base + '_image'] = 'astribot_camera/msg/CameraImage'
JOINT_NAMES = ('head', 'torso', 'chassis', 'arm_left', 'arm_right', 'gripper_left', 'gripper_right')
JOINT_TOPICS = ['/astribot_%s/joint_space_states' % n for n in JOINT_NAMES]
REQUIRED = list(CAMERA_TOPICS) + JOINT_TOPICS[:3]
OPTIONAL = JOINT_TOPICS[3:] + ['/astribot_whole_body/chassis_imu', '/tf', '/tf_static', '/ft_data']


def source_stamp(msg):
    h = getattr(msg, 'header', None)
    return h.stamp.sec * 1000000000 + h.stamp.nanosec if h else None


def atomic_json(path, value):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2))
    tmp.replace(path)


class EventLog:
    """Bounded queue: control loop never performs file I/O. Errors are latched."""
    def __init__(self, path=None):
        self.path = path
        self.error = None
        self.queue = queue.Queue(maxsize=8192)
        self.file = open(path, 'x') if path else None
        self.thread = None
        if self.file:
            self.thread = threading.Thread(target=self._write, daemon=True)
            self.thread.start()

    def emit(self, event, **data):
        if not self.file or self.error:
            return
        row = dict(event=event, receive_wall_ns=time.time_ns(), monotonic_ns=time.monotonic_ns(), **data)
        try:
            self.queue.put_nowait(row)
        except queue.Full:
            self.error = 'Event queue full; recording incomplete'

    def check(self):
        if self.error:
            raise RuntimeError(self.error)

    def _write(self):
        try:
            flushed=time.monotonic()
            while True:
                try:
                    item = self.queue.get(timeout=.2)
                except queue.Empty:
                    self.file.flush()
                    continue
                if item is None:
                    break
                self.file.write(json.dumps(item, ensure_ascii=False, allow_nan=False) + '\n')
                if time.monotonic()-flushed>=.2:
                    self.file.flush(); flushed=time.monotonic()
            self.file.flush()
        except Exception as exc:
            self.error = 'Event recording failed: ' + str(exc)
        finally:
            self.file.close()

    def close(self):
        if self.thread:
            try:
                self.queue.put(None, timeout=2)
            except queue.Full:
                self.error = 'Event recorder did not drain'
            self.thread.join(timeout=3)
            if self.thread.is_alive():
                self.error = 'Event recorder did not close'
        self.check()


class LoggedBackend:
    """Record actual SDK calls; logging failures cannot interrupt target holding."""
    def __init__(self, backend, log):
        self.backend, self.log = backend, log
        self.control = None

    def __getattr__(self, name):
        return getattr(self.backend, name)

    def has_control(self):
        current = self.backend.has_control()
        if current != self.control:
            self.log.emit('control_rights', owned=current)
            self.control = current
        return current

    def send(self, target):
        self.backend.send(target)
        self.log.emit('chassis_target_sent', target=list(target), frame='SDK202_LOCAL', control_way='filter')

    def send_torso(self, target):
        self.backend.send_torso(target)
        self.log.emit('torso_target_sent', joints=list(target), control_way='direct')

    def execute_pose(self, name, cancel):
        self.log.emit('pose_start', pose=name)
        try:
            return self.backend.execute_pose(name, cancel)
        finally:
            self.log.emit('pose_returned', pose=name, cancelled=cancel.is_set())

    def prepare_pose(self):
        self.backend.prepare_pose()
        self.log.emit('pose_prepare_returned')

    def stop_pose(self):
        self.backend.stop_pose()
        self.log.emit('pose_stop_returned')

    def close(self):
        try:
            self.backend.close()
        except BaseException as exc:
            self.log.emit('control_release_failed', error=str(exc))
            raise
        else:
            self.log.emit('control_released')
