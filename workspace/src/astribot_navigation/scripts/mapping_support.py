"""Bounded map preview, owned processes, health gates and read-only HTTP."""
import json
import gzip
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

RAW_TOPICS = ['/livox/lidar_front', '/livox/imu_front', '/livox/lidar_back', '/livox/imu_back']
REQUIRED_RAW_TOPICS = RAW_TOPICS[:3]
REQUIRED = ('back', 'front', 'imu', 'adapted_cloud', 'adapted_imu', 'odom', 'cloud')


def atomic_json(path, value):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))
    temp.replace(path)


def cloud_xyz(msg):
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields and fields[k].datatype == 7 and fields[k].count == 1 for k in ('x', 'y', 'z')):
        raise ValueError('PointCloud2 requires float32 x/y/z')
    if not msg.width or not msg.height:
        return np.empty((0, 3), np.float32)
    if msg.point_step <= 0 or msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.height*msg.row_step:
        raise ValueError('Truncated point cloud')
    columns = []
    for k in ('x', 'y', 'z'):
        off = fields[k].offset
        if off < 0 or off + 4 > msg.point_step:
            raise ValueError('Invalid field offset')
        a = np.ndarray((msg.height, msg.width), dtype='>f4' if msg.is_bigendian else '<f4',
                       buffer=msg.data, offset=off, strides=(msg.row_step, msg.point_step))
        columns.append(a.reshape(-1))
    pts = np.column_stack(columns)
    return pts[np.isfinite(pts).all(axis=1)]


class Preview:
    def __init__(self, max_points=100000, max_path=10000):
        self.points = np.empty((0, 3), np.float32)
        self.path = []
        self.pose = None
        self.voxel = .1
        self.max_points, self.max_path = max_points, max_path

    def cloud(self, points):
        points = points[np.isfinite(points).all(axis=1)]
        # Bound work per incoming message, as well as retained state.
        if len(points) > self.max_points:
            points = points[::math.ceil(len(points)/self.max_points)]
        merged = np.concatenate((self.points, points))
        while len(merged):
            indices = np.floor(merged/self.voxel).astype(np.int64)
            _, keep = np.unique(indices, axis=0, return_index=True)
            merged = merged[keep]
            if len(merged) <= self.max_points:
                break
            self.voxel *= 2
        self.points = merged.astype(np.float32)

    def odom(self, xyz, quat):
        if not all(math.isfinite(x) for x in (*xyz, *quat)):
            raise ValueError('Non-finite odometry')
        self.pose = {'xyz': list(xyz), 'quaternion_xyzw': list(quat)}
        self.path.append(list(xyz))
        if len(self.path) > self.max_path:
            self.path = self.path[::2] + [self.path[-1]]


class State:
    def __init__(self, run_id):
        self.lock = threading.RLock()
        self.preview = Preview()
        self.receipts, self.stamps, self.counts = {}, {}, {}
        self.started = time.monotonic()
        self.status, self.error = 'initializing', None
        self.run_id, self.free_gib = run_id, None
        self.stable_since = None
        self.frame = None
        self.fusion = {}
        self.fusion_reason = None
        self.validation_warning = None

    def seen(self, key, stamp, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if stamp <= 0 or (key in self.stamps and stamp <= self.stamps[key]):
                self.error = 'Repeated/backwards timestamp: ' + key
                return
            self.stamps[key] = stamp
            self.receipts[key] = now
            self.counts[key] = self.counts.get(key, 0) + 1

    def healthy(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            return not self.error and all(k in self.receipts and now-self.receipts[k] <= 2 for k in REQUIRED)

    def ready(self, now, recording_started):
        if not self.healthy(now):
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
        return now-recording_started >= 10 and now-self.stable_since >= 2

    def snapshot(self):
        with self.lock:
            return {'run_id': self.run_id, 'status': self.status, 'error': self.error,
                    'elapsed': round(time.monotonic()-self.started, 1), 'free_gib': self.free_gib,
                    'ages': {k: round(time.monotonic()-v, 2) for k,v in self.receipts.items()},
                    'counts': dict(self.counts), 'frame': self.frame,
                    'mode': 'dual_lidar', 'fusion': dict(self.fusion),
                    'validation_warning': self.validation_warning, 'fusion_reason': self.fusion_reason,
                    'points': np.round(self.preview.points.astype(np.float64), 3).tolist(),
                    'trajectory': list(self.preview.path), 'pose': self.preview.pose,
                    'voxel': self.preview.voxel}


def web_server(state, port, html):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                data, kind = html, 'text/html; charset=utf-8'
            elif self.path == '/api/state':
                data = json.dumps(state.snapshot(), ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode()
                kind = 'application/json'
            else:
                self.send_error(404); return
            compressed = 'gzip' in self.headers.get('Accept-Encoding', '') and len(data)>1024
            if compressed: data = gzip.compress(data, compresslevel=1, mtime=0)
            self.send_response(200)
            if compressed: self.send_header('Content-Encoding', 'gzip')
            self.send_header('Vary', 'Accept-Encoding')
            self.send_header('Content-Type', kind)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            try: self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError): pass

        def do_POST(self):
            self.send_error(405, 'Read-only')

        def log_message(self, *args):
            pass
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


class OwnedProcess:
    def __init__(self, name, cmd, log=None, tty=False, pass_fds=()):
        self.name, self.group = name, not tty
        self.log = Path(log).open('wb') if log else None
        try:
            self.proc = subprocess.Popen(cmd, stdin=None if tty else subprocess.DEVNULL,
                                         stdout=self.log, stderr=subprocess.STDOUT if self.log else None,
                                         start_new_session=self.group, pass_fds=pass_fds)
        except BaseException:
            if self.log: self.log.close()
            raise

    def poll(self):
        return self.proc.poll()

    def send(self, sig):
        if self.proc.poll() is None:
            try:
                if self.group: os.killpg(self.proc.pid, sig)
                else: self.proc.send_signal(sig)
            except ProcessLookupError: pass

    def stop(self, timeout=20):
        forced = False
        try:
            if self.poll() is None:
                self.send(signal.SIGINT)
                try: self.proc.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    forced = True
                    self.send(signal.SIGTERM)
                    try: self.proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        self.send(signal.SIGKILL); self.proc.wait(timeout=5)
            if forced or self.proc.returncode not in (0, -signal.SIGINT, 130):
                raise RuntimeError('%s unclean exit: %s' % (self.name, self.proc.returncode))
        finally:
            if self.log: self.log.close()


def shutdown_order(processes):
    # Stop driving first, preserve the recorders until mapping has stopped.
    return [k for k in ('teleop', 'player', 'fastlio', 'adapter', 'lio_record', 'raw_record') if k in processes]


def check_metadata(bag, required):
    import yaml
    data = yaml.safe_load((Path(bag)/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    counts = {x['topic_metadata']['name']: x['message_count'] for x in data['topics_with_message_count']}
    if any(counts.get(topic, 0) <= 0 for topic in required):
        raise RuntimeError('Empty/missing recorded topics: %s' % required)
    for f in data['relative_file_paths']:
        if not (Path(bag)/f).is_file() or (Path(bag)/f).stat().st_size == 0:
            raise RuntimeError('Missing MCAP: '+f)
    return counts
