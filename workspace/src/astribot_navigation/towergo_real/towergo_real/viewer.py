"""Read-only localhost HTTP viewer; no SDK, services, or motion publishers."""
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
import threading
import time
import numpy as np
from PIL import Image


class ViewState:
    def __init__(self):
        self.lock = threading.Lock()
        self.value = {'map': None, 'pose': None, 'trajectory': [], 'control': None,
                      'diagnostics': {}, 'mapping_fault': False}
        self.receipts = {}

    def update(self, key, value):
        with self.lock:
            self.value[key] = value
            self.receipts[key] = time.monotonic()

    def snapshot(self):
        with self.lock:
            return dict(self.value, ages={k: time.monotonic()-t for k, t in self.receipts.items()})


def handler_for(state, html):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                body, kind = html, 'text/html; charset=utf-8'
            elif self.path == '/api/state':
                body, kind = json.dumps(state.snapshot(), allow_nan=False).encode(), 'application/json'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', kind)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            self.send_error(405, 'Read-only viewer')

        def log_message(self, fmt, *args):
            pass
    return Handler


def encode_map(msg):
    width, height = msg.info.width, msg.info.height
    if width <= 0 or height <= 0 or width*height != len(msg.data) or width*height > 16_000_000:
        raise ValueError('Invalid/oversized occupancy grid')
    values = np.asarray(msg.data, dtype=np.int16).reshape(height, width)
    pixels = np.full((height, width), 175, np.uint8)
    known = values >= 0
    pixels[known] = np.round(255*(1-np.clip(values[known], 0, 100)/100)).astype(np.uint8)
    data = io.BytesIO()
    Image.fromarray(np.flipud(pixels)).save(data, format='PNG')
    p, q = msg.info.origin.position, msg.info.origin.orientation
    return {'width': width, 'height': height, 'resolution': msg.info.resolution,
            'origin': [p.x, p.y, 2*np.arctan2(q.z, q.w)],
            'png': base64.b64encode(data.getvalue()).decode(), 'frame': msg.header.frame_id}


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.time import Time
    from nav_msgs.msg import OccupancyGrid, Path as RosPath
    from std_msgs.msg import String, Bool
    from diagnostic_msgs.msg import DiagnosticArray
    from tf2_ros import Buffer, TransformListener
    from ament_index_python.packages import get_package_share_directory
    from .geometry import transform, apply
    from scipy.spatial.transform import Rotation

    rclpy.init(args=args)
    state = ViewState()
    class Viewer(Node):
        def __init__(self):
            super().__init__('towergo_map_viewer')
            self.port = self.declare_parameter('port', 8765).value
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.trajectory = np.empty((0,3))
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(OccupancyGrid, '/map', self.map, latched)
            self.create_subscription(RosPath, '/towergo/trajectory', self.path, 1)
            self.create_subscription(String, '/towergo/control_status', self.control, 1)
            self.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostics, 10)
            self.create_subscription(Bool, '/towergo/mapping_fault', lambda m: state.update('mapping_fault', m.data), latched)
            self.create_timer(1., self.pose)

        def map(self, msg):
            try:
                state.update('map', encode_map(msg))
            except ValueError as exc:
                self.get_logger().warning(str(exc))

        def path(self, msg):
            if msg.header.frame_id == 'odom':
                self.trajectory = np.array([[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses])

        def control(self, msg):
            try:
                state.update('control', json.loads(msg.data))
            except (ValueError, TypeError):
                pass

        def diagnostics(self, msg):
            with state.lock:
                for s in msg.status:
                    if s.name.startswith('towergo/'):
                        level = int.from_bytes(s.level, 'little') if isinstance(s.level, bytes) else int(s.level)
                        state.value['diagnostics'][s.name] = {'message': s.message, 'level': level}
                state.receipts['diagnostics'] = time.monotonic()

        def matrix(self, parent, child):
            msg = self.buffer.lookup_transform(parent, child, Time())
            p,q = msg.transform.translation,msg.transform.rotation
            return transform([p.x,p.y,p.z], [q.x,q.y,q.z,q.w]), msg.header.stamp

        def pose(self):
            try:
                t, stamp = self.matrix('map', 'base_link')
                age = (self.get_clock().now().nanoseconds-(stamp.sec*10**9+stamp.nanosec))*1e-9
                if not 0 <= age <= 2.:
                    state.update('pose', None)
                    return
                state.update('pose', [float(t[0,3]),float(t[1,3]),float(Rotation.from_matrix(t[:3,:3]).as_euler('xyz')[2])])
                if len(self.trajectory):
                    map_odom, _ = self.matrix('map', 'odom')
                    state.update('trajectory', apply(map_odom, self.trajectory)[:,:2].tolist())
            except Exception:
                state.update('pose', None)

    node = Viewer()
    html = (Path(get_package_share_directory('towergo_real'))/'web/index.html').read_bytes()
    server = ThreadingHTTPServer(('127.0.0.1', node.port), handler_for(state, html))
    worker = threading.Thread(target=server.serve_forever, daemon=True); worker.start()
    node.get_logger().info(f'Read-only map viewer: http://127.0.0.1:{node.port}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown(); server.server_close(); worker.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
