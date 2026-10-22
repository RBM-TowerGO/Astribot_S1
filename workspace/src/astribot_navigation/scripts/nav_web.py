#!/usr/bin/env python3
"""Local browser control for recorded-map SIMULATION only; no hardware imports."""
import argparse
import json
import math
import os
from pathlib import Path
import queue
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Log
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from astribot_nav_sim.grid_world import GridWorld


def pose_values(p):
    q = p.orientation
    return [p.position.x, p.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]


class WebNavigation(Node):
    def __init__(self, filename):
        super().__init__('towergo_sim_web', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.grid = GridWorld(filename)
        self.lock = threading.RLock(); self.commands = queue.Queue(maxsize=16)
        self.token = secrets.token_urlsafe(24)
        self.state = dict(mode='simulation', status='请选择起点', pose=None, truth=None,
                          trajectory=[], truth_trajectory=[], plan=[], local_plan=[], start=None,
                          goal=None, velocity=[0., 0.], command_velocity=[0.,0.], collisions=0, distance_remaining=None,
                          max_linear=.1, max_angular=.1, ready=False, initialized=False,
                          error='', diagnostics=[], generation=0)
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')
        self.lifecycle_future = None; self.lifecycle_check_at = 0.
        self.nav_active = False; self.lifecycle_reply_at = 0.
        self.goal_handle = None; self.busy = False; self.generation = 0
        self.last_truth = 0.; self.last_amcl = 0.; self.init_at = 0.
        self.init_pending = None; self.stop_pending = False; self.result_future = None
        self.initial_retry = None; self.initial_publish_at = 0.
        self.initial_stamp = 0.; self.amcl_pose = None
        self.reachable = self.grid.reachable_png([float('nan'), 0., 0.])
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.reset = self.create_publisher(PoseWithCovarianceStamped, '/sim/reset_pose', 1)
        self.gate = self.create_publisher(Bool, '/sim/navigation_enabled', 1)
        self.create_subscription(Odometry, '/ground_truth/odom', self.truth, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl, 10)
        self.create_subscription(NavPath, '/plan', lambda m: self.path(m, 'plan'), 1)
        self.create_subscription(NavPath, '/local_plan', lambda m: self.path(m, 'local_plan'), 1)
        self.create_subscription(NavPath, '/lookahead_collision_arc', lambda m: self.path(m, 'local_plan'), 1)
        self.create_subscription(Twist, '/cmd_vel', self.velocity, 10)
        self.create_subscription(Bool, '/ground_truth/collision', self.collision, 10)
        self.create_subscription(Log, '/rosout', self.diagnostic, 100)
        self.create_timer(.1, self.tick)

    def truth(self, msg):
        with self.lock:
            self.last_truth = time.monotonic(); self.state['truth'] = pose_values(msg.pose.pose)
            self.state['velocity'] = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]
            if self.state['initialized']:
                self.append('truth_trajectory', self.state['truth'])

    def amcl(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        # AMCL stamps its pose with the scan time, which can precede the
        # initialization request slightly. Otherwise a stationary robot can
        # wait forever because AMCL only republishes after sufficient motion.
        if stamp >= self.initial_stamp-.5:
            self.last_amcl = time.monotonic(); self.amcl_pose = pose_values(msg.pose.pose)

    def event(self, message):
        self.get_logger().info(message)

    def lifecycle_reply(self, future):
        with self.lock:
            try: self.nav_active = future.result().current_state.id == 3
            except Exception: self.nav_active = False
            self.lifecycle_reply_at = time.monotonic()

    def navigation_ready(self, now, tf_age):
        return bool(now-self.last_truth < 1. and -.1 <= tf_age < 1.
            and self.nav_active and now-self.lifecycle_reply_at < 2. and self.action.server_is_ready())

    def diagnostic(self, msg):
        # Humble bindings may expose uint8 constants as bytes and fields as int.
        level = msg.level[0] if isinstance(msg.level, bytes) else int(msg.level)
        if level < 30 or not self.busy: return
        if not any(k in msg.name for k in ('planner', 'controller', 'navigator', 'costmap', 'amcl')): return
        with self.lock:
            text = msg.name + ': ' + msg.msg
            entries = self.state['diagnostics']
            if not entries or entries[-1] != text:
                entries.append(text); del entries[:-8]

    def path(self, msg, name):
        with self.lock:
            points = [pose_values(p.pose)[:2] for p in msg.poses]
            if msg.header.frame_id and msg.header.frame_id != 'map':
                try:
                    t = self.buffer.lookup_transform('map', msg.header.frame_id, Time())
                    q = t.transform.rotation; a = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
                    x, y = t.transform.translation.x, t.transform.translation.y
                    points = [[x+math.cos(a)*p[0]-math.sin(a)*p[1],
                               y+math.sin(a)*p[0]+math.cos(a)*p[1]] for p in points]
                except Exception:
                    self.state[name] = []; return
            self.state[name] = points

    def velocity(self, msg):
        with self.lock: self.state['command_velocity'] = [msg.linear.x, msg.angular.z]

    def collision(self, msg):
        if msg.data:
            with self.lock:
                self.state['collisions'] += 1
                if self.busy and not self.stop_pending:
                    self.stop_pending = True; self.set_gate(False)
                    if self.goal_handle: self.goal_handle.cancel_goal_async()
                    self.state['error'] = '模拟器检测到矩形轮廓进入障碍或未知区，已停止；请检查局部路径与地图边界'
                    self.state['status'] = '地图碰撞阻挡，已停止模拟运动'
                    self.event(self.state['error'])

    def append(self, name, p):
        a = self.state[name]
        if not a or math.hypot(a[-1][0]-p[0], a[-1][1]-p[1]) > .02:
            a.append(p[:2]); del a[:-12000]

    def set_gate(self, enabled):
        self.gate.publish(Bool(data=enabled))

    def execute(self, command, values):
        if command == 'stop':
            self.set_gate(False); self.stop_pending = True
            if self.goal_handle: self.goal_handle.cancel_goal_async()
            self.state['status'] = '已停止模拟运动；正在取消任务' if self.busy else '模拟运动已停止'
            return
        if self.busy:
            raise ValueError('请先停止当前任务并等待取消完成')
        if not self.grid.safe(values):
            raise ValueError('该位置或朝向下，机器人矩形轮廓与障碍/未知区重叠，请移动选点或调整朝向')
        if command == 'start':
            self.set_gate(False); self.stop_pending = False
            self.initial_retry = None
            self.generation += 1; self.init_at = time.monotonic(); self.last_amcl = 0.
            self.state.update(start=values, goal=None, trajectory=[], truth_trajectory=[], plan=[],
                              local_plan=[], initialized=False, pose=None, truth=None, error='',
                              diagnostics=[], distance_remaining=None, generation=self.generation,
                              status='正在放置模拟机器人并初始化 AMCL')
            self.amcl_pose = None; self.initial_stamp = float('inf')
            self.reachable = self.grid.reachable_png(values)
            self.event('设置仿真起点 x=%.2f y=%.2f yaw=%.1f°；重置轨迹' % (values[0], values[1], math.degrees(values[2])))
            msg = PoseWithCovarianceStamped(); msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            p = msg.pose.pose; p.position.x, p.position.y = values[:2]
            p.orientation.z = math.sin(values[2]/2); p.orientation.w = math.cos(values[2]/2)
            msg.pose.covariance[0] = .01; msg.pose.covariance[7] = .01; msg.pose.covariance[35] = .01
            self.reset.publish(msg); self.init_pending = msg
            return
        if command != 'goal': raise ValueError('未知操作')
        if not self.state['initialized'] or not self.state['ready']:
            raise ValueError('请先设置起点，并等待定位和 Nav2 就绪')
        if not self.grid.connected(self.state['truth'], values):
            message = '无法通行：起点与终点之间没有满足模拟机器人宽度的连续已知空闲通道。请在绿色可达区域内选终点；窄道或未知区域会阻断路线。'
            self.state.update(goal=values, error=message, status='目标不可达，未启动运动',
                              distance_remaining=None, plan=[], local_plan=[])
            self.event(message)
            return
        self.stop_pending = False; self.busy = True; self.goal_handle = None
        self.state.update(goal=values, status='正在提交导航目标', plan=[], local_plan=[],
                          error='', diagnostics=[], distance_remaining=None)
        self.event('提交导航目标 x=%.2f y=%.2f yaw=%.1f°' % (values[0], values[1], math.degrees(values[2])))
        goal = NavigateToPose.Goal(); goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = values[:2]
        goal.pose.pose.orientation.z = math.sin(values[2]/2); goal.pose.pose.orientation.w = math.cos(values[2]/2)
        self.set_gate(True)
        try:
            future = self.action.send_goal_async(goal, feedback_callback=self.feedback)
            future.add_done_callback(self.accepted)
        except Exception:
            self.busy = False; self.set_gate(False); raise

    def feedback(self, msg):
        with self.lock: self.state['distance_remaining'] = msg.feedback.distance_remaining

    def accepted(self, future):
        with self.lock:
            try:
                self.goal_handle = future.result()
                if not self.goal_handle.accepted:
                    self.busy = False; self.set_gate(False); self.state['status'] = 'Nav2 拒绝目标'; self.event(self.state['status']); return
                self.state['status'] = '导航中'
                self.event('Nav2 已接受目标，开始规划和运动')
                if self.stop_pending: self.goal_handle.cancel_goal_async()
                self.result_future = self.goal_handle.get_result_async()
                self.result_future.add_done_callback(self.finished)
            except Exception as exc:
                self.busy = False; self.set_gate(False); self.state['status'] = str(exc)

    def finished(self, future):
        with self.lock:
            self.busy = False; self.goal_handle = None; self.set_gate(False)
            try:
                status = future.result().status
                self.state['status'] = {4:'已到达目标', 5:'任务已取消', 6:'导航失败，请检查规划路径和日志'}.get(status, '任务结束：'+str(status))
                if status == 6:
                    logs = self.state['diagnostics']
                    self.state['error'] = logs[-1] if logs else 'Nav2 中止任务；未收到具体错误日志。请检查终端的 planner_server / controller_server 输出。'
                self.event('导航结束 status=%s：%s %s' % (status, self.state['status'], self.state['error']))
            except Exception as exc: self.state['status'] = '任务异常：'+str(exc)

    def tick(self):
        with self.lock:
            now = time.monotonic()
            if now-self.lifecycle_check_at > .5 and self.lifecycle.service_is_ready():
                if self.lifecycle_future is None or self.lifecycle_future.done():
                    self.lifecycle_check_at = now
                    self.lifecycle_future = self.lifecycle.call_async(GetState.Request())
                    self.lifecycle_future.add_done_callback(self.lifecycle_reply)
            try:
                t = self.buffer.lookup_transform('map', 'base_link', Time())
                p = t.transform.translation; q = t.transform.rotation
                self.state['pose'] = [p.x, p.y, math.atan2(2*q.w*q.z, 1-2*q.z*q.z)]
                stamp = t.header.stamp.sec+t.header.stamp.nanosec*1e-9
                tf_age = self.get_clock().now().nanoseconds*1e-9-stamp
                if self.state['initialized']: self.append('trajectory', self.state['pose'])
            except Exception:
                tf_age = float('inf'); self.state['pose'] = None
            self.state['ready'] = self.navigation_ready(now, tf_age)
            if self.init_pending and now-self.init_at > .5:
                # Wait for reset to be visible in truth before setting localization.
                p = self.state['truth']; s = self.state['start']
                if p and math.hypot(p[0]-s[0], p[1]-s[1]) < .02 and abs(math.atan2(math.sin(p[2]-s[2]), math.cos(p[2]-s[2]))) < .02:
                    self.init_pending.header.stamp = self.get_clock().now().to_msg()
                    self.initial_stamp = self.get_clock().now().nanoseconds*1e-9
                    self.initial.publish(self.init_pending)
                    self.initial_retry = self.init_pending; self.initial_publish_at = now
                    self.init_pending = None
                elif now-self.init_at > 5:
                    self.state['status'] = '模拟器未确认起点，请重新设置'; self.init_pending = None
            # Startup can reach this point before AMCL creates its volatile
            # subscription. Retry until a fresh pose acknowledges initialization.
            if self.initial_retry is not None:
                if self.last_amcl > self.init_at:
                    self.initial_retry = None
                elif now-self.init_at > 30:
                    self.initial_retry = None
                    self.state['status'] = 'AMCL 未确认初始化，请检查仿真终端并重新设置起点'
                elif now-self.initial_publish_at > .5:
                    self.initial_retry.header.stamp = self.get_clock().now().to_msg()
                    self.initial_stamp = self.get_clock().now().nanoseconds*1e-9
                    self.initial.publish(self.initial_retry); self.initial_publish_at = now
            if not self.state['initialized'] and self.state['start'] and self.last_amcl > self.init_at and now-self.init_at > 2 and self.state['ready'] and self.amcl_pose:
                s = self.state['start']; a = self.amcl_pose
                if math.hypot(a[0]-s[0], a[1]-s[1]) < .2 and abs(math.atan2(math.sin(a[2]-s[2]), math.cos(a[2]-s[2]))) < .2:
                    self.state['initialized'] = True; self.state['status'] = '起点已设置，请在绿色可达区域中选择终点'
                    self.state['trajectory'] = []; self.state['truth_trajectory'] = []
                    self.event('起点重设和定位已确认；可设置目标')
            if self.busy and not self.state['ready']:
                self.set_gate(False)
                if not self.stop_pending:
                    self.stop_pending = True
                    if self.goal_handle: self.goal_handle.cancel_goal_async()
                    self.state['status'] = '位姿或仿真数据过期，已停止模拟运动'
            try:
                command, values = self.commands.get_nowait()
                self.execute(command, values)
            except queue.Empty: pass
            except Exception as exc:
                self.state['status'] = str(exc); self.state['error'] = str(exc)
            self.state['busy'] = self.busy
            # Simulation motion lease: a dead web process cannot leave motion enabled.
            self.set_gate(self.busy and not self.stop_pending and self.state['ready'])


def handler(node):
    html = Path(__file__).with_name('nav_web.html').read_bytes()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def send(self, status, data, content_type):
            self.send_response(status); self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store'); self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)
        def do_GET(self):
            if self.path == '/': return self.send(200, html.replace(b'__TOKEN__', node.token.encode()), 'text/html; charset=utf-8')
            if self.path == '/map.png': return self.send(200, node.grid.png, 'image/png')
            if self.path.split('?', 1)[0] == '/reachable.png':
                with node.lock: data = node.reachable
                return self.send(200, data, 'image/png')
            if self.path == '/api/state':
                with node.lock:
                    data = json.dumps(dict(node.state, map=node.grid.metadata()), ensure_ascii=False, allow_nan=False).encode()
                return self.send(200, data, 'application/json')
            self.send(404, b'Not found', 'text/plain')
        def do_POST(self):
            if self.headers.get('X-Session-Token') != node.token:
                return self.send(403, b'Forbidden', 'text/plain')
            try:
                if self.path not in ('/api/start', '/api/goal', '/api/stop'): raise ValueError('Unknown endpoint')
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 1024: raise ValueError('Invalid request size')
                obj = json.loads(self.rfile.read(length)); command = self.path.rsplit('/', 1)[-1]
                values = None
                if command != 'stop':
                    values = [float(obj[k]) for k in ('x','y','yaw')]
                    if not node.grid.safe(values): raise ValueError('当前选点或朝向下矩形轮廓空间不足，请挪动位置或调整朝向')
                node.commands.put_nowait((command, values))
                self.send(202, b'{"queued":true}', 'application/json')
            except (ValueError, KeyError, TypeError, queue.Full) as exc:
                self.send(400, json.dumps({'error':str(exc)}, ensure_ascii=False).encode(), 'application/json')
    return Handler


def main():
    p = argparse.ArgumentParser(); p.add_argument('--map', required=True); p.add_argument('--port', type=int, default=8766)
    args = p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '125' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Simulation requires Domain 125 and localhost ROS discovery')
    rclpy.init(args=[]); node = WebNavigation(args.map)
    server = ThreadingHTTPServer(('0.0.0.0', args.port), handler(node))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print('TowerGO 仿真网页监听端口 '+str(args.port), flush=True)
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        if rclpy.ok(): node.set_gate(False)
        server.shutdown(); server.server_close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
