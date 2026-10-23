import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from .contracts import checked_command

class CommandGuard(Node):
    def __init__(self):
        super().__init__('command_guard')
        self.declare_parameter('backend','sim')
        self.declare_parameter('max_linear_speed', .1)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        if not 0 < self.max_linear_speed <= .3:
            raise ValueError('Simulation max_linear_speed must be in (0, 0.3]')
        if self.get_parameter('backend').value!='sim':
            raise RuntimeError('Hardware backend is not implemented')
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.diag=self.create_publisher(DiagnosticArray,'/diagnostics',10)
        self.last_cmd=None; self.last_scan=None; self.wall_cmd=0.; self.wall_scan=0.
        self.target=(0.,0.); self.reason='Waiting for sensors and command'
        self.create_subscription(Twist,'/cmd_vel_nav',self.command,10)
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        self.create_timer(.02,self.tick)
    def command(self,m):
        try:
            self.target=checked_command([m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z], max_v=self.max_linear_speed)
            self.last_cmd=self.get_clock().now().nanoseconds*1e-9
            self.wall_cmd=time.monotonic()
        except ValueError as e:
            self.target=(0.,0.); self.last_cmd=None; self.reason=str(e)
    def scan(self,m):
        self.last_scan=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        self.wall_scan=time.monotonic()
    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9
        healthy=all(t is not None and 0<=now-t<=.5 for t in (self.last_cmd,self.last_scan))
        healthy=healthy and time.monotonic()-self.wall_cmd<.5 and time.monotonic()-self.wall_scan<.5
        out=Twist()
        if healthy: out.linear.x,out.angular.z=self.target
        else: self.reason='Command/scan stale, absent, invalid or clock moved backwards'
        self.pub.publish(out)
        if int(now*50)%25==0:
            msg=DiagnosticArray(); msg.header.stamp=self.get_clock().now().to_msg()
            status=DiagnosticStatus(name='towergo/command_guard',level=bytes([0 if healthy else 1]),message='Ready' if healthy else self.reason)
            msg.status=[status]; self.diag.publish(msg)

def main():
    rclpy.init(); n=CommandGuard()
    try: rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        if rclpy.ok(): n.pub.publish(Twist())
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
