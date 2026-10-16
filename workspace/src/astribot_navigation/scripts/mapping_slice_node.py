#!/usr/bin/env python3
"""Separate occupancy worker; no SDK and no control loop work."""
from array import array
import argparse
import json
from pathlib import Path
import signal
import time
from mapping_slice import Engine
from mapping_support import RAW_TOPICS, atomic_json


def main():
    parser=argparse.ArgumentParser();parser.add_argument('session');args=parser.parse_args()
    run=Path(args.session);session=json.loads((run/'session.json').read_text())
    config=json.loads((run/'slice_config.json').read_text());engine=Engine(config)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from sensor_msgs.msg import PointCloud2,Imu
    from nav_msgs.msg import Odometry,OccupancyGrid
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from scipy.spatial.transform import Rotation
    import numpy as np
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('slice_'+session['run_id'])
    stop=False
    def halt(*unused):
        nonlocal stop
        stop=True
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,halt)
    q=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE if session['replay'] else ReliabilityPolicy.BEST_EFFORT)
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub=node.create_publisher(OccupancyGrid,session['topics']['occupancy'],latched)
    tfpub=node.create_publisher(TFMessage,session['topics']['tf_static'],latched)
    for sensor in (0,1):
        node.create_subscription(PointCloud2,RAW_TOPICS[sensor*2],lambda m,s=sensor:engine.cloud(m,s),q)
    node.create_subscription(Imu,RAW_TOPICS[1],engine.imu,q)
    node.create_subscription(Odometry,session['topics']['odom'],engine.pose,q)
    last=0.;tf_sent=False
    def publish():
        nonlocal tf_sent
        a=engine.grid.raster()
        snapshot=engine.snapshot(a);snapshot['worker_updated']=time.time()
        atomic_json(run/'slice_live.json',snapshot)
        if engine.level is not None and not tf_sent:
            tf=TransformStamped();tf.header.frame_id=engine.poses.frame;tf.child_frame_id=config['frame']
            tf.header.stamp.sec=engine.poses.first[0]//1000000000
            tf.header.stamp.nanosec=engine.poses.first[0]%1000000000
            p=engine.anchor;q=Rotation.from_matrix(engine.level.T).as_quat()
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,p)
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,q)
            tfpub.publish(TFMessage(transforms=[tf]));tf_sent=True
        if not engine.grid.score.size:return
        msg=OccupancyGrid();msg.header.frame_id=config['frame']
        msg.header.stamp.sec=engine.latest_ns//1000000000;msg.header.stamp.nanosec=engine.latest_ns%1000000000
        msg.info.width=a.shape[1];msg.info.height=a.shape[0];msg.info.resolution=engine.grid.r
        msg.info.origin.position.x,msg.info.origin.position.y=map(float,engine.grid.origin*engine.grid.r)
        msg.info.origin.orientation.w=1.;msg.data=array('b',a.tobytes());pub.publish(msg)
    try:
        while not stop:
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()-last>=1:
                publish();last=time.monotonic()
        engine.drain(final=True);publish()
    except Exception as exc:
        atomic_json(run/'slice_live.json',{'error':str(exc),'worker_updated':time.time()})
        raise
    finally:
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
