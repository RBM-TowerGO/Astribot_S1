#!/usr/bin/env python3
"""Explicit offline check: Nav2 map_server only, never navigation/controllers."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
from PIL import Image
import yaml


def main():
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='125' and os.environ.get('ROS_LOCALHOST_ONLY')=='1', 'Use isolated Domain 125 / localhost'
    d=Path(args.directory).resolve();meta=json.loads((d/'slice.json').read_text())
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid
    rclpy.init();node=rclpy.create_node('slice_map_validation_client');received=[]
    sub=node.create_subscription(OccupancyGrid,'/slice_validation_map',received.append,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    log=(d.parent/'nav2_load.log').open('w')
    proc=subprocess.Popen(['/opt/ros/humble/lib/nav2_map_server/map_server','--ros-args',
            '-r','__node:=slice_map_validation','-p','yaml_filename:='+str(d/'map.yaml'),
            '-p','frame_id:='+meta['frame'],'-r','map:=/slice_validation_map'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        client=node.create_client(ChangeState,'/slice_map_validation/change_state')
        assert client.wait_for_service(timeout_sec=15),'Nav2 lifecycle service missing'
        for transition in (1,3):
            req=ChangeState.Request();req.transition.id=transition;future=client.call_async(req)
            rclpy.spin_until_future_complete(node,future,timeout_sec=15)
            assert future.done() and future.result().success,'Nav2 transition failed'
        deadline=time.monotonic()+10
        while not received and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        assert received,'No OccupancyGrid'
        m=received[-1];image=np.array(Image.open(d/'map.pgm'))[::-1]
        expected=np.full(image.shape,-1,np.int8);expected[image==0]=100;expected[image==254]=0
        actual=np.array(m.data).reshape(m.info.height,m.info.width)
        np.testing.assert_array_equal(actual,expected)
        cfg=yaml.safe_load((d/'map.yaml').read_text())
        np.testing.assert_allclose([m.info.origin.position.x,m.info.origin.position.y],cfg['origin'][:2])
        assert m.header.frame_id==meta['frame']
        assert abs(m.info.resolution-.05)<1e-6
        result=dict(passed=True,width=m.info.width,height=m.info.height,frame=m.header.frame_id,
                    occupied=int((actual==100).sum()),free=int((actual==0).sum()),unknown=int((actual==-1).sum()))
        (d.parent/'nav2_load_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
        log.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
