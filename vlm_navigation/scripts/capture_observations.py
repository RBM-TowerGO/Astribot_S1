"""Read-only ROS observation capture. No services, publishers or robot SDK."""
import argparse
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from astribot_camera.msg import CameraImage
from tf2_msgs.msg import TFMessage

ROOT=Path(__file__).resolve().parents[1]
CAMERAS=['head_rgbd','torso_rgbd','head_stereo','left_wrist_rgbd','right_wrist_rgbd']

def stamp(header):
    return header.stamp.sec*1000000000+header.stamp.nanosec


class Capture(Node):
    def __init__(self,out,interval,max_saved):
        super().__init__('towergo_vlm_observation_audit')
        self.out,self.interval,self.max_saved=out,interval,max_saved
        self.rows=defaultdict(list);self.metadata={};self.saved=[];self.transforms={};self.last_saved={}
        self.subs=[]
        q=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
        for camera in CAMERAS:
            for kind in (['color'] if camera=='head_stereo' else ['color','depth']):
                topic=f'/astribot_camera/{camera}/{kind}_compress'+('/compressed' if kind=='color' else '')
                key=f'{camera}/{kind}'
                self.subs.append(self.create_subscription(CompressedImage,topic,lambda m,k=key,t=topic:self.frame(m,k,t),q))
                meta_topic=f'/astribot_camera/{camera}/{kind}_image'
                self.subs.append(self.create_subscription(CameraImage,meta_topic,lambda m,k=key:self.meta(m,k),q))
        for topic in ['/tf','/tf_static']:
            tq=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL if topic=='/tf_static' else DurabilityPolicy.VOLATILE)
            self.subs.append(self.create_subscription(TFMessage,topic,lambda m,t=topic:self.tf(m,t),tq))

    def frame(self,msg,key,topic):
        now=time.time_ns();mono=time.monotonic()
        self.rows[key].append(dict(stamp_ns=stamp(msg.header),receive_wall_ns=now,receive_monotonic=mono,
                                  frame_id=msg.header.frame_id,format=msg.format,bytes=len(msg.data)))
        count=sum(x['stream']==key for x in self.saved)
        if count>=self.max_saved or mono-self.last_saved.get(key,0)<self.interval:return
        self.last_saved[key]=mono
        filename=key.replace('/','_')+f'_{count:03d}'+('.jpg' if key.endswith('/color') else '.raw')
        raw=bytes(msg.data);(self.out/filename).write_bytes(raw)
        self.saved.append(dict(stream=key,topic=topic,file=filename,sha256=hashlib.sha256(raw).hexdigest(),
                               **self.rows[key][-1]))

    def meta(self,msg,key):
        im=msg.image
        self.metadata[key]=dict(stamp_ns=stamp(msg.header),frame_id=msg.header.frame_id,
            image_stamp_ns=stamp(im.header),image_frame_id=im.header.frame_id,
            width=im.width,height=im.height,encoding=im.encoding,step=im.step,
            is_bigendian=im.is_bigendian,data_bytes=len(im.data),gpu_shared=msg.use_gpu_share_memory,
            pitch=msg.pitch,size=msg.size,sequence=msg.seq_num)

    def tf(self,msg,topic):
        for t in msg.transforms:
            v=t.transform
            self.transforms[(t.header.frame_id,t.child_frame_id)]=dict(topic=topic,parent=t.header.frame_id,
                child=t.child_frame_id,stamp_ns=stamp(t.header),
                translation=[v.translation.x,v.translation.y,v.translation.z],
                rotation_xyzw=[v.rotation.x,v.rotation.y,v.rotation.z,v.rotation.w])


def main():
    p=argparse.ArgumentParser();p.add_argument('--output-dir');p.add_argument('--seconds',type=float,default=20)
    p.add_argument('--save-interval',type=float,default=2);p.add_argument('--max-saved',type=int,default=5)
    a=p.parse_args()
    if min(a.seconds,a.save_interval,a.max_saved)<=0:p.error('Positive values required')
    out=Path(a.output_dir) if a.output_dir else ROOT/'assets/observations'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    out.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=Capture(out,a.save_interval,a.max_saved);start=time.monotonic()
    try:
        while time.monotonic()-start<a.seconds:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        topics=dict(node.get_topic_names_and_types())
        report=dict(timestamp_utc=datetime.now(timezone.utc).isoformat(),duration_seconds=time.monotonic()-start,
                    mode='subscription only; no services or motion',topics=topics,
                    streams=dict(node.rows),metadata=node.metadata,saved=node.saved,transforms=list(node.transforms.values()))
        (out/'capture.json').write_text(json.dumps(report,indent=2)+'\n')
        for source in ['/etc/config/sensor_calib.json','/etc/config/camera_info.json',
                       '/home/astribot/camera_parameter/astribot_camera_parameter.json',
                       '/home/astribot/camera_parameter/filp_config_for_robot.json']:
            path=Path(source)
            if path.is_file():(out/path.name).write_bytes(path.read_bytes())
        node.destroy_node();rclpy.shutdown()
    print(json.dumps({'directory':str(out),'received':{k:len(v) for k,v in report['streams'].items()},
                      'saved':len(report['saved']),'metadata':report['metadata'],'transforms':len(report['transforms'])},indent=2))


if __name__=='__main__':main()
