"""Sensor-only probe of installed SDK methods; NEVER construct Astribot()."""
import argparse
from datetime import datetime,timezone
import hashlib
import inspect
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace,MethodType
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,Imu,CompressedImage
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
from astribot_sdk.core.astribot_api.astribot_client import Astribot
from astribot_sdk.core.astribot_api.astribot_interface import AstribotInterface
ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--activate-lidar',action='store_true',help='Temporarily activate only if no Livox publishers; restore OFF afterward')
    args=parser.parse_args()
    out=ROOT/'reports'/('p2-sdk-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    out.mkdir(parents=True)
    report=dict(probe_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),scope='sensor-only method harness; full SDK constructor NOT executed',
                started_utc=datetime.now(timezone.utc).isoformat(),camera={},other={},errors=[])
    rclpy.init();node=Node('towergo_sensor_sdk_probe');executor=SingleThreadedExecutor();executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    facade=SimpleNamespace(node=node,logger=node.get_logger(),_logger=node.get_logger())
    names=['camera_name_to_topic_name','is_camera_activated','camera_callback','register_image_callback','activate_lidar','deactivate_lidar']
    report['sdk_methods']={}
    for name in names:
        fn=getattr(AstribotInterface,name)
        setattr(facade,name,MethodType(fn,facade))
        source=inspect.getsource(fn)
        report['sdk_methods'][name]=dict(file=inspect.getsourcefile(fn),line=inspect.getsourcelines(fn)[1],sha256=hashlib.sha256(source.encode()).hexdigest())
    subscriptions=[];started_lidar=False
    def save():
        (out/'sdk-probe.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    def frame_probe(camera,kind,decode,timeout=4):
        got=threading.Event();result={}
        def callback(topic,msg,width,height,array):
            if got.is_set():return
            result.update(topic=topic,format=msg.format,width=width,height=height,array_shape=list(array.shape),
                dtype=str(array.dtype),bytes=int(array.nbytes),stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                frame_id=msg.header.frame_id)
            if msg.format.lower()=='jpeg':
                import cv2
                im=array if decode else cv2.imdecode(array,cv2.IMREAD_COLOR)
                result['decoded_shape']=None if im is None else list(im.shape)
                result['valid']=im is not None and im.size>0
            else:result['valid']=array.size>0
            got.set()
        sub=facade.register_image_callback(camera,kind,callback,decode)
        ok=got.wait(timeout)
        if sub is not None:node.destroy_subscription(sub)
        return dict(received=ok,need_decode=decode,**result)
    try:
        time.sleep(1)
        report['camera_status']=Astribot.get_cameras_info(SimpleNamespace(astribot_interface=facade))
        for camera in ['head_rgbd','torso_rgbd','head_stereo','left_wrist_rgbd','right_wrist_rgbd']:
            for kind in (['color'] if camera=='head_stereo' else ['color','depth']):
                key=camera+'/'+kind
                report['camera'][key]=frame_probe(camera,kind,False)
                save();print(key,report['camera'][key].get('received'),flush=True)
        report['camera']['torso_rgbd/color_decoded']=frame_probe('torso_rgbd','color',True)
        report['camera']['head_rgbd/grey_sdk']=frame_probe('head_rgbd','grey',False,2)
        topics=dict(node.get_topic_names_and_types())
        lidar_topics=['/livox/lidar_front','/livox/lidar_back','/livox/imu_front','/livox/imu_back']
        report['lidar_publishers_before']={t:node.count_publishers(t) for t in lidar_topics}
        report['services']=dict(node.get_service_names_and_types())
        if args.activate_lidar and not any(report['lidar_publishers_before'].values()):
            started_lidar=True
            report['lidar_activation_result']=facade.activate_lidar()
            deadline=time.monotonic()+20
            while time.monotonic()<deadline and not all(node.count_publishers(t)>0 for t in lidar_topics):time.sleep(.2)
        else:report['lidar_activation_result']='not_requested_or_already_active'
        topics=dict(node.get_topic_names_and_types())
        report['graph_publishers']={t:node.count_publishers(t) for t in topics if any(k in t for k in ['/astribot_camera/','/livox/','imu','joint_space_states','endpoint_current_states','/ft_data','audio','/tf','odom'])}
        wanted=lidar_topics+['/astribot_whole_body/imu_sensor/astribot_chassis_imu_site',
            '/astribot_whole_body/chassis_imu','/astribot_head/joint_space_states',
            '/astribot_torso/joint_space_states','/astribot_chassis/joint_space_states',
            '/astribot_chassis/endpoint_current_states','/ft_data',
            '/astribot_camera/head_rgbd/grey_compress/compressed']
        def on_data(msg,topic):
            row=report['other'].setdefault(topic,dict(count=0,first_received_monotonic=time.monotonic()))
            row.setdefault('first_received_monotonic',time.monotonic())
            row['count']+=1;row['last_received_monotonic']=time.monotonic()
            if row['count']!=1:return
            row['message_type']=type(msg).__name__
            if isinstance(msg,PointCloud2):
                row.update(width=msg.width,height=msg.height,point_step=msg.point_step,data_bytes=len(msg.data),frame_id=msg.header.frame_id,
                    stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,fields=[dict(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count) for f in msg.fields])
                offsets={f.name:f.offset for f in msg.fields}
                if all(k in offsets for k in ['x','y','z']):
                    endian='>' if msg.is_bigendian else '<'
                    dtype=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,'offsets':[offsets[k] for k in ['x','y','z']],'itemsize':msg.point_step})
                    points=np.frombuffer(msg.data,dtype=dtype,count=msg.width*msg.height)
                    xyz=np.stack([points[k] for k in ['x','y','z']],axis=1)
                    valid=np.isfinite(xyz).all(axis=1);nz=valid&(np.linalg.norm(xyz,axis=1)>0)
                    row['finite_fraction']=float(valid.mean());row['nonzero_points']=int(nz.sum())
                    if nz.any():row['xyz_bounds']=[xyz[nz].min(axis=0).tolist(),xyz[nz].max(axis=0).tolist()]
            elif isinstance(msg,CompressedImage):
                import cv2
                im=cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_UNCHANGED)
                row.update(format=msg.format,bytes=len(msg.data),decoded_shape=None if im is None else list(im.shape))
            else:row['first_message']=message_to_ordereddict(msg)
        q=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic in wanted:
            report['other'][topic]=dict(count=0)
            if topic in lidar_topics and topic not in topics:
                topics[topic]=['sensor_msgs/msg/Imu' if '/imu_' in topic else 'sensor_msgs/msg/PointCloud2']
            if topic not in topics:
                report['other'][topic]=dict(count=0,status='topic_absent');continue
            try:
                subscriptions.append(node.create_subscription(get_message(topics[topic][0]),topic,lambda m,t=topic:on_data(m,t),q))
            except Exception as exc:report['errors'].append(dict(topic=topic,error=str(exc)))
        time.sleep(8)
        for row in report['other'].values():
            if row.get('count',0)>1:row['observed_hz']=(row['count']-1)/(row['last_received_monotonic']-row['first_received_monotonic'])
        save()
    except Exception as exc:
        report['errors'].append(dict(error=type(exc).__name__,detail=str(exc)))
        raise
    finally:
        for sub in subscriptions:node.destroy_subscription(sub)
        if started_lidar:
            report['lidar_restore_off_result']=facade.deactivate_lidar()
            deadline=time.monotonic()+15
            while time.monotonic()<deadline and any(node.count_publishers(t)>0 for t in ['/livox/lidar_front','/livox/lidar_back','/livox/imu_front','/livox/imu_back']):time.sleep(.2)
        report['lidar_publishers_after']={t:node.count_publishers(t) for t in ['/livox/lidar_front','/livox/lidar_back','/livox/imu_front','/livox/imu_back']}
        report['finished_utc']=datetime.now(timezone.utc).isoformat();save()
        executor.shutdown(timeout_sec=2);thread.join(timeout=2);node.destroy_node();rclpy.shutdown()
        print('REPORT_DIR='+str(out),flush=True)


if __name__=='__main__':main()
