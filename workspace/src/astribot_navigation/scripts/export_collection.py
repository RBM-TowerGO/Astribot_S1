#!/usr/bin/env python3
"""Offline images and timestamp associations. Never creates ROS nodes or SDK."""
import argparse
import bisect
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
import time
from collection_common import CAMERAS, JOINT_TOPICS, source_stamp, atomic_json


def messages(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={x.name:x.type for x in reader.get_all_topics_and_types()}
    classes={}
    while reader.has_next():
        topic,data,receive=reader.read_next()
        if topic not in classes: classes[topic]=get_message(types[topic])
        yield topic,deserialize_message(data,classes[topic]),receive


def clock_stamp(msg, receive):
    source=source_stamp(msg)
    return (source, 'header') if source and source>0 else (receive, 'receive')


def metadata(msg):
    image=msg.image
    return dict(width=image.width,height=image.height,encoding=image.encoding,
                is_bigendian=image.is_bigendian,step=image.step,sequence=msg.seq_num,
                frame_id=msg.header.frame_id)


def export(run, out, interval=1., tolerance=.15):
    from PIL import Image
    run,out=Path(run),Path(out)
    out.mkdir(parents=True,exist_ok=False)
    frames=[]; errors=[]; last={}; best={}
    source=json.loads((run/'session.json').read_text()) if (run/'session.json').exists() else {'status':'unknown'}
    summary=dict(source_session=str(run),source_status=source.get('status'),status='incomplete',
                 interval_s=interval,tolerance_s=tolerance,depth_units='raw uint16; metric scale/alignment unverified',
                 association='Nearest timestamp within tolerance; software association, not hardware synchronization',
                 pose_frame='LIO/IMU frame from message; not chassis pose',errors=errors)
    try:
        for filename in ('task.json','annotations.jsonl','teleop.jsonl','session.json','collection_report.json','slice_config.json','sensor_calib.json','collection_config.json'):
            if (run/filename).is_file(): shutil.copyfile(run/filename,out/filename)
        if (run/'snapshots').is_dir(): shutil.copytree(run/'snapshots',out/'snapshots')
        # Pass 1: keep compressed RGB as recorded, sample by recorder receipt time.
        for topic,msg,receive in messages(run/'sensor_bag'):
            if not topic.endswith('/color_compress/compressed'): continue
            camera=topic.split('/')[2]
            if camera not in CAMERAS or receive-last.get(camera,-10**30)<interval*1e9: continue
            last[camera]=receive
            stamp,basis=clock_stamp(msg,receive)
            folder=out/camera; folder.mkdir(exist_ok=True)
            path=folder/('%019d_%06d.jpg'%(receive,len(frames)))
            path.write_bytes(bytes(msg.data))
            with Image.open(path) as im:
                im.load(); size=list(im.size)
            frames.append(dict(camera=camera,image=str(path.relative_to(out)),image_size=size,source_stamp_ns=source_stamp(msg),
                               receive_wall_ns=receive,association_stamp_ns=stamp,time_basis=basis,context={}))
        frames.sort(key=lambda f:f['association_stamp_ns'])
        stamps=[f['association_stamp_ns'] for f in frames]
        window=int(tolerance*1e9)
        receipt_order=sorted(range(len(frames)),key=lambda i:frames[i]['receive_wall_ns'])
        receipt_stamps=[frames[i]['receive_wall_ns'] for i in receipt_order]

        def offer(topic,msg,receive,value,kind='context',camera=None):
            stamp,basis=clock_stamp(msg,receive)
            timeline=stamps if basis=='header' else receipt_stamps
            for j in range(bisect.bisect_left(timeline,stamp-window),bisect.bisect_right(timeline,stamp+window)):
                i=j if basis=='header' else receipt_order[j]
                frame=frames[i]
                if camera and camera!=frame['camera']: continue
                if basis=='header' and frame['time_basis']!='header': continue
                delta=stamp-timeline[j]; key=(i,topic)
                if key in best and abs(delta)>=best[key]: continue
                best[key]=abs(delta)
                match=dict(topic=topic,delta_ns=delta,source_stamp_ns=source_stamp(msg),receive_wall_ns=receive,time_basis=basis)
                if kind=='depth':
                    path=(out/frame['image']).with_suffix('.depth.raw')
                    path.write_bytes(bytes(msg.data))
                    match.update(file=str(path.relative_to(out)),bytes=len(msg.data),format=msg.format)
                    frame['depth']=match
                else:
                    match['value']=value
                    frame['context'][topic]=match
        # Pass 2: associate original depth and state, without storing all video in RAM.
        for topic,msg,receive in messages(run/'sensor_bag'):
            if topic.endswith('/depth_compress'):
                offer(topic,msg,receive,None,'depth',topic.split('/')[2])
            elif topic.startswith('/astribot_camera/') and topic.endswith('_image'):
                offer(topic,msg,receive,metadata(msg),camera=topic.split('/')[2])
            elif topic in JOINT_TOPICS:
                from rosidl_runtime_py.convert import message_to_ordereddict
                offer(topic,msg,receive,message_to_ordereddict(msg))
            elif topic=='/astribot_whole_body/chassis_imu':
                offer(topic,msg,receive,dict(data=list(msg.data),units='unverified',layout='unverified'))
        if (run/'lio_bag'/'metadata.yaml').exists():
            for topic,msg,receive in messages(run/'lio_bag'):
                if not topic.endswith('/odometry'): continue
                p,q=msg.pose.pose.position,msg.pose.pose.orientation
                offer(topic,msg,receive,dict(position=[p.x,p.y,p.z],quaternion_xyzw=[q.x,q.y,q.z,q.w],
                                             frame_id=msg.header.frame_id,child_frame_id=msg.child_frame_id))
        initial=json.loads((run/'task.json').read_text()).get('description','') if (run/'task.json').exists() else ''
        annotations=[]
        if (run/'annotations.jsonl').exists():
            annotations=[json.loads(line) for line in (run/'annotations.jsonl').open()]
        tasks=sorted([r for r in annotations if r['event']=='task'],key=lambda r:r['receive_wall_ns'])
        task_stamps=[r['receive_wall_ns'] for r in tasks]
        for frame in frames:
            task_i=bisect.bisect_right(task_stamps,frame['receive_wall_ns'])-1
            frame['task']=tasks[task_i]['text'] if task_i>=0 else initial
            frame['event_log']='teleop.jsonl' if (out/'teleop.jsonl').exists() else None
            camera=frame['camera']; context=frame['context']
            missing=[t for t in JOINT_TOPICS[:3] if t not in context]
            if not any(t.endswith('/odometry') for t in context): missing.append('LIO odometry')
            rgbmeta='/astribot_camera/%s/color_image'%camera
            if rgbmeta not in context: missing.append(rgbmeta)
            if camera!='head_stereo':
                metatopic='/astribot_camera/%s/depth_image'%camera
                depth=frame.get('depth'); meta=context.get(metatopic)
                if not depth: missing.append('depth')
                if not meta: missing.append(metatopic)
                if depth and meta:
                    m=meta['value']; depth['metadata_delta_ns']=meta['delta_ns']-depth['delta_ns']
                    depth['metadata']=m
                    raw=out/depth['file']
                    # Only produce uint16 PNG where encoding and byte layout are known.
                    if (m['encoding'].lower() in ('16uc1','mono16') and m['width']>0 and m['height']>0
                        and depth['bytes']==m['width']*m['height']*2 and abs(depth['metadata_delta_ns'])<=window):
                        import numpy as np
                        a=np.frombuffer(raw.read_bytes(),dtype='>u2' if m['is_bigendian'] else '<u2').reshape(m['height'],m['width'])
                        png=raw.with_suffix('.png'); Image.fromarray(a.astype('uint16')).save(png)
                        depth['uint16_png']=str(png.relative_to(out))
                    else: depth['decode_note']='Retained raw: encoding/layout or metadata association unverified'
            frame['missing']=missing
        with (out/'frames.jsonl').open('w') as f:
            for frame in frames: f.write(json.dumps(frame,ensure_ascii=False,allow_nan=False)+'\n')
        summary.update(status='exported' if frames else 'incomplete',frames=len(frames),
                       frames_with_missing_context=sum(bool(f['missing']) for f in frames),
                       cameras={c:sum(f['camera']==c for f in frames) for c in CAMERAS})
        if not frames: errors.append('No decodable RGB frames')
    except BaseException as exc:
        errors.append(str(exc)); raise
    finally:
        atomic_json(out/'export_report.json',summary)
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory'); p.add_argument('--output-directory'); p.add_argument('--interval',type=float,default=1.)
    p.add_argument('--max-time-difference',type=float,default=.15)
    a=p.parse_args()
    if not all(math.isfinite(v) and v>0 for v in (a.interval,a.max_time_difference)): p.error('Positive finite timing required')
    run=Path(a.directory).resolve()
    out=Path(a.output_directory) if a.output_directory else run/('vlm_export_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    result=export(run,out,a.interval,a.max_time_difference)
    print(json.dumps(dict(directory=str(out),**result),ensure_ascii=False))
    return 0 if result['status']=='exported' else 2

if __name__=='__main__': raise SystemExit(main())
