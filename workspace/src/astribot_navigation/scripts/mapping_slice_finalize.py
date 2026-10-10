"""Replay recorded raw rays and LIO poses into a navigation slice, without ROS nodes."""
import csv
import datetime
import json
import os
from pathlib import Path
import shutil
import tempfile
import uuid
import numpy as np
import yaml
from mapping_slice import Engine, defaults, stamp_ns, png
from mapping_support import RAW_TOPICS, atomic_json, check_metadata


def configuration(run,offset=0.,thickness=.1):
    run=Path(run);c=defaults(offset,thickness)
    session=json.loads((run/'session.json').read_text())
    c['frame']=session['run_id']+'_slice_map'
    params=yaml.safe_load((run/'lio_params.yaml').read_text())['/**']['ros__parameters']['mapping']
    c['imu_from_front_R']=np.array(params['extrinsic_R']).reshape(3,3).tolist()
    c['imu_from_front_T']=params['extrinsic_T']
    # Only the rear origin is needed; endpoints were transformed by the driver.
    driver=json.loads((run/'MID360_config.json').read_text())
    rear=next(x for x in driver['lidar_configs'] if x['ip']=='192.168.0.13')['extrinsic_parameter']
    c['back_origin_front']=[rear[k]/1000 for k in ('x','y','z')]
    calib=run/'sensor_calib.json'
    if calib.exists():
        data=json.loads(calib.read_text())['lidar']['lidar_front']['extrinsics']['matrix']
        c['base_from_front']=np.array(data).reshape(4,4).tolist()
    c['body_filter_source']='vendor front chassis transform; chassis mesh XY bounds, extruded vertically'
    return c


def messages(bag,topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={x.name:get_message(x.type) for x in reader.get_all_topics_and_types() if x.name in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while reader.has_next():
        topic,data,t=reader.read_next()
        yield topic,deserialize_message(data,types[topic])


def finalize_slice(run, output=None, offset=None, thickness=None, export_pcd=False,cancelled=lambda:False):
    run=Path(run).resolve();session=json.loads((run/'session.json').read_text());topics=session['topics']
    check_metadata(run/'raw_bag',RAW_TOPICS[:3]);check_metadata(run/'lio_bag',[topics['odom']])
    c=json.loads((run/'slice_config.json').read_text()) if (run/'slice_config.json').exists() else configuration(run)
    if offset is not None:c['offset']=defaults(offset,c['thickness'])['offset']
    if thickness is not None:c['thickness']=defaults(c['offset'],thickness)['thickness']
    target=Path(output).resolve() if output else run/('navigation_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    if target.exists():raise RuntimeError('Output directory already exists: '+str(target))
    target.parent.mkdir(parents=True,exist_ok=True)
    engine=Engine(c)
    def check():
        if cancelled():raise InterruptedError('Slice export interrupted; bags retained')
    poses=iter(messages(run/'lio_bag',[topics['odom']]))
    next_pose=next(poses,None)
    with tempfile.TemporaryDirectory(prefix='.slice-',dir=target.parent) as temp:
        work=Path(temp)
        # Trajectory CSV is streamed separately so retained live path stays bounded.
        with (work/'trajectory_world.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw'])
            def consume_pose():
                nonlocal next_pose
                m=next_pose[1];engine.pose(m);p=m.pose.pose.position;q=m.pose.pose.orientation
                writer.writerow([stamp_ns(m),p.x,p.y,p.z,q.x,q.y,q.z,q.w]);next_pose=next(poses,None)
            for topic,m in messages(run/'raw_bag',RAW_TOPICS[:3]):
                check()
                if topic==RAW_TOPICS[1]:engine.imu(m);continue
                # Read enough pose history to bracket the entire raw scan, without extrapolation.
                limit=stamp_ns(m)+200000000
                while next_pose is not None and (not engine.poses.data or engine.poses.data[-1][0]<limit):consume_pose()
                engine.cloud(m,0 if topic==RAW_TOPICS[0] else 1)
            while next_pose is not None:check();consume_pose()
        engine.drain(final=True);snapshot=engine.snapshot()
        if not snapshot['ready']:raise RuntimeError('Slice has no confirmed occupied/free cells; diagnostics: '+json.dumps(engine.stats))
        from scipy.spatial.transform import Rotation
        from PIL import Image,ImageDraw
        path=[]
        with (work/'trajectory_world.csv').open() as f,(work/'trajectory.csv').open('w',newline='') as out:
            reader=csv.DictReader(f);writer=csv.writer(out);writer.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw'])
            count=0
            for row in reader:
                check();v=np.array([float(row[k]) for k in ['x','y','z','qx','qy','qz','qw']])
                p=engine.level@(v[:3]-engine.anchor);q=(Rotation.from_matrix(engine.level)*Rotation.from_quat(v[3:])).as_quat()
                writer.writerow([row['stamp_ns'],*p,*q]);count+=1
                path.append(p[:2].tolist())
                if len(path)>10000:path=path[::2]
        (work/'trajectory_world.csv').unlink()
        a=engine.grid.raster();pixel=np.full(a.shape,205,np.uint8);pixel[a==0]=254;pixel[a==100]=0
        h,w=a.shape
        with (work/'map.pgm').open('wb') as f:f.write(('P5\n%d %d\n255\n'%(w,h)).encode()+pixel[::-1].tobytes())
        origin=(engine.grid.origin*engine.grid.r).tolist()
        (work/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',mode='trinary',resolution=engine.grid.r,
                            origin=[*origin,0.],negate=0,occupied_thresh=.65,free_thresh=.196),sort_keys=False))
        image=Image.fromarray(pixel[::-1]).convert('RGB');draw=ImageDraw.Draw(image)
        line=[((p[0]-origin[0])/engine.grid.r,h-1-(p[1]-origin[1])/engine.grid.r) for p in path]
        if len(line)>1:draw.line(line,fill=(230,90,40),width=2)
        image.save(work/'preview.png')
        snapshot.pop('trajectory');snapshot.pop('pose')
        snapshot['grid'].pop('png_base64')
        report=dict(snapshot, navigation_map=True, poses=count,trajectory_reference='IMU in slice map',
                    source_session_status=session['status'],source_session_error=session.get('error'),
                    output=str(target),export_pcd=export_pcd)
        if export_pcd:
            from mapping_finalize import VoxelStore
            from mapping_support import cloud_xyz
            store=VoxelStore(work/'points.sqlite')
            try:
                for _,m in messages(run/'lio_bag',[topics['cloud']]):check();store.add(cloud_xyz(m))
                n=store.count()
                if not n:raise RuntimeError('Empty optional 3D cloud')
                with (work/'map.pcd').open('wb') as f:
                    f.write(('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA binary\n'%(n,n)).encode())
                    for b in store.batches():check();f.write(((b-engine.anchor)@engine.level.T).astype('<f4').tobytes())
                report['points_3d']=n
            finally:store.close()
            (work/'points.sqlite').unlink()
        atomic_json(work/'slice.json',report);check()
        # Exclusive destination creation; leave evidence if publication is interrupted.
        target.mkdir(exist_ok=False)
        for p in work.iterdir():check();os.link(p,target/p.name)
        return report
