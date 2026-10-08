"""Offline, disk-backed extraction. No SDK or live ROS nodes."""
import csv
import json
import math
import os
from pathlib import Path
import sqlite3
import tempfile

import numpy as np
from mapping_support import atomic_json, check_metadata, cloud_xyz


class VoxelStore:
    def __init__(self, path, resolution=.05):
        self.db = sqlite3.connect(str(path))
        self.resolution = resolution
        self.db.execute('PRAGMA cache_size=-8192')
        self.db.execute('PRAGMA temp_store=FILE')
        self.db.execute('CREATE TABLE vox (ix INTEGER,iy INTEGER,iz INTEGER,x REAL,y REAL,z REAL, PRIMARY KEY(ix,iy,iz)) WITHOUT ROWID')

    def add(self, points):
        for start in range(0, len(points), 20000):
            chunk = points[start:start+20000]
            chunk = chunk[np.isfinite(chunk).all(axis=1)]
            key = np.floor(chunk/self.resolution).astype(np.int64)
            _, idx = np.unique(key, axis=0, return_index=True)
            self.db.executemany('INSERT OR IGNORE INTO vox VALUES (?,?,?,?,?,?)',
                                ((int(k[0]), int(k[1]), int(k[2]), float(p[0]), float(p[1]), float(p[2]))
                                 for k,p in zip(key[idx], chunk[idx])))
            self.db.commit()

    def count(self):
        return self.db.execute('SELECT count(*) FROM vox').fetchone()[0]

    def batches(self):
        cursor = self.db.execute('SELECT x,y,z FROM vox')
        while True:
            rows = cursor.fetchmany(20000)
            if not rows: break
            yield np.asarray(rows, dtype=np.float32)

    def close(self):
        self.db.close()


def products(store, trajectory, directory, cancelled=lambda:False):
    from PIL import Image, ImageDraw
    count = store.count()
    if count == 0: raise RuntimeError('No finite map points')
    out = Path(directory)
    def check():
        if cancelled(): raise InterruptedError('Finalization interrupted; recorded bags retained')
    header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              'WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n' % (count, count))
    with (out/'map.pcd').open('wb') as f:
        f.write(header.encode())
        for batch in store.batches():
            check(); f.write(batch.astype('<f4').tobytes())
    x0,y0,x1,y1 = store.db.execute('SELECT min(x),min(y),max(x),max(y) FROM vox').fetchone()
    with Path(trajectory).open() as f:
        for row in csv.DictReader(f):
            check()
            x,y = float(row['x']),float(row['y'])
            x0,y0,x1,y1 = min(x0,x),min(y0,y),max(x1,x),max(y1,y)
    span = max(x1-x0,y1-y0,.1); scale = 1100/span
    def xy(x,y): return (int(50+(x-x0)*scale), int(1150-(y-y0)*scale))
    canvas = np.full((1200,1200,3), 245, np.uint8)
    for batch in store.batches():
        check()
        px = np.clip((50+(batch[:,0]-x0)*scale).astype(int),0,1199)
        py = np.clip((1150-(batch[:,1]-y0)*scale).astype(int),0,1199)
        canvas[py,px] = [44,90,110]
    image = Image.fromarray(canvas); draw = ImageDraw.Draw(image)
    previous = None
    with Path(trajectory).open() as f:
        for row in csv.DictReader(f):
            check()
            current = xy(float(row['x']),float(row['y']))
            if previous: draw.line([previous,current], fill=(230,90,40), width=2)
            previous = current
    draw.text((20,15), 'FAST-LIO XY point projection / IMU trajectory (not occupancy)', fill=(20,30,40))
    image.save(out/'preview.png')
    return count


def finalize(run, cancelled=lambda:False):
    run = Path(run).resolve()
    session = json.loads((run/'session.json').read_text())
    topics = session['topics']; bag = run/'lio_bag'
    check_metadata(bag, [topics['odom'], topics['cloud']])
    outputs = ['map.pcd','trajectory.csv','preview.png','finalize.json']
    if any((run/name).exists() for name in outputs):
        raise RuntimeError('Result already exists; refusing to overwrite')
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),
                rosbag2_py.ConverterOptions('',''))
    # Store work inside this run; partial work never replaces final products.
    with tempfile.TemporaryDirectory(prefix='.finalize-', dir=run) as tmp:
        work = Path(tmp); store = VoxelStore(work/'voxels.sqlite')
        count, clouds, last, frame = 0,0,None,None
        try:
            with (work/'trajectory.csv').open('w',newline='') as f:
                writer = csv.writer(f); writer.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw'])
                while reader.has_next():
                    if cancelled(): raise InterruptedError('Finalization interrupted; recorded bags retained')
                    topic, data, stamp = reader.read_next()
                    if topic == topics['cloud']:
                        m = deserialize_message(data, PointCloud2)
                        if frame is not None and frame != m.header.frame_id:
                            raise RuntimeError('Recorded cloud/odometry frame mismatch')
                        frame=m.header.frame_id
                        store.add(cloud_xyz(m)); clouds += 1
                    elif topic == topics['odom']:
                        m = deserialize_message(data, Odometry)
                        if frame is not None and frame != m.header.frame_id:
                            raise RuntimeError('Recorded cloud/odometry frame mismatch')
                        frame=m.header.frame_id
                        ns = m.header.stamp.sec*1000000000 + m.header.stamp.nanosec
                        p,q = m.pose.pose.position,m.pose.pose.orientation
                        values = [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
                        if ns <= 0 or (last is not None and ns <= last) or not all(math.isfinite(v) for v in values):
                            raise RuntimeError('Invalid/non-monotonic trajectory')
                        writer.writerow([ns,*values]); last=ns; count+=1
            if not count or not clouds: raise RuntimeError('Missing trajectory/cloud stream')
            points = products(store, work/'trajectory.csv', work,cancelled)
            report = {'points':points,'poses':count,'cloud_messages':clouds,'voxel_m':.05,
                      'frame':frame,'trajectory_reference':'IMU','source_session_status':session['status'],
                      'source_session_error':session.get('error'), 'navigation_map':False,
                      'lidar_mode':session.get('lidar_mode','front'),
                      'live_alignment_verified':session.get('live_alignment_verified'),
                      'coordinate_validation':session.get('coordinate_validation')}
            atomic_json(work/'finalize.json', report)
            if cancelled(): raise InterruptedError('Finalization interrupted; recorded bags retained')
            for name in outputs:
                # Hard links provide exclusive publish: concurrent finalizers
                # cannot silently replace a result created after preflight.
                os.link(work/name, run/name)
            return report
        finally:
            store.close()
