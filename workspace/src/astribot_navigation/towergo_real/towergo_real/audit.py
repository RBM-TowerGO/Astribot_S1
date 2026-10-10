"""Inspect real recordings without replaying any ROS traffic."""
import argparse
import json
from pathlib import Path
import numpy as np

SENSORS = ['/livox/lidar_front', '/livox/lidar_back', '/livox/imu_front', '/livox/imu_back']


def audit_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2, Imu
    from .geometry import points_from_cloud
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''), rosbag2_py.ConverterOptions('', ''))
    rows, errors, imu_norms = {}, [], {}
    for topic in reader.get_all_topics_and_types():
        if topic.name.startswith('/ground_truth'):
            errors.append('Ground-truth topic found: not accepted as a real capture')
    while reader.has_next():
        topic, data, bag_stamp = reader.read_next()
        if topic not in SENSORS:
            continue
        msg = deserialize_message(data, Imu if '/imu_' in topic else PointCloud2)
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        row = rows.setdefault(topic, {'count':0,'first_ns':stamp,'last_ns':stamp,'max_gap_ns':0,'backwards_or_duplicates':0,'frames':[]})
        if row['count']:
            dt = stamp-row['last_ns']; row['max_gap_ns'] = max(row['max_gap_ns'], dt)
            row['backwards_or_duplicates'] += int(dt <= 0)
        row['last_ns'] = stamp; row['count'] += 1
        if msg.header.frame_id not in row['frames']:
            row['frames'].append(msg.header.frame_id)
        if '/imu_' in topic:
            v = msg.linear_acceleration
            g = msg.angular_velocity
            if not np.isfinite([v.x,v.y,v.z,g.x,g.y,g.z]).all():
                errors.append(topic+': nonfinite IMU')
            imu_norms.setdefault(topic, []).append(float(np.linalg.norm([v.x,v.y,v.z])))
        else:
            try:
                xyz = points_from_cloud(msg)
                if not np.isfinite(xyz).all():
                    raise ValueError('nonfinite XYZ')
                f = next(f for f in msg.fields if f.name == 'timestamp')
                if f.datatype != 8 or f.count != 1 or f.offset+8 > msg.point_step:
                    raise ValueError('timestamp must be float64')
                values = np.ndarray((msg.height,msg.width), dtype=('>' if msg.is_bigendian else '<')+'f8',
                                    buffer=msg.data, offset=f.offset, strides=(msg.row_step,msg.point_step)).ravel()
                delta = values.astype(np.longdouble)-stamp
                if not np.isfinite(delta).all() or np.min(delta)<-1000 or np.max(delta)>.2e9 or np.any(np.diff(values)<0):
                    raise ValueError('invalid absolute-ns point times')
                row['max_point_span_seconds'] = max(row.get('max_point_span_seconds',0),float(np.ptp(delta)*1e-9))
                row['points'] = row.get('points',0)+len(xyz)
            except (ValueError,StopIteration) as exc:
                errors.append(topic+': '+str(exc))
    for t in SENSORS:
        if t not in rows:
            errors.append('Missing '+t)
    for t,row in rows.items():
        row['duration_seconds']=(row['last_ns']-row['first_ns'])*1e-9
        if row['backwards_or_duplicates']:
            errors.append(t+': non-increasing timestamps')
        if t in imu_norms:
            row['acceleration_norm_median_native_units']=float(np.median(imu_norms[t]))
    return {'bag':str(Path(path).resolve()),'passed':not errors,'topics':rows,'errors':sorted(set(errors)),
            'validated_for_slam':False,'note':'Input integrity only; no motion, extrinsic or SLAM acceptance implied.'}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('bag');parser.add_argument('--output')
    args=parser.parse_args();report=audit_bag(args.bag)
    data=json.dumps(report,indent=2)
    if args.output:
        with Path(args.output).open('x') as f:f.write(data+'\n')
    print(data)
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':main()
