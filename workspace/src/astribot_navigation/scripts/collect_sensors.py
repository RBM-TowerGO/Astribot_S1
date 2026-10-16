#!/usr/bin/env python3
"""Independent passive ROS recorder. No SDK, publishers or sensor services."""
import argparse
import json
from pathlib import Path
import signal
import time
from collection_common import CAMERA_TOPICS, CAMERAS, REQUIRED, OPTIONAL, source_stamp, atomic_json


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory')
    p.add_argument('--seconds', type=float, default=0, help='Optional passive recording duration')
    a = p.parse_args()
    run = Path(a.directory)
    run.mkdir(parents=True, exist_ok=True)
    import rclpy
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.utilities import get_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('towergo_collection_' + str(__import__('os').getpid()))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(run/'sensor_bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    stats, subs, previews = {}, {}, {}
    error, stopped = None, []
    start = time.monotonic()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda s, f: stopped.append(s))
    (run/'previews').mkdir(exist_ok=True)

    def callback(msg, topic):
        nonlocal error
        if error:
            return
        now, wall = time.monotonic(), time.time_ns()
        try:
            if topic in CAMERA_TOPICS:
                if CAMERA_TOPICS[topic]=='sensor_msgs/msg/CompressedImage':
                    if not len(msg.data): raise ValueError('Empty camera payload: '+topic)
                    if not stats[topic]['count'] and topic.endswith('/compressed'):
                        from PIL import Image
                        from io import BytesIO
                        with Image.open(BytesIO(bytes(msg.data))) as im: im.load()
                elif msg.image.width<=0 or msg.image.height<=0:
                    raise ValueError('Invalid camera dimensions: '+topic)
            payload = serialize_message(msg)
            writer.write(topic, payload, wall)
            s = stats[topic]
            stamp = source_stamp(msg)
            s['max_receive_gap_s'] = max(s['max_receive_gap_s'], now-s['last_mono'] if s['count'] else 0.)
            if stamp and s.get('source_stamp_ns'):
                delta = stamp-s['source_stamp_ns']
                if delta <= 0:
                    s['nonincreasing_stamps'] += 1
                else:
                    s['max_source_gap_s'] = max(s['max_source_gap_s'], delta/1e9)
            s.update(count=s['count']+1, last_mono=now, receive_wall_ns=wall, source_stamp_ns=stamp,
                     bytes=s['bytes']+len(payload))
            if not s.get('first_receive_wall_ns'):
                s['first_receive_wall_ns'] = wall
            if topic.endswith('/color_compress/compressed') and now-previews.get(topic, 0.) >= 2.:
                previews[topic] = now
                camera = topic.split('/')[2]
                path = run/'previews'/(camera+'.jpg')
                tmp = path.with_suffix('.tmp')
                tmp.write_bytes(bytes(msg.data)); tmp.replace(path)
        except Exception as exc:
            error = 'Sensor recording failed: ' + str(exc)

    def discover():
        known = dict(node.get_topic_names_and_types())
        extra=[t for t,kinds in known.items() if any(t.startswith('/astribot_camera/'+c+'/') for c in CAMERAS)
               and len(kinds)==1 and kinds[0] in ('sensor_msgs/msg/CompressedImage','astribot_camera/msg/CameraImage','sensor_msgs/msg/CameraInfo')]
        for topic in dict.fromkeys(REQUIRED + OPTIONAL + extra):
            if topic in subs or topic not in known:
                continue
            types = known[topic]
            if len(types) != 1:
                raise RuntimeError('Ambiguous topic type: '+topic)
            kind = types[0]
            if topic in CAMERA_TOPICS and kind != CAMERA_TOPICS[topic]:
                raise RuntimeError('Unexpected camera type: '+topic+' '+kind)
            msg_type = get_message(kind)
            writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=kind, serialization_format='cdr'))
            stats[topic] = dict(type=kind, count=0, bytes=0, last_mono=0., max_receive_gap_s=0.,
                                max_source_gap_s=0., nonincreasing_stamps=0)
            qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT)
            if topic == '/tf_static':
                qos.reliability = ReliabilityPolicy.RELIABLE
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            subs[topic] = node.create_subscription(msg_type, topic, lambda m, t=topic: callback(m, t), qos)

    def report(final=False):
        now = time.monotonic()
        rows = {t:dict(s, age_s=now-s['last_mono'] if s['count'] else None) for t,s in stats.items()}
        missing = [t for t in REQUIRED if not stats.get(t, {}).get('count')]
        stale = [t for t in REQUIRED if stats.get(t, {}).get('count') and now-stats[t]['last_mono'] > 2]
        return dict(updated_wall=time.time(), updated_monotonic=now, started_monotonic=start,
                    status=('incomplete' if error or missing or stale else 'closed') if final else 'recording',
                    ready=not error and not missing and not stale, error=error, required=REQUIRED,
                    missing=missing, stale=stale, streams=rows,
                    optional_missing=[t for t in OPTIONAL if not stats.get(t, {}).get('count')],
                    timing='Bag timestamp = recorder receive wall time; source Header preserved. No hardware sync claim.',
                    gap_note='Gaps and observed stamp anomalies recorded; DDS losses without sequence evidence are not exact drop counts.',
                    cameras=list(CAMERAS), depth_units='raw; metric scale and RGB alignment unverified')
    try:
        next_update = 0.
        while not stopped and not error and (not a.seconds or time.monotonic()-start<a.seconds):
            if time.monotonic() >= next_update:
                discover()
                atomic_json(run/'collection_live.json', report())
                next_update = time.monotonic()+.5
            rclpy.spin_once(node, timeout_sec=.05)
    except BaseException as exc:
        error = str(exc) or type(exc).__name__
    finally:
        # Destructor closes MCAP and writes metadata before final report.
        del writer
        node.destroy_node(); rclpy.shutdown()
        result = report(final=True)
        atomic_json(run/'collection_report.json', result)
        atomic_json(run/'collection_live.json', result)
    return 2 if result['status']=='incomplete' else 0


if __name__ == '__main__':
    raise SystemExit(main())
