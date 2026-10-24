#!/usr/bin/env python3
"""Read original rosbag files directly; no ROS publishers or robot connection."""
import argparse,json,math
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--mapping-run',required=True);ap.add_argument('--comparison-map',required=True);ap.add_argument('--output',required=True);args=ap.parse_args()
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu,PointCloud2
    from nav_msgs.msg import Odometry
    from real_navigation.geometry import Geometry,scan_points
    from real_navigation.localization import MapMatcher
    from mapping_slice import raw_points,stamp_ns
    from astribot_nav_sim.grid_world import GridWorld
    root=Path(args.mapping_run);script=Path(__file__).resolve().parent
    def reader(path,topics):
        r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(path),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=topics));return r
    meta=yaml.safe_load((root/'lio_bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
    topic=next(x['topic_metadata']['name'] for x in meta['topics_with_message_count'] if x['topic_metadata']['type']=='nav_msgs/msg/Odometry')
    lio=reader(root/'lio_bag',[topic]);poses=[]
    while lio.has_next():
        _,b,_=lio.read_next();m=deserialize_message(b,Odometry);p=m.pose.pose.position;q=m.pose.pose.orientation
        T=np.eye(4);T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();T[:3,3]=[p.x,p.y,p.z];poses.append((stamp_ns(m)*1e-9,T))
    imu=reader(root/'raw_bag',['/livox/imu_front']);acc=[]
    while imu.has_next() and len(acc)<100:
        _,b,_=imu.read_next();m=deserialize_message(b,Imu);v=m.linear_acceleration;acc.append([v.x,v.y,v.z])
    config=yaml.safe_load((script/'real_navigation/navigation.yaml').read_text());geo=Geometry(config)
    for v in acc:geo.imu(v)
    geo.pose(*poses[0]);idx=1
    # Map maker: anchor at first front laser; navigation: initial base heading.
    up=poses[0][1][:3,:3]@np.mean(acc,axis=0);level=Rotation.align_vectors([[0,0,1]],[up/np.linalg.norm(up)])[0].as_matrix()
    anchor=poses[0][1][:3,3]+poses[0][1][:3,:3]@np.asarray(config['imu_from_lidar'])[:3,3]
    map_from_raw=np.eye(4);map_from_raw[:3,:3]=level;map_from_raw[:3,3]=-level@anchor
    map_from_odom=map_from_raw@np.linalg.inv(geo.origin)
    original=next(root.glob('navigation_*/map.yaml'))
    matchers={k:MapMatcher(GridWorld(p),geo.base_lidar[:2,3]) for k,p in [('original',original),('edited',args.comparison_map)]}
    r=reader(root/'raw_bag',['/livox/lidar_front','/livox/lidar_back']);last={};rows=[]
    # Sample every 10 s across the recording; deserialize only sampled raw clouds.
    start=poses[0][0]
    while r.has_next():
        topic,b,received=r.read_next();t=received*1e-9
        if t<start+.3 or t-last.get(topic,-1e20)<10:continue
        last[topic]=t
        m=deserialize_message(b,PointCloud2)
        try:points,stamps,_=raw_points(m)
        except ValueError:continue
        if not len(points):continue
        reference=float(stamps.max())*1e-9
        while idx<len(poses) and poses[idx-1][0]<reference:
            geo.pose(*poses[idx]);idx+=1
        pose=geo.at(reference)
        if pose is None:continue
        deskew=geo.deskew(points,stamps,pose)
        if deskew is None:continue
        _,ranges=geo.cloud(deskew,pose,'back' in topic)
        local=scan_points(ranges)+geo.base_lidar[:3,3]
        exact=map_from_odom@pose;yaw=math.atan2(exact[1,0],exact[0,0]);c,s=math.cos(yaw),math.sin(yaw)
        planar=(local@exact[:3,:3].T)[:,:2]@np.array([[c,-s],[s,c]])
        p=[exact[0,3],exact[1,3],yaw]
        result={k:v.evaluate(planar,p) for k,v in matchers.items()}
        rows.append(dict(seconds=reference-start,sensor=topic,pose=p,points=planar.tolist(),metrics=result))
    # Independent motion-compensated fusion audit, retaining both source times.
    from real_navigation.localization import transform
    fused=[]
    front=[x for x in rows if x['sensor'].endswith('front')]
    back=[x for x in rows if x['sensor'].endswith('back')]
    for f in front:
        b=min(back,key=lambda x:abs(x['seconds']-f['seconds'])) if back else None
        if b is None or abs(b['seconds']-f['seconds'])>.2:continue
        world=transform(b['points'],b['pose']);yaw=f['pose'][2];c,s=math.cos(yaw),math.sin(yaw)
        local=(world-f['pose'][:2])@np.array([[c,-s],[s,c]])
        points=np.concatenate([f['points'],local]);origin=geo.base_lidar[:2,3];v=points-origin
        bins=np.floor((np.arctan2(v[:,1],v[:,0])+math.pi)/(2*math.pi)*720+.5).astype(int)%720
        ranges=np.full(720,np.inf);np.minimum.at(ranges,bins,np.linalg.norm(v,axis=1));ranges[~np.isfinite(ranges)]=np.nan
        points=scan_points(ranges)[:,:2]+origin
        fused.append(dict(seconds=f['seconds'],back_seconds=b['seconds'],metrics={k:m.evaluate(points,f['pose']) for k,m in matchers.items()}))
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
    data=dict(kind='original_raw_and_recorded_lio_no_live_publishers',mapping_run=str(root),original_map=str(original),comparison_map=args.comparison_map,frames=rows,fused=fused)
    output.write_text(json.dumps(data,indent=2))
    for name in matchers:
        for sensor in ['/livox/lidar_front','/livox/lidar_back']:
            rs=[x['metrics'][name] for x in rows if x['sensor']==sensor]
            print(name,sensor,'frames',len(rs),'accepted',sum(x['good'] for x in rs),'median_distance_m',float(np.median([x['median_distance_m'] for x in rs])) if rs else None,'median_match',float(np.median([x['match_fraction'] for x in rs])) if rs else None,flush=True)
if __name__=='__main__':main()
