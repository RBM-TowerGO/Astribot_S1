#!/usr/bin/env python3
"""Offline coordinate diagnostics only; does not approve or rewrite an extrinsic."""
import json,hashlib,sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
bag=Path(sys.argv[1]); out=Path(sys.argv[2]); cfg=Path('/opt/astribot_ros/software/livox_ros_driver2/share/config/MID360_config.json')
if out.exists(): raise RuntimeError('Refusing to overwrite existing diagnostic report')
captured=bag.parent/'MID360_config.json'
if not captured.is_file() or captured.read_bytes()!=cfg.read_bytes():
 raise RuntimeError('Capture configuration missing or different from current driver')
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
pts={};times={};limits={};frames={}
while r.has_next():
 t,b,record=r.read_next()
 if t not in ['/livox/lidar_front','/livox/lidar_back']:continue
 if len(pts.get(t,[]))>=100:continue
 m=deserialize_message(b,PointCloud2);fields={f.name:f for f in m.fields}
 a=np.column_stack([np.ndarray((m.height,m.width),dtype='<f4',buffer=m.data,offset=fields[k].offset,strides=(m.row_step,m.point_step)).ravel() for k in 'xyz'])
 stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
 ns=np.ndarray((m.height,m.width),dtype='<f8',buffer=m.data,offset=fields['timestamp'].offset,strides=(m.row_step,m.point_step)).ravel()
 times.setdefault(t,[]).append(stamp);limits.setdefault(t,[]).append([float(ns.min()-stamp),float(ns.max()-stamp)])
 frames[t]=m.header.frame_id
 a=a[np.isfinite(a).all(1)];a=a[(np.linalg.norm(a,axis=1)>1)&(np.linalg.norm(a,axis=1)<15)]
 pts.setdefault(t,[]).append(a[::5])
 if len(pts)==2 and min(map(len,pts.values()))>=100:break
def voxel(p):
 _,ids=np.unique(np.floor(p/.05).astype(np.int64),axis=0,return_index=True);return p[ids]
f,b=[voxel(np.concatenate(pts[t])) for t in ['/livox/lidar_front','/livox/lidar_back']]
c=json.loads(cfg.read_text());e=c['lidar_configs'][1]['extrinsic_parameter'];rot=Rotation.from_euler('xyz',[e[k] for k in ['roll','pitch','yaw']],degrees=True).as_matrix();trans=np.array([e[k] for k in ['x','y','z']])/1000
# Compare same-region points, no fitted registration that could hide a bad extrinsic.
def metric(b):
 a=cKDTree(f).query(b)[0];rev=cKDTree(b).query(f)[0]
 return {'median_m':float(np.median(a)),'p90_m':float(np.percentile(a,90)), 'within_10cm':float(np.mean(a<.1)), 'symmetric_within_10cm':float((np.mean(a<.1)+np.mean(rev<.1))/2)}
raw=metric(b);twice=metric(b@rot.T+trans);inverse=metric((b-trans)@rot)
# Fit only existing local overlap; never apply this correction to production data.
corrected=b.copy();total_r=np.eye(3);total_t=np.zeros(3);tree=cKDTree(f)
for _ in range(20):
 dist,ids=tree.query(corrected);mask=dist<.15
 src=corrected[mask];dst=f[ids[mask]]
 c1,c2=src.mean(0),dst.mean(0);u,_,vt=np.linalg.svd((src-c1).T@(dst-c2));rr=vt.T@u.T
 if np.linalg.det(rr)<0:vt[-1]*=-1;rr=vt.T@u.T
 tt=c2-rr@c1;corrected=corrected@rr.T+tt;total_t=rr@total_t+tt;total_r=rr@total_r
correction={'translation_m':total_t.tolist(),'translation_norm_m':float(np.linalg.norm(total_t)),'rotation_matrix':total_r.tolist(),'rotation_deg':float(np.linalg.norm(Rotation.from_matrix(total_r).as_rotvec())*180/np.pi),'overlap_pairs':int(mask.sum()),'initial_overlap_rmse_m':float(np.sqrt(np.mean(tree.query(b)[0][tree.query(b)[0]<.15]**2))),'refined_overlap_rmse_m':float(np.sqrt(np.mean(dist[mask]**2)))}
a=np.array(times['/livox/lidar_front'],dtype=np.int64);z=np.array(times['/livox/lidar_back'],dtype=np.int64)
skew=np.min(abs(a[:,None]-z[None,:]),axis=1)/1e6
passed=raw['within_10cm']>.25 and correction['translation_norm_m']<.03 and correction['rotation_deg']<.3 and raw['symmetric_within_10cm']>twice['symmetric_within_10cm']+.15 and raw['symmetric_within_10cm']>inverse['symmetric_within_10cm']+.15
report={'verified':bool(passed),'coordinate_contract':'both_raw_topics_already_in_front_lidar_frame','bag':str(bag),'driver_config_sha256':hashlib.sha256(cfg.read_bytes()).hexdigest(),'driver_config':c,'frames':frames,'points':[len(f),len(b)],'overlap_registration_check':correction,'raw_alignment':raw,'applying_extrinsic_again':twice,'inverse_extrinsic':inverse,'nearest_header_skew_ms':np.percentile(skew,[0,50,95,100]).tolist(),'point_span_ns':{t:[min(x[0] for x in v),max(x[1] for x in v)] for t,v in limits.items()},'method':'100 static frames per lidar; 5 cm voxel; 1-15 m range; unfitted comparison plus diagnostic local ICP; compare raw vs repeated/inverse driver transform; >=25% rear within 10 cm (partial FOV); >15 percentage point symmetric margin; local overlap correction <3 cm and <0.3 deg; fitted correction is diagnostic only','hardware_commands_sent':False}
out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
