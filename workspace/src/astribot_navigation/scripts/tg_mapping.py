#!/usr/bin/env python3
"""S1 teleop + FAST-LIO session. Replay/finalize never construct an SDK."""
import argparse
import datetime
import fcntl
import json
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import sys
import threading
import time
import uuid

import yaml
from mapping_support import (RAW_TOPICS, REQUIRED_RAW_TOPICS, State, OwnedProcess, atomic_json, cloud_xyz,
                             check_metadata, shutdown_order, web_server)

HERE = Path(__file__).resolve().parent
DATA = HERE.parents[2]/'data'/'mapping'


def topic_names(prefix):
    return {key: prefix+'/'+suffix for key,suffix in {
        'lidar':'lidar', 'imu':'imu', 'odom':'odometry', 'cloud':'cloud_registered',
        'fault':'fault', 'diagnostics':'diagnostics', 'tf':'tf', 'tf_static':'tf_static',
        'occupancy':'occupancy', 'path':'path', 'body':'cloud_body', 'map':'laser_map', 'effect':'cloud_effected'}.items()}


def parameters(base, topics, replay):
    doc = yaml.safe_load(Path(base).read_text())
    p = doc['/**']['ros__parameters']
    p['use_sim_time'] = replay
    p['common']['lid_topic'],p['common']['imu_topic'] = topics['lidar'],topics['imu']
    p['mapping']['extrinsic_est_en'] = False
    p.setdefault('preprocess', {})['scan_line'] = 8
    p['feature_extract_enable'] = False
    p['pcd_save']['pcd_save_en'] = False
    p['publish'].update(map_en=False, path_en=False, scan_publish_en=True,
                        scan_bodyframe_pub_en=False)
    return doc


class ROSObserver:
    def __init__(self, state, topics, replay):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import PointCloud2, Imu
        from nav_msgs.msg import Odometry
        from livox_ros_driver2.msg import CustomMsg
        from std_msgs.msg import Bool
        from diagnostic_msgs.msg import DiagnosticArray
        self.rclpy, self.state = rclpy,state
        from rclpy.signals import SignalHandlerOptions
        # Session owns SIGINT/SIGTERM; ROS must not shut its context underneath it.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.name = 'tg_mapping_observer_'+str(os.getpid())
        self.node = rclpy.create_node(self.name)
        q = QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.last_preview = self.last_path = 0.
        def stamp(m): return m.header.stamp.sec*1000000000+m.header.stamp.nanosec
        for topic,key,kind in [(RAW_TOPICS[0],'front',PointCloud2),(RAW_TOPICS[1],'imu',Imu),
                               (RAW_TOPICS[2],'back',PointCloud2),
                               (topics['lidar'],'adapted_cloud',CustomMsg),(topics['imu'],'adapted_imu',Imu)]:
            self.node.create_subscription(kind,topic,lambda m,k=key: state.seen(k,stamp(m)),q)
        def odom(m):
            state.seen('odom',stamp(m))
            now=time.monotonic()
            if now-self.last_path < .1: return
            self.last_path=now
            p,v=m.pose.pose.position,m.pose.pose.orientation
            try:
                with state.lock:
                    if state.frame is not None and state.frame != m.header.frame_id:
                        raise ValueError('LIO cloud/odometry frames disagree')
                    state.frame=m.header.frame_id
                    state.preview.odom([p.x,p.y,p.z],[v.x,v.y,v.z,v.w])
            except Exception as exc: state.error=str(exc)
        def cloud(m):
            state.seen('cloud',stamp(m))
            now=time.monotonic()
            if now-self.last_preview < 1: return
            self.last_preview=now
            try:
                with state.lock:
                    if state.frame is not None and state.frame != m.header.frame_id:
                        raise ValueError('LIO cloud/odometry frames disagree')
                    state.frame=m.header.frame_id
                    pass  # Slice worker owns the displayed navigation map.
            except Exception as exc: state.error=str(exc)
        self.node.create_subscription(Odometry,topics['odom'],odom,q)
        self.node.create_subscription(PointCloud2,topics['cloud'],cloud,q)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        def fault(m):
            if m.data and not state.error: state.error='LiDAR adapter reports a session fault'
        self.node.create_subscription(Bool,topics['fault'],fault,latched)
        def diag(m):
            for s in m.status:
                level = s.level[0] if isinstance(s.level,bytes) else int(s.level)
                if level >= 2: state.error='LiDAR adapter: '+s.message
                if s.name == 'towergo/dual_livox':
                    with state.lock:
                        state.fusion = {v.key: float(v.value) for v in s.values}
                        state.fusion_reason=s.message
        self.node.create_subscription(DiagnosticArray,topics['diagnostics'],diag,QoSProfile(depth=20))
        def spin():
            try: rclpy.spin(self.node)
            except Exception as exc:
                if rclpy.ok(): state.error='Observer failed: '+str(exc)
        self.thread=threading.Thread(target=spin,daemon=True); self.thread.start()

    def lidar(self, on):
        from std_srvs.srv import SetBool
        client=self.node.create_client(SetBool,'/astribot_lidar_control')
        try:
            if not client.wait_for_service(timeout_sec=5): raise RuntimeError('LiDAR control service unavailable')
            req=SetBool.Request(); req.data=on
            future=client.call_async(req); deadline=time.monotonic()+15
            while not future.done() and time.monotonic()<deadline: time.sleep(.05)
            if not future.done(): raise RuntimeError('LiDAR service timed out; outcome unknown, do not retry automatically')
            response=future.result()
            if not response.success: raise RuntimeError('LiDAR service failed: '+response.message)
        finally: self.node.destroy_client(client)

    def camera(self, on):
        from std_srvs.srv import SetBool
        client=self.node.create_client(SetBool,'/astribot_camera_control')
        try:
            if not client.wait_for_service(timeout_sec=5): raise RuntimeError('Camera control service unavailable')
            req=SetBool.Request(); req.data=on
            future=client.call_async(req); deadline=time.monotonic()+20
            while not future.done() and time.monotonic()<deadline: time.sleep(.05)
            if not future.done(): raise RuntimeError('Camera service timed out; outcome unknown, no automatic retry')
            response=future.result()
            if not response.success: raise RuntimeError('Camera service failed: '+response.message)
        finally: self.node.destroy_client(client)

    def other_camera_users(self):
        from collection_common import CAMERA_TOPICS
        return sorted({info.node_name for topic in CAMERA_TOPICS
                       for info in self.node.get_subscriptions_info_by_topic(topic)
                       if info.node_name not in (self.name, "compress_camera_driver_node")})

    def other_sensor_users(self):
        users=set()
        for topic in RAW_TOPICS:
            for info in self.node.get_subscriptions_info_by_topic(topic):
                if info.node_name != self.name: users.add(info.node_name)
        return sorted(users)

    def close(self):
        if getattr(self, '_closed', False): return
        self._closed=True
        try:
            self.rclpy.try_shutdown()
        finally:
            self.thread.join(timeout=3)
            self.node.destroy_node()


class Session:
    def __init__(self,args):
        self.args=args; self.replay=bool(args.replay)
        self.run_id='mapping_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
        self.run=Path(args.output_root).expanduser().resolve()/self.run_id
        self.topics=topic_names('/towergo/mapping/'+self.run_id)
        self.state=State(self.run_id)
        self.procs={}; self.observer=self.server=None
        self.collect = not self.replay and not getattr(args, "mapping_only", False)
        self.health_write=None; self.signal=None; self.owned_lidar=False; self.owned_camera=False
        self.last_slice_read=0.
        self.manual_active=False
        self.health_stop=threading.Event(); self.health_thread=None
        self.capture_stop_at=None; self.capture_stop_stage=0
        self.last_health_check=0.; self.last_warning_save=0.
        self.report_warnings={}; self.warning_dirty=False
        self.report={'run_id':self.run_id,'status':'initializing','error':None,'topics':self.topics,
                     'replay':self.replay,'hardware_commands_sent':False,'lidar_started':False,
                     'navigation_map':True,'shutdown_errors':[],
                     'lidar_mode':'dual','imu_source':'front',
                     'fusion_parameters':{'window_ns':100000000,'max_buffer_ns':1000000000,'lines':8,
                                          'rear_transform_applied_by_adapter':False}}

    def spawn(self,name,cmd,tty=False,pass_fds=()):
        if not tty:
            # Background computation/recording yields CPU and best-effort I/O to SSH/control.
            cmd=['nice','-n','10','ionice','-c','2','-n','7',*cmd]
        p=OwnedProcess(name,cmd,log=None if tty else self.run/(name+'.log'),tty=tty,pass_fds=pass_fds)
        self.procs[name]=p
        self.report.setdefault('processes',{})[name]={'pid':p.proc.pid,'command':cmd}
        return p

    def heartbeat(self):
        if self.health_write is not None:
            try: os.write(self.health_write,struct.pack('!d',time.monotonic()))
            except (BrokenPipeError,BlockingIOError): pass

    def start_heartbeat(self):
        self.health_stop.clear()
        def beat():
            while not self.health_stop.is_set():
                self.heartbeat()
                self.health_stop.wait(.1)
        self.health_thread=threading.Thread(target=beat,daemon=True)
        self.health_thread.start()

    def revoke(self):
        self.health_stop.set()
        if self.health_thread:
            self.health_thread.join(timeout=1); self.health_thread=None
        if self.health_write is not None:
            os.close(self.health_write); self.health_write=None

    def stop_recording_for_disk(self):
        # Signal all recorders before waiting; never block keyboard heartbeat on flush.
        if self.capture_stop_at is None:
            self.capture_stop_at=time.monotonic()
            self.report['recording_degraded']=True
            self.report['recording_stop_reason']='Less than 3 GiB free disk space'
        elapsed=time.monotonic()-self.capture_stop_at
        stage=3 if elapsed>=20 else 2 if elapsed>=10 else 1
        if stage>self.capture_stop_stage:
            self.capture_stop_stage=stage
            sig={1:signal.SIGINT,2:signal.SIGTERM,3:signal.SIGKILL}[stage]
            for name in ('raw_record','lio_record','sensors'):
                if name in self.procs: self.procs[name].send(sig)

    def publish_warnings(self, issues):
        now=time.time(); active={key:message for key,message,_ in issues}
        previous=self.state.warnings
        if active!=previous:
            self.warning_dirty=True
            for key,message,damaged in issues:
                item=self.report_warnings.setdefault(key,dict(first_seen=now,occurrences=0))
                if previous.get(key)!=message:
                    item['occurrences']+=1
                    print('\n警告：'+message+'；键盘遥操继续，X 可结束。',flush=True)
                item.update(message=message,last_seen=now,active=True)
                if damaged: self.report['recording_degraded']=True
            for key in previous.keys()-active.keys():
                self.report_warnings[key].update(active=False,recovered_at=now)
            with self.state.lock: self.state.warnings=active
        self.report['warnings']=self.report_warnings
        if self.warning_dirty and time.monotonic()-self.last_warning_save>=1:
            self.last_warning_save=time.monotonic()
            try:
                atomic_json(self.run/'runtime_warnings.json',self.report_warnings)
                self.warning_dirty=False
            except OSError:
                # A full/unavailable recording filesystem must not take down control.
                self.report['recording_degraded']=True

    def check(self,require_streams=False,allow_player_end=False):
        if self.signal: raise InterruptedError('Session interrupted by signal %s'%self.signal)
        advisory=self.manual_active and not self.replay
        issues=[]
        def issue(key,message,damaged=True):
            if not advisory: raise RuntimeError(message)
            issues.append((key,message,damaged))
        for name,p in self.procs.items():
            if name=='teleop' or (name=='player' and allow_player_end): continue
            if p.poll() is not None: issue('process_'+name,'%s exited (%s)'%(name,p.poll()))
        try:
            free=shutil.disk_usage(self.run).free/1024**3
            self.state.free_gib=round(free,2)
            if free<3:
                issue('disk','Less than 3 GiB free disk space; stopping recording only')
                self.stop_recording_for_disk()
        except OSError as exc: issue('disk_status','Disk status unavailable: '+str(exc))
        if self.capture_stop_at is not None:
            self.stop_recording_for_disk()
            issue('recording_stopped','录制已因磁盘不足停止，不会自动重启')
        if self.state.notes:
            try: self.state.notes.check()
            except RuntimeError as exc: issue('annotations',str(exc))
        if 'slice' in self.procs:
            try:
                data=json.loads((self.run/'slice_live.json').read_text())
                with self.state.lock: self.state.slice=data
                if data.get('error'): issue('slice_error','Slice worker: '+data['error'])
                # Compare data from the SAME snapshot, not a 1 Hz snapshot with newer LIO.
                pose_ns=data.get('lio_stamp_ns',data.get('stamp_ns',0))
                if require_streams and pose_ns-data.get('stamp_ns',0)>2000000000:
                    issue('slice_lag','地图切片落后其 LIO 输入超过 2 秒',False)
                if require_streams and time.time()-data['worker_updated']>3:
                    issue('slice_stale','地图预览超过 3 秒未更新',False)
            except (OSError,ValueError,KeyError) as exc:
                if require_streams: issue('slice_status','Slice status unavailable: '+str(exc))
        if self.collect and 'sensors' in self.procs:
            try:
                collection=json.loads((self.run/'collection_live.json').read_text())
                with self.state.lock: self.state.collection=collection
                if collection.get('error'): issue('collection_error',collection['error'])
                if require_streams and (time.monotonic()-collection['updated_monotonic']>3 or not collection['ready']):
                    issue('collection_stale','Collection stale/missing: '+str(collection.get('stale') or collection.get('missing')))
            except (OSError,ValueError,KeyError) as exc:
                if require_streams: issue('collection_status','Collection status unavailable: '+str(exc))
        if self.state.error: issue('observer',self.state.error)
        if require_streams and not self.state.healthy():
            issue('streams','Sensor/LIO stream stale for more than 2 seconds')
        if advisory: self.publish_warnings(issues)

    def pause(self,seconds,require_streams=False):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            self.check(require_streams); time.sleep(.1)

    def prepare(self):
        if os.environ.get('ROS_DOMAIN_ID') != ('125' if self.replay else '25'):
            raise RuntimeError('Wrong ROS domain; use tg-mapping.sh')
        if self.replay and os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            raise RuntimeError('Replay must be localhost-only')
        if not self.replay:
            if not os.isatty(0): raise RuntimeError('Live mapping requires SSH TTY')
            from tg_teleop import conflicts,SessionLock
            if conflicts(): raise RuntimeError('Other chassis controller running; exit it first')
            with SessionLock(): pass
        from ament_index_python.packages import get_package_prefix,get_package_share_directory
        self.fast=Path(get_package_prefix('fast_lio'))/'lib/fast_lio/fastlio_mapping'
        self.adapter=Path(get_package_prefix('towergo_dual_livox'))/'lib/towergo_dual_livox/dual_adapter'
        import rosbag2_py
        from PIL import Image
        from livox_ros_driver2.msg import CustomMsg
        if 'mcap' not in rosbag2_py.get_registered_writers(): raise RuntimeError('MCAP writer missing')
        if not self.fast.is_file() or not self.adapter.is_file(): raise RuntimeError('Runtime binaries missing')
        if self.replay:
            check_metadata(Path(self.args.replay).resolve(),REQUIRED_RAW_TOPICS)
        self.run.parent.mkdir(parents=True,exist_ok=True)
        if shutil.disk_usage(self.run.parent).free < 5*1024**3: raise RuntimeError('At least 5 GiB free required')
        self.run.mkdir(exist_ok=False)
        from dual_lidar_contract import coordinate_diagnostics, CONFIG, EVIDENCE
        evidence_dir=EVIDENCE.parent
        snapshot_notes=[]
        for source in (CONFIG, EVIDENCE, evidence_dir/'static_first.json', evidence_dir/'static_second.json'):
            try: shutil.copyfile(source, self.run/source.name)
            except OSError as exc: snapshot_notes.append(str(exc))
        verification=coordinate_diagnostics()
        verification['snapshot_notes']=snapshot_notes
        self.report['coordinate_validation']=verification
        self.report['live_alignment_verified']=verification['live_alignment_verified']
        if verification['diagnostic_notes']:
            self.state.validation_warning=' '.join(verification['diagnostic_notes'])
            print('标定参考：'+self.state.validation_warning,flush=True)
        atomic_json(self.run/'session.json',self.report)
        self.report['collection_enabled']=self.collect
        self.report['task']=getattr(self.args,'task','')
        if self.collect:
            self.state.collection_directory=self.run
            self.state.task=getattr(self.args,'task','')
            atomic_json(self.run/'task.json', {'description':self.state.task})
            import secrets
            self.state.note_token=secrets.token_urlsafe(24)
            from collection_common import EventLog, REQUIRED, OPTIONAL, CAMERAS
            atomic_json(self.run/'collection_config.json',dict(cameras=CAMERAS,required=REQUIRED,optional=OPTIONAL,
                qos={'default':'best_effort volatile keep_last depth=30','tf_static':'reliable transient_local depth=30'},
                depth_units='unverified',timing='Header source time plus recorder receive wall time',
                navigation_cameras=['head_rgbd','torso_rgbd']))
            self.state.notes=EventLog(self.run/'annotations.jsonl')
            snapshots=self.run/'snapshots'; snapshots.mkdir()
            for source in ['/etc/config/camera_info.json',
                           '/home/astribot/camera_parameter/astribot_camera_parameter.json',
                           '/home/astribot/camera_parameter/filp_config_for_robot.json']:
                src=Path(source)
                if src.is_file(): shutil.copyfile(src,snapshots/src.name)
            models=Path('/home/astribot/astribot_sdk_aarch64/astribot_config/robot_config/astribot_s1/model')
            if models.is_dir():
                target=snapshots/'robot_model'; target.mkdir()
                for f in models.glob('*.urdf'): shutil.copyfile(f,target/f.name)
            import hashlib
            sources=list(HERE.glob('*.py'))+list(HERE.glob('tg-*.sh'))+[HERE.parent/'web/mapping.html']
            code=snapshots/'code'; code.mkdir()
            for f in sources: shutil.copyfile(f,code/f.name)
            atomic_json(snapshots/'code_hashes.json',{str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources})
        self.server=web_server(self.state,self.args.port,(HERE.parent/'web/mapping.html').read_bytes())
        base=Path(get_package_share_directory('fast_lio'))/'config/mid360.yaml'
        (self.run/'lio_params.yaml').write_text(yaml.safe_dump(parameters(base,self.topics,self.replay),sort_keys=False))
        shutil.copyfile(base,self.run/'base_mid360.yaml')
        try: shutil.copyfile('/etc/config/sensor_calib.json',self.run/'sensor_calib.json')
        except OSError: pass
        from mapping_slice_finalize import configuration
        config=configuration(self.run,getattr(self.args,'slice_offset',0.),getattr(self.args,'slice_thickness',.1))
        atomic_json(self.run/'slice_config.json',config)
        self.report['slice_parameters']=config
        self.report['git_commit']=subprocess.check_output(['git','-C',str(HERE),'rev-parse','HEAD'],text=True).strip()
        self.report['git_status']=subprocess.check_output(['git','-C',str(HERE),'status','--short'],text=True)
        self.report['frames']='FAST-LIO world; trajectory follows IMU, not chassis'
        atomic_json(self.run/'session.json',self.report)
        self.observer=ROSObserver(self.state,self.topics,self.replay)
        print('会话: %s\n网页: http://127.0.0.1:%d\n等待传感器和 SLAM 初始化，暂不接受驾驶。'%(self.run,self.args.port),flush=True)

    def start_mapping(self):
        if not self.replay:
            self.pause(2)
            # Do not toggle existing publishers merely because their data is stalled.
            existing=any(self.observer.node.count_publishers(t)>0 for t in REQUIRED_RAW_TOPICS)
            if not existing:
                self.observer.lidar(True); self.owned_lidar=True; self.report['lidar_started']=True
        topics=self.topics
        if self.collect:
            from collection_common import CAMERA_TOPICS
            if not any(self.observer.node.count_publishers(t)>0 for t in CAMERA_TOPICS):
                self.report['camera_start_requested']=True
                atomic_json(self.run/'session.json',self.report)
                self.observer.camera(True); self.owned_camera=True; self.report['camera_started']=True
            self.spawn('sensors',[sys.executable,str(HERE/'collect_sensors.py'),str(self.run)])
        self.spawn('slice',[sys.executable,str(HERE/'mapping_slice_node.py'),str(self.run)])
        qos={t:{'reliability':'reliable' if self.replay else 'best_effort','durability':'volatile','history':'keep_last','depth':100}
             for t in RAW_TOPICS+[topics['odom'],topics['cloud']]}
        qos[topics['occupancy']]={'reliability':'reliable','durability':'transient_local','history':'keep_last','depth':1}
        (self.run/'record_qos.yaml').write_text(yaml.safe_dump(qos))
        for name,directory,selected in [('raw_record','raw_bag',RAW_TOPICS),
                                        ('lio_record','lio_bag',[topics['odom'],topics['cloud'],topics['occupancy']])]:
            self.spawn(name,['ros2','bag','record','-s','mcap','-o',str(self.run/directory),
                             '--max-cache-size','33554432','--qos-profile-overrides-path',str(self.run/'record_qos.yaml'),*selected])
        sim='true' if self.replay else 'false'
        remaps={'/towergo/lio/lidar':topics['lidar'],'/towergo/lio/imu':topics['imu'],
                '/towergo/mapping_fault':topics['fault'],'/diagnostics':topics['diagnostics']}
        cmd=[str(self.adapter),'--ros-args','-p','imu_acceleration_unit:=g','-p','use_sim_time:='+sim,
             '-p','coordinate_contract:=both_raw_topics_already_in_front_lidar_frame',
             '-p','reliable_input:='+sim,
             '-r','__node:=adapter_'+self.run_id]
        for old,new in remaps.items(): cmd+=['-r',old+':='+new]
        self.spawn('adapter',cmd)
        cmd=[str(self.fast),'--ros-args','--params-file',str(self.run/'lio_params.yaml'),'-r','__node:=lio_'+self.run_id]
        for old,key in [('/Odometry','odom'),('/cloud_registered','cloud'),('/tf','tf'),('/tf_static','tf_static'),
                        ('/path','path'),('/cloud_registered_body','body'),('/Laser_map','map'),('/cloud_effected','effect')]:
            cmd+=['-r',old+':='+topics[key]]
        self.spawn('fastlio',cmd)
        self.pause(3)
        if self.replay:
            # A second bag player would invalidate clock and sensor timestamps.
            if self.observer.node.count_publishers('/clock') or any(self.observer.node.count_publishers(t) for t in REQUIRED_RAW_TOPICS):
                raise RuntimeError('Replay domain already has a clock/sensor publisher')
            self.spawn('player',['ros2','bag','play',str(Path(self.args.replay).resolve()),'--clock','--rate','1.0','--delay','2',
                                 '--qos-profile-overrides-path',str(self.run/'record_qos.yaml'),
                                 '--wait-for-all-acked','2000',
                                 '--topics',*RAW_TOPICS])
        self.recording_started=time.monotonic()+(2 if self.replay else 0)
        self.replay_started=self.recording_started
        deadline=self.recording_started+60
        while time.monotonic()<deadline:
            self.check()
            if (self.state.ready(time.monotonic(),self.recording_started)
                and (not self.collect or self.state.collection.get("ready",False))
                    and self.state.slice.get('statistics',{}).get('frames',0)>0): return
            time.sleep(.1)
        raise RuntimeError('Initialization timed out (60 seconds); no driving enabled; missing collection topics: '+str(self.state.collection.get('missing',[])))

    def drive(self):
        self.state.status=self.report['status']='mapping'
        atomic_json(self.run/'session.json',self.report)
        print('READY：双雷达数据链路就绪；'+('离线回放不启动 SDK。' if self.replay else '进入键盘遥操，X 结束并保存。'),flush=True)
        if self.replay:
            while True:
                self.check(allow_player_end=True)
                player=self.procs['player']
                if player.poll() is not None:
                    if player.poll()!=0: raise RuntimeError('Replay player failed')
                    break
                self.check(require_streams=True,allow_player_end=True)
                if self.args.replay_seconds and time.monotonic()-self.replay_started>=self.args.replay_seconds:
                    player.stop(); break
                time.sleep(.1)
            return
        read_fd,self.health_write=os.pipe(); os.set_blocking(self.health_write,False)
        self.heartbeat()
        self.start_heartbeat()
        self.manual_active=True
        try:
            child=self.spawn('teleop',[str(HERE/'tg-teleop.sh'),'--session-fd',str(read_fd),
                                      '--linear-speed',str(self.args.linear_speed),
                                      '--angular-speed',str(self.args.angular_speed),
                                      '--torso-speed',str(getattr(self.args,'torso_speed',.02)),
                                      *(['--event-log',str(self.run/'teleop.jsonl')] if self.collect else [])],tty=True,pass_fds=(read_fd,))
        finally: os.close(read_fd)
        self.report['teleop_started']=True
        self.report['hardware_commands_sent']=None  # SDK motion depends on keyboard input; not inferred here.
        atomic_json(self.run/'session.json',self.report)
        while child.poll() is None:
            if time.monotonic()-self.last_health_check>=.5:
                self.check(require_streams=True); self.last_health_check=time.monotonic()
            if self.signal: raise InterruptedError('Session interrupted by signal %s'%self.signal)
            time.sleep(.05)
        self.revoke()
        if child.poll()!=0: raise RuntimeError('Teleop exited abnormally: %s'%child.poll())
        self.state.status='sensor_tail'
        self.pause(5,require_streams=True)

    def cleanup(self):
        self.revoke()
        for name in shutdown_order(self.procs):
            try:
                # EOF asks teleop to hold/release; do not shut native ROS down with SIGINT first.
                if name=='teleop' and hasattr(self.procs[name],'proc'):
                    try: self.procs[name].proc.wait(timeout=6)
                    except subprocess.TimeoutExpired: pass
                self.procs[name].stop(timeout=30 if 'record' in name or name=='sensors' else 8)
            except Exception as exc: self.report['shutdown_errors'].append(str(exc))
        if self.owned_camera and self.observer:
            try:
                time.sleep(2)
                users=self.observer.other_camera_users()
                if users: self.report['camera_left_for_users']=users
                else: self.observer.camera(False); self.report['camera_stopped']=True
            except Exception as exc: self.report['shutdown_errors'].append('Camera: '+str(exc))
        if self.owned_lidar and self.observer:
            try:
                time.sleep(2)  # Let stopped subscriptions leave discovery.
                users=self.observer.other_sensor_users()
                if users: self.report['lidar_left_for_users']=users
                else: self.observer.lidar(False); self.report['lidar_stopped']=True
            except Exception as exc: self.report['shutdown_errors'].append('LiDAR: '+str(exc))

    def execute(self):
        handlers={}
        normal=False
        try:
            for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
                handlers[sig]=signal.signal(sig,lambda s,f: setattr(self,'signal',s))
            self.prepare(); self.start_mapping(); self.drive(); normal=True
        except BaseException as exc:
            self.report['error']=str(exc) or type(exc).__name__
            self.state.error=self.report['error']
        finally:
            self.state.status='closing'
            self.cleanup()
            if self.report.get('error') == 'LiDAR adapter reports a session fault':
                try:
                    errors=[line for line in (self.run/'adapter.log').read_text().splitlines() if '[ERROR]' in line]
                    if errors: self.report['error']='LiDAR adapter: '+errors[-1].rsplit(']: ',1)[-1]
                except OSError: pass
            if self.observer:
                try: self.observer.close()
                except Exception as exc: self.report['shutdown_errors'].append('Observer: '+str(exc))
            if self.signal and not self.report['error']:
                self.report['error']='Interrupted during shutdown by signal %s'%self.signal
            self.report['status']='recorded' if normal and not self.signal and not self.report['shutdown_errors'] and not self.report.get('recording_degraded') else 'incomplete'
            if self.run.exists():
                try:
                    self.report['raw_counts']=check_metadata(self.run/'raw_bag',REQUIRED_RAW_TOPICS)
                    self.report['lio_counts']=check_metadata(self.run/'lio_bag',[self.topics['odom'],self.topics['cloud'],self.topics['occupancy']])
                except Exception as exc:
                    self.report['status']='incomplete'; self.report['recording_error']=str(exc)
                if self.collect:
                    try:
                        from collection_common import REQUIRED
                        self.report['sensor_counts']=check_metadata(self.run/'sensor_bag',REQUIRED)
                        collection=json.loads((self.run/'collection_report.json').read_text())
                        if collection['status']!='closed': raise RuntimeError('Sensor recorder incomplete: '+str(collection.get('error') or collection.get('missing')))
                        self.report['collection_report']='collection_report.json'
                        if self.report.get('teleop_started'):
                            with (self.run/'teleop.jsonl').open() as f:
                                ended=any(json.loads(line)['event']=='session_end' for line in f)
                            if not ended: raise RuntimeError('Teleop event log not closed')
                    except Exception as exc:
                        self.report['status']='incomplete'; self.report['collection_error']=str(exc)
                if (self.report.get('error') == 'LiDAR adapter reports a session fault'
                        and self.state.error and self.state.error.startswith('LiDAR adapter:')):
                    self.report['error']=self.state.error
                self.report['frame']=self.state.frame
                with self.state.lock:
                    self.report['fusion_statistics']=dict(self.state.fusion)
                    self.report['fusion_last_diagnostic']=self.state.fusion_reason
                    self.report['slice_statistics']=self.state.slice.get('statistics',{})
                self.report['acceptance']='software_replay_only' if self.replay else 'onsite_pending'
                atomic_json(self.run/'session.json',self.report)
            try:
                if self.report['status']=='recorded' and not self.signal:
                    self.state.status='finalizing'
                    print('录包已关闭，正在生成地图和轨迹…',flush=True)
                    from mapping_slice_finalize import finalize_slice
                    self.report['products']=finalize_slice(self.run,output=self.run/'navigation',
                        export_pcd=getattr(self.args,'export_pcd',False),cancelled=lambda:bool(self.signal))
                    self.report['status']='complete'
            except Exception as exc:
                self.report['status']='incomplete'; self.report['finalize_error']=str(exc)
            finally:
                self.state.status=self.report['status']
                if self.server: self.server.shutdown(); self.server.server_close()
                if self.state.notes:
                    try: self.state.notes.close()
                    except Exception as exc:
                        self.report['status']='incomplete'; self.report['annotation_error']=str(exc)
                if self.run.exists(): atomic_json(self.run/'session.json',self.report)
                for sig,handler in handlers.items(): signal.signal(sig,handler)
        print(json.dumps({'status':self.report['status'],'directory':str(self.run),
                          'error':self.report['error'],'shutdown_errors':self.report['shutdown_errors'],
                          'finalize_error':self.report.get('finalize_error')},ensure_ascii=False),flush=True)
        return 0 if self.report['status']=='complete' else 2


def main():
    parser=argparse.ArgumentParser(description='S1 键盘遥操实时建图 / 隔离录包回放 / 离线导出')
    parser.add_argument('command',nargs='?',choices=['run','finalize'],default='run')
    parser.add_argument('directory',nargs='?')
    parser.add_argument('--output-root',default=str(DATA))
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--replay',help='已有 bag 目录；隔离 Domain 125，不连接硬件')
    parser.add_argument('--replay-seconds',type=float,default=0,help='回放验证时间上限，0 为完整录包')
    from tg_teleop import speed, MAX_LINEAR_SPEED
    parser.add_argument('--linear-speed',type=lambda x:speed(x,.01,MAX_LINEAR_SPEED),default=.05)
    parser.add_argument('--angular-speed',type=lambda x:speed(x,.025,.25),default=.10)
    parser.add_argument('--slice-offset',type=float,default=None,help='切片相对前雷达初始中心的高度偏移，默认 0 m')
    parser.add_argument('--slice-thickness',type=float,default=None,help='水平切片厚度，默认 0.10 m')
    parser.add_argument('--task',default='',help='本次驾驶/采集任务描述')
    parser.add_argument('--mapping-only',action='store_true',help='仅原有雷达建图，不采集相机和机器人状态')
    parser.add_argument('--torso-speed',type=lambda x:speed(x,.001,.02),default=.02)
    parser.add_argument('--export-pcd',action='store_true',help='另外保存三维 PCD')
    parser.add_argument('--output-directory',help='finalize 的新输出目录，不覆盖已有目录')
    args=parser.parse_args()
    from mapping_slice import defaults
    defaults(0. if args.slice_offset is None else args.slice_offset,.1 if args.slice_thickness is None else args.slice_thickness)
    if not 1<=args.port<=65535: parser.error('Invalid port')
    if args.replay_seconds and (not args.replay or not 15<=args.replay_seconds<=86400):
        parser.error('--replay-seconds requires --replay and 15..86400 seconds')
    if args.command=='finalize':
        if not args.directory: parser.error('finalize requires a session directory')
        from mapping_slice_finalize import finalize_slice
        print(json.dumps(finalize_slice(args.directory,output=args.output_directory,offset=args.slice_offset,
            thickness=args.slice_thickness,export_pcd=args.export_pcd),ensure_ascii=False)); return 0
    if args.directory: parser.error('Unexpected session directory')
    if args.output_directory: parser.error('--output-directory is only for finalize')
    if args.slice_offset is None: args.slice_offset=0.
    if args.slice_thickness is None: args.slice_thickness=.1
    # Separate lock per ROS domain; no controller or sensor is touched by locking.
    with open('/tmp/towergo-mapping-%d-%s.lock'%(os.getuid(),os.environ.get('ROS_DOMAIN_ID','unset')),'a') as lock:
        try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError: raise RuntimeError('Another mapping session is running') from None
        return Session(args).execute()


if __name__=='__main__':
    try: sys.exit(main())
    except Exception as exc:
        print('ERROR: '+str(exc),file=sys.stderr); sys.exit(2)
