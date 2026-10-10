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
                pts=cloud_xyz(m)
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
        self.health_write=None; self.signal=None; self.owned_lidar=False
        self.last_slice_read=0.
        self.report={'run_id':self.run_id,'status':'initializing','error':None,'topics':self.topics,
                     'replay':self.replay,'hardware_commands_sent':False,'lidar_started':False,
                     'navigation_map':True,'shutdown_errors':[],
                     'lidar_mode':'dual','imu_source':'front',
                     'fusion_parameters':{'window_ns':100000000,'max_buffer_ns':1000000000,'lines':8,
                                          'rear_transform_applied_by_adapter':False}}

    def spawn(self,name,cmd,tty=False,pass_fds=()):
        p=OwnedProcess(name,cmd,log=None if tty else self.run/(name+'.log'),tty=tty,pass_fds=pass_fds)
        self.procs[name]=p
        self.report.setdefault('processes',{})[name]={'pid':p.proc.pid,'command':cmd}
        return p

    def heartbeat(self):
        if self.health_write is not None:
            try: os.write(self.health_write,struct.pack('!d',time.monotonic()))
            except (BrokenPipeError,BlockingIOError): pass

    def revoke(self):
        if self.health_write is not None:
            os.close(self.health_write); self.health_write=None

    def check(self,require_streams=False,allow_player_end=False):
        if self.signal: raise InterruptedError('Session interrupted by signal %s'%self.signal)
        for name,p in self.procs.items():
            if name=='teleop' or (name=='player' and allow_player_end): continue
            if p.poll() is not None: raise RuntimeError('%s exited (%s)'%(name,p.poll()))
        free=shutil.disk_usage(self.run).free/1024**3
        self.state.free_gib=round(free,2)
        if free<3: raise RuntimeError('Less than 3 GiB free disk space')
        if 'slice' in self.procs and time.monotonic()-self.last_slice_read>=.5:
            self.last_slice_read=time.monotonic()
            try:
                data=json.loads((self.run/'slice_live.json').read_text())
                with self.state.lock: self.state.slice=data
                if data.get('error'): raise RuntimeError('Slice worker: '+data['error'])
                if require_streams and self.state.stamps.get('odom',0)-data.get('stamp_ns',0)>2000000000:
                    raise RuntimeError('Slice observations lag LIO by more than 2 seconds')
                if require_streams and time.time()-data['worker_updated']>3:
                    raise RuntimeError('Slice worker has not updated for 3 seconds')
            except FileNotFoundError:
                if require_streams: raise RuntimeError('Slice worker has no status')
        if self.state.error: raise RuntimeError(self.state.error)
        if require_streams and not self.state.healthy():
            raise RuntimeError('Sensor/LIO stream stale for more than 2 seconds')

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
                    and self.state.slice.get('statistics',{}).get('frames',0)>0): return
            time.sleep(.1)
        raise RuntimeError('Initialization timed out (60 seconds); no driving enabled')

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
        try:
            child=self.spawn('teleop',[str(HERE/'tg-teleop.sh'),'--session-fd',str(read_fd),
                                      '--linear-speed',str(self.args.linear_speed),
                                      '--angular-speed',str(self.args.angular_speed)],tty=True,pass_fds=(read_fd,))
        finally: os.close(read_fd)
        self.report['teleop_started']=True
        self.report['hardware_commands_sent']=None  # SDK motion depends on keyboard input; not inferred here.
        atomic_json(self.run/'session.json',self.report)
        while child.poll() is None:
            self.check(require_streams=True); self.heartbeat(); time.sleep(.1)
        self.revoke()
        if child.poll()!=0: raise RuntimeError('Teleop exited abnormally: %s'%child.poll())
        self.state.status='sensor_tail'
        self.pause(5,require_streams=True)

    def cleanup(self):
        self.revoke()
        for name in shutdown_order(self.procs):
            try: self.procs[name].stop(timeout=30 if 'record' in name else 8)
            except Exception as exc: self.report['shutdown_errors'].append(str(exc))
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
            self.report['status']='recorded' if normal and not self.signal and not self.report['shutdown_errors'] else 'incomplete'
            if self.run.exists():
                try:
                    self.report['raw_counts']=check_metadata(self.run/'raw_bag',REQUIRED_RAW_TOPICS)
                    self.report['lio_counts']=check_metadata(self.run/'lio_bag',[self.topics['odom'],self.topics['cloud'],self.topics['occupancy']])
                except Exception as exc:
                    self.report['status']='incomplete'; self.report['recording_error']=str(exc)
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
                if self.run.exists(): atomic_json(self.run/'session.json',self.report)
                if self.server: self.server.shutdown(); self.server.server_close()
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
    from tg_teleop import speed
    parser.add_argument('--linear-speed',type=lambda x:speed(x,.01,.15),default=.05)
    parser.add_argument('--angular-speed',type=lambda x:speed(x,.025,.25),default=.10)
    parser.add_argument('--slice-offset',type=float,default=None,help='切片相对前雷达初始中心的高度偏移，默认 0 m')
    parser.add_argument('--slice-thickness',type=float,default=None,help='水平切片厚度，默认 0.10 m')
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
