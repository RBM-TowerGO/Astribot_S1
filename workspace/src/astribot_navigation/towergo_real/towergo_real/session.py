"""Raw recording, isolated replay, map save/restore checks. Never controls chassis."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
import yaml
from .audit import SENSORS, audit_bag
from .contracts import calibration


def stop_process(process):
    if process is None:
        return
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
            raise RuntimeError('Process required SIGTERM; recording/session is not clean')
    if process.returncode not in (0, -signal.SIGINT, 128+signal.SIGINT):
        raise RuntimeError('Child process exited abnormally: '+str(process.returncode))


def stop_all(processes):
    errors=[]
    for process in processes:
        try:stop_process(process)
        except Exception as exc:errors.append(str(exc))
    if errors:raise RuntimeError('; '.join(errors))


class Observer:
    def __init__(self):
        import rclpy
        from sensor_msgs.msg import PointCloud2, Imu
        from nav_msgs.msg import OccupancyGrid, Path as RosPath
        from std_msgs.msg import Bool
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
        rclpy.init();self.node=rclpy.create_node('towergo_session_'+str(os.getpid()))
        self.counts={t:0 for t in SENSORS};self.mapping_fault=False;self.map=None;self.trajectory=None
        def receive(topic):self.counts[topic]+=1
        for t in SENSORS:
            self.node.create_subscription(Imu if '/imu_' in t else PointCloud2,t,lambda m,t=t:receive(t),qos_profile_sensor_data)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Bool,'/towergo/mapping_fault',lambda m:setattr(self,'mapping_fault',self.mapping_fault or m.data),latched)
        self.node.create_subscription(OccupancyGrid,'/map',lambda m:setattr(self,'map',m),latched)
        self.node.create_subscription(RosPath,'/towergo/trajectory',lambda m:setattr(self,'trajectory',m),1)
        self.thread=threading.Thread(target=rclpy.spin,args=(self.node,),daemon=True);self.thread.start()

    def call(self,kind,name,request,timeout=15):
        client=self.node.create_client(kind,name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):raise RuntimeError('Service unavailable: '+name)
            future=client.call_async(request);end=time.monotonic()+timeout
            while not future.done() and time.monotonic()<end:time.sleep(.02)
            if not future.done():raise TimeoutError(name)
            return future.result()
        finally:self.node.destroy_client(client)

    def close(self):
        import rclpy
        if rclpy.ok():rclpy.shutdown()
        self.thread.join(timeout=3);self.node.destroy_node()


def write_json(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,ensure_ascii=False)


def create_run(data_root,kind):
    ident=kind+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
    root=Path(data_root).resolve();run=root/'reports'/ident;run.mkdir(parents=True,exist_ok=False)
    provenance={'run_id':ident,'kind':kind,'time':datetime.datetime.now().astimezone().isoformat(),
                'ros_domain_id':os.environ.get('ROS_DOMAIN_ID'),'chassis_commands_sent':False,
                'image_id':os.environ.get('TOWERGO_IMAGE_ID')}
    repo=Path(__file__).resolve()
    for parent in repo.parents:
        if (parent/'.git').exists():
            provenance['git_commit']=subprocess.check_output(['git','-C',str(parent),'rev-parse','HEAD'],text=True).strip()
            provenance['git_status']=subprocess.check_output(['git','-C',str(parent),'status','--short'],text=True)
            for name in ('source_versions.json','dependency-versions.txt'):
                version=parent/'runtime_ws'/name
                if version.is_file():(run/name).write_bytes(version.read_bytes())
            break
    provenance['python_source_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                                      for p in repo.parent.glob('*.py')}
    for source,name in [(Path('/real_ws/dependency-versions.txt'),'dependency-versions.txt'),
                        (Path('/real_ws/src/dependencies/source_versions.json'),'source_versions.json')]:
        if source.is_file():(run/name).write_bytes(source.read_bytes())
    write_json(run/'provenance.json',provenance)
    return root,run,ident


def record(args):
    from std_srvs.srv import SetBool
    if os.environ.get('ROS_DOMAIN_ID')!='25':raise RuntimeError('Real recording requires Domain 25')
    if not os.isatty(0):raise RuntimeError('Interactive terminal required to confirm physical stop before ending capture')
    root,run,ident=create_run(args.data_root,'keyboard_slam')
    bag=root/'bags'/ident/'bag';bag.parent.mkdir(parents=True,exist_ok=False)
    obs=Observer();owned=False;proc=None;log=None;error=None
    report={'operator':args.operator,'bag':str(bag),'driver_started_by_this_run':False,'physical_stop_confirmed':False}
    try:
        time.sleep(2)
        existing=any(obs.node.count_publishers(t)>0 for t in SENSORS)
        if not existing:
            req=SetBool.Request();req.data=True
            response=obs.call(SetBool,'/astribot_lidar_control',req)
            if not response.success:raise RuntimeError(response.message)
            owned=True;report['driver_started_by_this_run']=True
        end=time.monotonic()+30
        while not all(obs.counts.values()):
            if time.monotonic()>end:raise TimeoutError('Four live sensor streams did not arrive')
            time.sleep(.1)
        free=os.statvfs(str(root));free_bytes=free.f_bavail*free.f_frsize
        if free_bytes<5*1024**3:raise RuntimeError('At least 5 GiB free space required')
        topics=SENSORS+['/astribot_chassis/joint_space_states','/astribot_chassis/endpoint_current_states',
                        '/odom','/scan','/tf','/tf_static','/towergo/control_status','/diagnostics',
                        '/towergo/lio/imu','/towergo/lio/lidar','/towergo/lio/odometry_raw',
                        '/towergo/lio/cloud_body','/towergo/trajectory','/towergo/mapping_fault']
        qos={t:{'reliability':'best_effort','durability':'volatile','history':'keep_last','depth':1000} for t in topics}
        qos['/tf_static']={'reliability':'reliable','durability':'transient_local','history':'keep_last','depth':100}
        (run/'record_qos.yaml').write_text(yaml.safe_dump(qos))
        from ament_index_python.packages import get_package_share_directory
        configs=Path(get_package_share_directory('towergo_real'))/'config'
        for key in ('calibration','hardware_contract'):
            selected=getattr(args,key)
            source=Path(selected) if selected else configs/('calibration.unverified.yaml' if key=='calibration' else 'hardware.unverified.yaml')
            (run/(key+'.yaml')).write_bytes(source.read_bytes())
            report[key+'_source']=str(source.resolve())
        driver=Path('/opt/astribot_ros/software/livox_ros_driver2/share')
        for src,name in [(driver/'config/MID360_config.json','MID360_config.json'),
                         (driver/'livox_ros_driver2/msg_MID360_launch_rviz_msg_type.py','driver_launch.py')]:
            (run/name).write_bytes(src.read_bytes())
        log=(run/'record.log').open('w')
        proc=subprocess.Popen(['ros2','bag','record','-s','mcap','-o',str(bag),'--qos-profile-overrides-path',str(run/'record_qos.yaml'),*topics],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        print(f'Run: {ident}\nBag: {bag}\n先静止 10 秒，再在独立终端驾驶。停车后再静止 10 秒。',flush=True)
        while True:
            if proc.poll() is not None:raise RuntimeError('Recorder exited; inspect record.log')
            answer=input('确认机器人已物理停止后输入 stopped 结束录包：').strip()
            if answer=='stopped':report['physical_stop_confirmed']=True;break
        stop_process(proc);proc=None
        report['audit']=audit_bag(bag)
        if not report['audit']['passed']:raise RuntimeError('Recorded input audit failed')
    except BaseException as exc:
        error=exc;report['error']=str(exc)
        print('录包操作异常不代表底盘已停车；请使用独立键盘停止/现场急停。',flush=True)
    finally:
        try:stop_process(proc)
        except Exception as exc:report['record_shutdown_error']=str(exc)
        if owned:
            # Do not stop a driver now used by a running live mapping pipeline.
            nodes=[name for name,_ in obs.node.get_node_names_and_namespaces()]
            if 'towergo_fast_lio' not in nodes:
                req=SetBool.Request();req.data=False
                try:report['driver_shutdown_success']=obs.call(SetBool,'/astribot_lidar_control',req).success
                except Exception as exc:report['driver_shutdown_error']=str(exc)
            else:report['driver_left_for_live_mapping']=True
        if log:log.close()
        obs.close();write_json(run/'capture_report.json',report)
    if error:raise error
    print(str(run))


def save_map(obs,output,sim=False):
    from slam_toolbox.srv import SerializePoseGraph
    if obs.mapping_fault:raise RuntimeError('Cannot accept a faulted mapping session')
    if obs.map is None or not obs.map.data:raise RuntimeError('No map received')
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    subprocess.run(['ros2','run','nav2_map_server','map_saver_cli','-f',str(output/'map'),
                    '--ros-args','-p',f'use_sim_time:={str(sim).lower()}','-p','save_map_timeout:=10.0',
                    '-p','free_thresh_default:=0.196','-p','occupied_thresh_default:=0.65'],check=True,timeout=30)
    request=SerializePoseGraph.Request();request.filename=str(output/'posegraph')
    result=obs.call(SerializePoseGraph,'/slam_toolbox/serialize_map',request)
    if result.result!=0:raise RuntimeError('Posegraph serialization failed')
    if obs.trajectory is not None:
        from rosidl_runtime_py.convert import message_to_ordereddict
        write_json(output/'trajectory.json',message_to_ordereddict(obs.trajectory))
    for name in ('map.yaml','map.pgm','posegraph.posegraph','posegraph.data'):
        if not (output/name).is_file() or (output/name).stat().st_size==0:raise RuntimeError('Missing map output: '+name)
    from PIL import Image
    config=yaml.safe_load((output/'map.yaml').read_text());img=Image.open(output/config['image']);img.load()
    report={'files':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir() if p.is_file()},
            'dimensions':list(img.size),'resolution':config['resolution'],'ros_reload_verified':False,
            'field_accuracy_verified':False,'note':'File validation; run verify-map for ROS reload/posegraph restore.'}
    write_json(output/'save_report.json',report)
    return report


def replay(args):
    if os.environ.get('ROS_DOMAIN_ID')!='125' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Replay requires isolated Domain 125 and ROS_LOCALHOST_ONLY=1')
    if not Path('/.dockerenv').exists() and not args.allow_host_localhost_test:
        raise RuntimeError('Use the isolated container; host mode is only an explicit local test')
    calibration(args.calibration)
    report=audit_bag(args.bag)
    if not report['passed']:raise RuntimeError('Input audit failed: '+str(report['errors']))
    root,run,ident=create_run(args.data_root,'replay_slam');write_json(run/'input_audit.json',report)
    (run/'calibration.yaml').write_bytes(Path(args.calibration).read_bytes())
    obs=Observer();mapping=player=None
    with (run/'mapping.log').open('w') as log:
        try:
            mapping=subprocess.Popen(['ros2','launch','towergo_real','real_mapping.launch.py',
                'offline:=true','viewer:=false','calibration:='+str(Path(args.calibration).resolve())],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+30
            while not (obs.node.count_subscribers('/livox/lidar_front')>=2 and obs.node.count_subscribers('/towergo/lio/lidar')>=1):
                if mapping.poll() is not None:raise RuntimeError('Mapping startup failed')
                if time.monotonic()>deadline:raise TimeoutError('Mapping subscribers not ready')
                time.sleep(.1)
            player=subprocess.Popen(['ros2','bag','play',str(args.bag),'--clock','--rate',str(args.rate),'--topics',*SENSORS],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            while player.poll() is None:
                if mapping.poll() is not None or obs.mapping_fault:raise RuntimeError('Mapping pipeline stopped/faulted')
                time.sleep(.1)
            if player.returncode:raise RuntimeError('Bag playback failed')
            time.sleep(2)
            result=save_map(obs,root/'maps'/ident,sim=True)
            write_json(run/'result.json',result)
        finally:
            try:stop_all([player,mapping])
            finally:obs.close()
    print(str(run))


def verify_map(args):
    """Restore into private topics: no historical TF/control reaches live nodes."""
    from nav2_msgs.srv import LoadMap
    from lifecycle_msgs.srv import ChangeState
    from slam_toolbox.srv import DeserializePoseGraph
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile, DurabilityPolicy
    output=Path(args.output).resolve();obs=Observer();processes=[];maps={};logs=[]
    ns='/towergo_verify_'+str(os.getpid())
    def spawn(command):
        log=(output/('reload_process_'+str(len(processes))+'.log')).open('a');logs.append(log)
        p=subprocess.Popen(command,start_new_session=True,stdout=log,stderr=subprocess.STDOUT)
        processes.append(p);return p
    try:
        spawn(['ros2','run','nav2_map_server','map_server','--ros-args','-r','__ns:='+ns,
               '-p','yaml_filename:='+str(output/'map.yaml')])
        req=ChangeState.Request();req.transition.id=1
        if not obs.call(ChangeState,ns+'/map_server/change_state',req).success:raise RuntimeError('Map server configure failed')
        req.transition.id=3
        if not obs.call(ChangeState,ns+'/map_server/change_state',req).success:raise RuntimeError('Map server activate failed')
        request=LoadMap.Request();request.map_url=str(output/'map.yaml')
        loaded=obs.call(LoadMap,ns+'/map_server/load_map',request)
        if loaded.result!=0 or not loaded.map.data:raise RuntimeError('Map reload failed: result='+str(loaded.result))
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        obs.node.create_subscription(OccupancyGrid,ns+'/restored_map',lambda m:maps.update(restored=m),q)
        spawn(['ros2','run','slam_toolbox','sync_slam_toolbox_node','--ros-args','-r','__ns:='+ns,
               '-r','map:='+ns+'/restored_map','-r','/map:='+ns+'/restored_map',
               '-r','/tf:='+ns+'/tf','-r','/tf_static:='+ns+'/tf_static','-p','scan_topic:='+ns+'/unused_scan'])
        req=DeserializePoseGraph.Request();req.filename=str(output/'posegraph');req.match_type=1
        obs.call(DeserializePoseGraph,ns+'/slam_toolbox/deserialize_map',req)
        deadline=time.monotonic()+15
        while 'restored' not in maps:
            if time.monotonic()>deadline:raise TimeoutError('Restored posegraph did not publish a map')
            time.sleep(.1)
        if not maps['restored'].data:raise RuntimeError('Empty restored map')
        write_json(output/('reload_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.json'),
                   {'map_reload':True,'posegraph_restore':True,'navigation_ready':False})
    finally:
        try:stop_all(reversed(processes))
        finally:
            for log in logs:log.close()
            obs.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('record');p.add_argument('--data-root',required=True);p.add_argument('--operator',required=True)
    p.add_argument('--calibration');p.add_argument('--hardware-contract')
    p=sub.add_parser('replay');p.add_argument('--bag',required=True);p.add_argument('--calibration',required=True)
    p.add_argument('--data-root',required=True);p.add_argument('--rate',type=float,default=.5)
    p.add_argument('--allow-host-localhost-test',action='store_true')
    p=sub.add_parser('save');p.add_argument('--output',required=True);p.add_argument('--physical-stop-confirmed',action='store_true',required=True)
    p=sub.add_parser('verify-map');p.add_argument('--output',required=True)
    args=parser.parse_args()
    if args.command=='record':record(args)
    elif args.command=='replay':
        if not 0<args.rate<=1:parser.error('Replay rate must be in (0,1]')
        replay(args)
    elif args.command=='verify-map':verify_map(args)
    else:
        obs=Observer()
        try:
            deadline=time.monotonic()+10
            while obs.map is None and time.monotonic()<deadline:time.sleep(.1)
            print(json.dumps(save_map(obs,Path(args.output).resolve()),indent=2))
        finally:obs.close()


if __name__=='__main__':main()
