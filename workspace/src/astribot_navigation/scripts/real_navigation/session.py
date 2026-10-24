"""Owned process supervisor for live observation, validated driving, and bag replay."""
import argparse
import fcntl
import copy
import datetime
import json
import os
from pathlib import Path
import signal
import sys
import time
import uuid
import yaml
from .core import validate_config,digest

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
WORKSPACE=REPO.parents[1]


def nav_params(config,mapfile,replay):
    base=yaml.safe_load((REPO/'astribot_nav_bringup/config/nav2.yaml').read_text())
    web=yaml.safe_load((REPO/'astribot_nav_bringup/config/web_controller.yaml').read_text())
    for name in ('planner_server','controller_server'):
        base[name]['ros__parameters'].update(web[name]['ros__parameters'])
    def rewrite(obj):
        if isinstance(obj,dict):
            for key,value in obj.items():
                if key=='use_sim_time':obj[key]=replay
                elif isinstance(value,str):
                    obj[key]={'base_link':'tg_nav_base','odom':'tg_nav_odom','map':'tg_nav_map','/scan':'/towergo/nav/scan','/odom':'/towergo/nav/odom'}.get(value,value)
                else:rewrite(value)
        elif isinstance(obj,list):
            for value in obj:rewrite(value)
    rewrite(base)
    base['map_server']['ros__parameters'].update(yaml_filename=str(mapfile),frame_id='tg_nav_map')
    base['amcl']['ros__parameters'].update(update_min_d=config.get('amcl_update_min_d',0.),update_min_a=config.get('amcl_update_min_a',0.),transform_tolerance=.1)
    cp=base['controller_server']['ros__parameters'];cp['FollowPath']['desired_linear_vel']=.05
    cp['FollowPath']['max_allowed_time_to_collision_up_to_carrot']=config.get('collision_time_horizon',1.5)
    cp['odom_topic']='/towergo/nav/odom'
    for name in ('global_costmap','local_costmap'):
        p=base[name][name]['ros__parameters']
        p['footprint']=json.dumps(config['footprint']);p['footprint_padding']=config['footprint_padding']
        p['track_unknown_space']=True
        # User-selected static-map navigation. Live scans remain localization/preview only.
        p['plugins']=['static_layer','inflation_layer']
        p.pop('obstacle_layer',None)
        p.pop('voxel_layer',None)
        p['static_layer']={'plugin':'nav2_costmap_2d::StaticLayer','map_topic':'/towergo/nav/map','map_subscribe_transient_local':True}
        p['inflation_layer'].update(inflation_radius=float(config.get('inflation_radius',.2)),cost_scaling_factor=5.)
    bt=base['bt_navigator']['ros__parameters']
    bt['default_nav_through_poses_bt_xml']=str(REPO/'astribot_nav_bringup/config/navigation_through.xml')
    bt['default_nav_to_pose_bt_xml']=str(HERE/'navigation.xml')
    bt['plugin_lib_names']=['nav2_compute_path_through_poses_action_bt_node','nav2_compute_path_to_pose_action_bt_node','nav2_follow_path_action_bt_node','nav2_pipeline_sequence_bt_node',
        'nav2_rate_controller_bt_node','nav2_recovery_node_bt_node','nav2_wait_action_bt_node']
    # Namespace parameters match all node fully-qualified names, including nested costmaps.
    return {'towergo':{'nav':base}}


def start_live_sensors(node, replay, pump):
    from mapping_support import REQUIRED_RAW_TOPICS
    from tg_mapping import lidar_control
    if replay:
        return 'replay'
    # Same startup policy as keyboard SLAM: reuse existing drivers;
    # never toggle them merely because one input is temporarily missing.
    if any(node.count_publishers(topic)>0 for topic in REQUIRED_RAW_TOPICS):
        return 'existing'
    print('正在开启双雷达及前 IMU…', flush=True)
    message=lidar_control(node, True, pump=pump)
    print('传感器服务：'+message+'；等待实际数据', flush=True)
    return 'requested'


def parser():
    p=argparse.ArgumentParser(description='S1 真机网页导航，默认只观察，不加载 SDK')
    p.add_argument('--map',default=str(WORKSPACE/'data/maps/web_sim_manual_20261021_v4/map.yaml'))
    p.add_argument('--config',default=str(HERE/'navigation.yaml'))
    p.add_argument('--mode',choices=['observe','hardware','mock'],default='observe')
    p.add_argument('--port',type=int,default=8767)
    p.add_argument('--replay',help='原始双雷达/IMU bag；隔离 Domain 125，不加载 SDK')
    p.add_argument('--replay-rate',type=float,default=1.,help='隔离回放倍率（0.5–1）')
    p.add_argument('--duration',type=float,default=0,help='验证会话最长秒数；0 为持续运行')
    p.add_argument('--output-root',default=str(WORKSPACE/'data/reports/real_navigation'))
    p.add_argument('--diagnostic-seconds',type=float,default=600.,help='完整诊断录包时长；0 禁用，默认 600 秒')
    p.add_argument('--diagnostic-max-bytes',type=int,default=2147483648,help='诊断录包总大小上限（默认 2 GiB）')
    p.add_argument('--check',action='store_true',help='检查配置和依赖，不启动节点')
    return p


def main():
    args=parser().parse_args()
    if not .5<=args.replay_rate<=1:raise ValueError('回放倍率必须在 0.5–1；更慢会超出逐帧等待期限')
    if args.diagnostic_seconds<0 or args.diagnostic_max_bytes<0:raise ValueError('诊断记录限制必须非负')
    config=yaml.safe_load(Path(args.config).read_text())
    validate_config(config,args.mode=='hardware')
    inflation=float(config.get('inflation_radius',.2))
    if not 0<=inflation<=2:raise ValueError('膨胀半径必须在 0–2 m')
    if args.mode=='mock':
        if not args.replay:raise ValueError('mock 仅用于隔离录包回放')
        config['max_speed']=.3
    if args.replay and args.mode=='hardware':raise ValueError('回放禁止硬件后端')
    mapfile=Path(args.map).resolve()
    m=yaml.safe_load(mapfile.read_text());image=mapfile.parent/m['image']
    if abs(m['resolution']-.05)>1e-8:raise ValueError('当前 lattice 配置要求 0.05 m 地图')
    from ament_index_python.packages import get_package_prefix,get_package_share_directory
    dependencies=['fast_lio','towergo_dual_livox','astribot_nav_bringup','nav2_amcl','nav2_regulated_pure_pursuit_controller']
    for package in dependencies:get_package_prefix(package)
    if args.check:
        print(json.dumps(dict(mode=args.mode,map=str(mapfile),max_speed=config['max_speed'],dependencies=dependencies),ensure_ascii=False,indent=2));return
    if os.environ.get('ROS_DOMAIN_ID')!=('125' if args.replay else '25'):raise ValueError('错误的 ROS Domain，请使用 tg-navigation.sh')
    session_lock=open('/tmp/towergo-navigation-%s-%s.lock'%(os.getuid(),os.environ['ROS_DOMAIN_ID']),'a')
    try:fcntl.flock(session_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise RuntimeError('已有导航会话；未启动节点或发送停止指令')
    from mapping_support import OwnedProcess,atomic_json,REQUIRED_RAW_TOPICS
    from tg_mapping import parameters,topic_names
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import SingleThreadedExecutor
    from .web import Web,server
    run=Path(args.output_root)/('navigation_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    run.mkdir(parents=True)
    (run/'config.yaml').write_text(yaml.safe_dump(config));(run/'nav2.yaml').write_text(yaml.safe_dump(nav_params(config,mapfile,bool(args.replay))))
    report=dict(mode=args.mode,replay=args.replay,map=str(mapfile),map_sha256=digest(mapfile),image_sha256=digest(image),
                config_sha256=digest(run/'config.yaml'),max_speed=config['max_speed'],started=time.time(),status='starting',processes={})
    procs={};node=None;http=None;executor=None;interrupted=False
    def spawn(name,cmd):
        proc=OwnedProcess(name,cmd,log=run/(name+'.log'))
        procs[name]=proc;report['processes'][name]={'pid':proc.proc.pid,'command':cmd}
        return proc
    def halt(sig,frame):
        nonlocal interrupted
        interrupted=True
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,halt)
    rclpy.init(args=['--ros-args','-r','/tf:=/towergo/nav/tf','-r','/tf_static:=/towergo/nav/tf_static',
                      '-p','use_sim_time:='+str(bool(args.replay)).lower()],signal_handler_options=SignalHandlerOptions.NO)
    try:
        node=Web(config,str(mapfile),args.mode,run)
        # Keep the node attached: rclpy.spin_once(node) removes/re-adds it
        # on every callback and repeatedly rebuilds this large wait set.
        executor=SingleThreadedExecutor();executor.add_node(node)
        http=server(node,args.port)  # Bind before starting any owned processes.
        # Discover existing publishers before reusing the keyboard SLAM sensor startup.
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.05)
        if node.count_publishers('/towergo/nav/odom')>1 or node.count_publishers('/towergo/nav/lio/odometry'):
            raise RuntimeError('已有真机导航会话，请先结束原会话')
        if args.replay and (node.count_publishers('/clock') or any(node.count_publishers(t) for t in REQUIRED_RAW_TOPICS)):
            raise RuntimeError('回放 Domain 已有传感器或时钟发布者')
        report['sensor_startup']=start_live_sensors(
            node, bool(args.replay), lambda: executor.spin_once(timeout_sec=.05))
        # Device service success means requested, not that sensor data is ready.
        # The existing live-data health checks determine readiness in the UI.
        topics=topic_names('/towergo/nav/lio')
        base=Path(get_package_share_directory('fast_lio'))/'config/mid360.yaml'
        lio_parameters=parameters(base,topics,bool(args.replay))
        # Navigation derives both obstacle clouds directly from raw sensors.
        # The registered visualization cloud is unused; avoid transforming and
        # serializing it on FAST-LIO's real-time estimation thread.
        lio_parameters['/**']['ros__parameters']['publish']['scan_publish_en']=False
        (run/'lio.yaml').write_text(yaml.safe_dump(lio_parameters))
        sim=str(bool(args.replay)).lower()
        adapter=Path(get_package_prefix('towergo_dual_livox'))/'lib/towergo_dual_livox/dual_adapter'
        cmd=[str(adapter),'--ros-args','-p','imu_acceleration_unit:=g','-p','use_sim_time:='+sim,
             '-p','drop_invalid_timestamp_frames:=true',
             '-p','coordinate_contract:=both_raw_topics_already_in_front_lidar_frame','-p','reliable_input:='+sim,
             '-r','__node:=navigation_dual_adapter']
        for old,new in {'/towergo/lio/lidar':topics['lidar'],'/towergo/lio/imu':topics['imu'],'/towergo/mapping_fault':topics['fault'],'/diagnostics':topics['diagnostics']}.items():cmd+=['-r',old+':='+new]
        spawn('adapter',cmd)
        fast=Path(get_package_prefix('fast_lio'))/'lib/fast_lio/fastlio_mapping'
        cmd=[str(fast),'--ros-args','--params-file',str(run/'lio.yaml'),'-r','__node:=navigation_fastlio']
        for old,key in [('/Odometry','odom'),('/cloud_registered','cloud'),('/tf','tf'),('/tf_static','tf_static')]:cmd+=['-r',old+':='+topics[key]]
        spawn('fastlio',cmd)
        spawn('nav2',['ros2','launch',str(HERE/'launch.py'),'params:='+str(run/'nav2.yaml')])
        if args.mode in ('hardware','mock'):spawn('executor',[sys.executable,'-m','real_navigation.executor',str(run/'config.yaml'),args.mode])
        if args.diagnostic_seconds and args.diagnostic_max_bytes:
            diagnostic_topics=[*REQUIRED_RAW_TOPICS,'/towergo/nav/lio/odometry','/towergo/nav/odom',
                '/towergo/nav/scan','/towergo/nav/amcl_pose','/towergo/nav/tf','/towergo/nav/tf_static',
                '/towergo/nav/executor_status','/towergo/nav/executor_control','/towergo/nav/cmd_vel_nav',
                '/astribot_chassis/joint_space_states','/towergo/nav/map','/clock']
            qos={t:dict(reliability='best_effort',durability='volatile',history='keep_last',depth=10)
                 for t in diagnostic_topics}
            for t in ('/towergo/nav/tf_static','/towergo/nav/map'):
                qos[t]=dict(reliability='reliable',durability='transient_local',history='keep_last',depth=1)
            (run/'diagnostic_qos.yaml').write_text(yaml.safe_dump(qos))
            spawn('diagnostic_recorder',['ros2','bag','record','-s','mcap','-o',str(run/'diagnostic_bag'),
                '--qos-profile-overrides-path',str(run/'diagnostic_qos.yaml'),
                '--max-bag-size','268435456','--max-cache-size','67108864',*diagnostic_topics])
            report['diagnostics']=dict(status='recording',seconds=args.diagnostic_seconds,max_bytes=args.diagnostic_max_bytes)
        if args.replay:
            spawn('player',['ros2','bag','play',str(Path(args.replay).resolve()),'--clock','--rate',str(args.replay_rate),'--delay','2','--topics',*REQUIRED_RAW_TOPICS])
        print('真机导航 '+args.mode+'：http://127.0.0.1:'+str(args.port)+'\n记录目录：'+str(run),flush=True)
        report['status']='running';atomic_json(run/'session.json',report)
        diagnostic_check=0.;diagnostic_stopping=False
        start=time.monotonic();maxima={'front':0,'back':0,'scan':0};ready_seen=False
        while rclpy.ok() and not interrupted and (not args.duration or time.monotonic()-start<args.duration):
            executor.spin_once(timeout_sec=.02)
            if 'diagnostic_recorder' in procs and not diagnostic_stopping and time.monotonic()-diagnostic_check>=1:
                diagnostic_check=time.monotonic()
                size=sum(p.stat().st_size for p in (run/'diagnostic_bag').glob('*') if p.is_file())
                if time.monotonic()-start>=args.diagnostic_seconds or size>=args.diagnostic_max_bytes:
                    proc=procs['diagnostic_recorder'].proc
                    if proc.poll() is None:proc.send_signal(signal.SIGINT)
                    diagnostic_stopping=True;report['diagnostics'].update(status='bounded_stop',bytes=size)
            for name,p in procs.items():
                if p.poll() is not None:
                    if name=='diagnostic_recorder':
                        report['diagnostics']['exit_code']=p.poll()
                        if not diagnostic_stopping:report['diagnostics']['status']='recorder_exited'
                        continue
                    if name=='player' and p.poll()==0:interrupted=True;break
                    log_path = run / (name + '.log')
                    try:
                        tail = '\n'.join(log_path.read_text(errors='replace').splitlines()[-15:])
                    except OSError:
                        tail = '(无法读取日志)'
                    raise RuntimeError(name + ' 进程退出: ' + str(p.poll()) +
                                       '\n日志: ' + str(log_path) + '\n' + tail)
            ready_seen=ready_seen or node.sensors.healthy()
            for key in maxima:
                if key in node.sensors.received:maxima[key]+=1
        report.update(status='complete',sensor_ready_seen=ready_seen,sensor_observation_counts=maxima,
                      localization_initialized=node.state['initialized'],navigation_ready=node.state['ready'],
                      last_state=node.state,validation='observation/replay only; no hardware acceptance')
    except Exception as exc:
        report.update(status='failed',error=str(exc));raise
    finally:
        if node:
            node.stop('会话结束')
            end=time.monotonic()+.3
            while rclpy.ok() and time.monotonic()<end:executor.spin_once(timeout_sec=.02)
        for name in ('diagnostic_recorder','executor','player','nav2','fastlio','adapter'):
            if name in procs:
                try:procs[name].stop()
                except Exception as exc:report.setdefault('shutdown_errors',[]).append(str(exc))
        if http:http.shutdown();http.server_close()
        if executor:executor.shutdown()
        if node:node.quality.close();node.sensors.close();node.log.close();node.destroy_node()
        rclpy.try_shutdown();report['ended']=time.time();atomic_json(run/'session.json',report)

if __name__=='__main__':main()
