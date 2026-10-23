import ast
import importlib.util
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace, ModuleType
import urllib.request
import urllib.error
import json
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import numpy as np
import pytest
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from real_navigation.geometry import Geometry

def geometry():
    g=Geometry(yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text()))
    for _ in range(100):g.imu([0,0,1])
    return g

def test_gap_preserves_origin_and_rebuilds_history():
    g=geometry();a=g.pose(1.,np.eye(4));origin=g.origin.copy()
    b=g.pose(2.,np.eye(4))
    assert g.continuity_epoch==1
    assert np.allclose(a,b) and np.allclose(origin,g.origin)
    assert g.at(1.5) is None
    g.pose(2.1,np.eye(4));assert g.at(2.05) is not None

def test_backwards_and_jumps_are_not_recovered_as_delays():
    g=geometry();g.pose(1.,np.eye(4))
    with pytest.raises(ValueError,match='时间倒退'):g.pose(.9,np.eye(4))
    t=np.eye(4);t[0,3]=1.
    with pytest.raises(ValueError,match='位姿跳变'):g.pose(2.,t)
    assert g.continuity_epoch==0

def test_old_lio_updates_history_but_not_health(monkeypatch):
    monkeypatch.setitem(sys.modules,'rclpy',ModuleType('rclpy'))
    name='real_navigation._sensor_recovery_test'
    spec=importlib.util.spec_from_file_location(name,ROOT/'scripts/real_navigation/sensors.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    s=module.Sensors.__new__(module.Sensors)
    s.frames={};s.stamps={};s.received={};s.error=None
    s.node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=2_000_000_000)))
    msg=SimpleNamespace(header=SimpleNamespace(frame_id='odom',stamp=SimpleNamespace(sec=1,nanosec=0)))
    assert s.seen('lio',msg,history=True)==1.
    assert 'lio' not in s.received
    msg.header.stamp.sec=2
    assert s.seen('lio',msg,history=True)==2.
    assert 'lio' in s.received

def web_ast():return ast.parse((ROOT/'scripts/real_navigation/web.py').read_text())

def test_stop_timeout_clears_on_confirmation_without_hiding_other_faults():
    cls=next(n for n in web_ast().body if isinstance(n,ast.ClassDef) and n.name=='Web')
    tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_tick')
    block=next(n for n in tick.body if isinstance(n,ast.If) and ast.unparse(n.test)=="self.phase == 'stopping'")
    for error,expected in [('未能确认停车，请检查现场状态',None),('底盘故障','底盘故障')]:
        obj=SimpleNamespace(phase='stopping',phase_at=0.,still_since=9.,result_pending='canceled',
            state={'error':error},sensors=SimpleNamespace(healthy=lambda:True,planar_speed=0.,velocity=[0.,0.]))
        stop_evidence_fixture(obj)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[block],type_ignores=[])),'stop-block','exec'),{'self':obj,'now':10.})
        assert obj.phase=='idle' and obj.state['error']==expected

def test_expired_page_token_returns_actionable_reason_without_enqueuing():
    fn=next(n for n in web_ast().body if isinstance(n,ast.FunctionDef) and n.name=='server')
    ns=dict(Path=Path,__file__=str(ROOT/'scripts/real_navigation/web.py'),BaseHTTPRequestHandler=BaseHTTPRequestHandler,
            ThreadingHTTPServer=ThreadingHTTPServer,threading=threading,json=json)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'server','exec'),ns)
    node=SimpleNamespace(token='new-session')
    server=ns['server'](node,0)
    try:
        req=urllib.request.Request('http://127.0.0.1:%d/api/localize'%server.server_port,data=b'{}',headers={'X-Session-Token':'old-session'})
        with pytest.raises(urllib.error.HTTPError) as result:urllib.request.urlopen(req)
        assert result.value.code==403
        assert '刷新' in json.loads(result.value.read())['error']
    finally:server.shutdown();server.server_close()

@pytest.mark.parametrize('active', [False,True])
def test_idle_cancel_does_not_claim_motion_or_wait_for_stop(active):
    import time
    cls=next(n for n in web_ast().body if isinstance(n,ast.ClassDef) and n.name=='Web')
    fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='stop')
    ns=dict(time=time)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'stop','exec'),ns)
    sent=[]
    lease=SimpleNamespace(active=active)
    def revoke():lease.active=False
    lease.stop=revoke
    node=SimpleNamespace(phase='running' if active else 'idle',lease=lease,state={'busy':active},goal_handle=None,
                         send=lambda op:sent.append(op),event=lambda _:None)
    ns['stop'](node)
    assert sent==['stop'] and not lease.active
    assert node.phase==('stopping' if active else 'idle')
    if not active:
        assert '没有活动导航任务' in node.state['status']
        assert node.state['error'] is None

def test_scan_endpoints_filter_missing_and_out_of_range():
    from real_navigation.geometry import scan_points
    points=scan_points([1.,float('nan'),float('inf'),21.,.1,2.,0.,1.])
    assert points.shape==(3,3)
    assert np.allclose(points[0],[-1,0,0])
    assert np.isfinite(points).all()

@pytest.mark.parametrize('age,missing_tf,expected,source_age',[(.1,False,1,None),(1.,False,0,None),(.1,True,0,None),(.1,False,0,.9),(.1,False,1,.535),(.1,False,1,.3)])
def test_scan_overlay_uses_timestamped_transform_or_hides(age,missing_tf,expected,source_age):
    from scipy.spatial.transform import Rotation
    cls=next(n for n in web_ast().body if isinstance(n,ast.ClassDef) and n.name=='Web')
    fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='update_obstacles')
    ns=dict(np=np,Rotation=Rotation,SCAN_MAX_SOURCE_AGE=.8,FRAME={'map':'map','front':'front'},
            rclpy=SimpleNamespace(time=SimpleNamespace(Time=lambda **kw:kw)))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'overlay','exec'),ns)
    calls=[]
    def lookup(a,b,stamp):
        calls.append((a,b,stamp))
        if missing_tf:raise RuntimeError('no transform')
        return SimpleNamespace(transform=SimpleNamespace(rotation=SimpleNamespace(x=0,y=0,z=0,w=1),translation=SimpleNamespace(x=2,y=3,z=0)))
    node=SimpleNamespace(sensors=SimpleNamespace(obstacles={},latest_scan=(10.-age,[[1.,0.,0.]]),error=None),state={},
        tf=SimpleNamespace(lookup_transform=lookup),get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)))
    if source_age is not None:node.sensors.latest_scan_source_stamp=10.-source_age
    ns['update_obstacles'](node)
    assert node.state['laser_scan_age']==round(source_age if source_age is not None else age,3)
    assert len(node.state['laser_scan'])==expected
    if expected:
        assert node.state['laser_scan']==[[3.,3.]]
        assert calls==[('map','front',{'seconds':9.9})]


@pytest.mark.parametrize('linear,angular,healthy,confirmed', [
    (.008,.00014,True,True),
    (.0099,-.0201,True,True),
    (.019,.029,True,True),
    (.029,0.,True,True),
    (.03,0.,True,False),
    (0.,-.03,True,False),
    (.05,0.,True,False),
    (.008,.001,False,False),
    (float('nan'),0.,True,False),
])
def test_stop_confirmation_tolerates_stationary_jitter_only(linear,angular,healthy,confirmed):
    cls=next(n for n in web_ast().body if isinstance(n,ast.ClassDef) and n.name=='Web')
    tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_tick')
    block=next(n for n in tick.body if isinstance(n,ast.If) and ast.unparse(n.test)=="self.phase == 'stopping'")
    code=compile(ast.fix_missing_locations(ast.Module(body=[block],type_ignores=[])),'stop-block','exec')
    obj=SimpleNamespace(phase='stopping',phase_at=0.,still_since=None,result_pending='canceled',
        state={'error':'未能确认停车，请检查现场状态','busy':True},
        sensors=SimpleNamespace(healthy=lambda:healthy,planar_speed=linear,velocity=[linear,angular]))
    stop_evidence_fixture(obj)
    for now in [10.,10.49]:
        exec(code,{'self':obj,'now':now})
        assert obj.phase=='stopping' and obj.state['busy']
    exec(code,{'self':obj,'now':10.51})
    assert (obj.phase=='idle')==confirmed
    if confirmed:
        assert not obj.state['busy'] and obj.state['error'] is None
    else:
        assert obj.still_since is None


def test_stop_confirmation_motion_resets_dwell():
    cls=next(n for n in web_ast().body if isinstance(n,ast.ClassDef) and n.name=='Web')
    tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_tick')
    block=next(n for n in tick.body if isinstance(n,ast.If) and ast.unparse(n.test)=="self.phase == 'stopping'")
    code=compile(ast.fix_missing_locations(ast.Module(body=[block],type_ignores=[])),'stop-block','exec')
    obj=SimpleNamespace(phase='stopping',phase_at=0.,still_since=10.,result_pending='canceled',
        state={'error':None,'busy':True},sensors=SimpleNamespace(healthy=lambda:True,planar_speed=.035,velocity=[.035,0.]))
    stop_evidence_fixture(obj)
    exec(code,{'self':obj,'now':10.4});assert obj.still_since is None
    obj.sensors.planar_speed=.008
    exec(code,{'self':obj,'now':10.5})
    exec(code,{'self':obj,'now':10.9});assert obj.phase=='stopping'
    exec(code,{'self':obj,'now':11.01});assert obj.phase=='idle'


def test_stale_sensor_still_faults_even_when_nav2_is_recovering():
    cls=next(n for n in web_ast().body if isinstance(n,ast.ClassDef) and n.name=='Web')
    tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_tick')
    block=next(n for n in tick.body if isinstance(n,ast.If) and ast.unparse(n.test)=='self.lease.active')
    stopped=[]
    node=SimpleNamespace(lease=SimpleNamespace(active=True,task='task'),valid=lambda task:True,
        state={'ready':False,'ready_checks':{'sensors':False}},sensors=SimpleNamespace(stamps={'scan':1.1},error=None),
        executor_state={},active_lifecycle=True,lifecycle_at=2.,
        stop=lambda reason,result,**kw:stopped.append((reason,result,kw)))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[block],type_ignores=[])),'stale-sensor','exec'),
         dict(self=node,now=2.,ros_now=2.,localized=True,execution=True,json=json))
    assert len(stopped)==1 and stopped[0][1]=='fault'
    assert stopped[0][2]['recoverable'] is True
    assert '传感器未就绪' in stopped[0][0] and '0.9' in stopped[0][0]


def test_initialization_uses_exact_confirmed_tf_and_waits_without_chasing_new_scan():
    cls=next(n for n in web_ast().body if isinstance(n,ast.ClassDef) and n.name=='Web')
    fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='refresh_initialization_transform')
    ns=dict(FRAME={'odom':'odom','front':'front'},rclpy=SimpleNamespace(time=SimpleNamespace(Time=lambda **kw:kw)))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'initialization-tf','exec'),ns)
    stamp=1792756845999991001;clock=[stamp+200000000];available=[False];calls=[]
    def lookup(a,b,t):
        calls.append(t)
        if not available[0]:raise RuntimeError('TF callback pending')
    node=SimpleNamespace(sensors=SimpleNamespace(latest_scan_stamp_ns=stamp),initial_tf_ns=None,initial_tf_error=None,
        tf=SimpleNamespace(lookup_transform=lookup),state={},
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=clock[0])))
    check=ns['refresh_initialization_transform']
    assert not check(node) and node.initial_tf_ns is None
    available[0]=True
    assert check(node) and node.initial_tf_ns==stamp
    assert calls[-1]=={'nanoseconds':stamp}
    # New scan arrived before its TF; the previously confirmed scan stays valid.
    node.sensors.latest_scan_stamp_ns=stamp+100000000;available[0]=False
    assert check(node) and node.initial_tf_ns==stamp
    # A genuinely missing transform cannot be renewed from the old cache.
    clock[0]=stamp+600000000
    assert not check(node) and node.state['initialization_tf_age']==.6


def test_initial_pose_keeps_confirmed_integer_nanoseconds_without_float_roundtrip():
    import time
    cls=next(n for n in web_ast().body if isinstance(n,ast.ClassDef) and n.name=='Web')
    fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='localize')
    stamp=1792756845999991001;published=[]
    def message():
        return SimpleNamespace(header=SimpleNamespace(frame_id=None,stamp=None),pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=0.,y=0.),orientation=SimpleNamespace(z=0.,w=1.)),covariance=[0.]*36))
    ns=dict(time=time,math=math,FRAME={'map':'map'},PoseWithCovarianceStamped=message,
        rclpy=SimpleNamespace(time=SimpleNamespace(Time=lambda **kw:SimpleNamespace(to_msg=lambda:kw['nanoseconds']))))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'localize','exec'),ns)
    node=SimpleNamespace(localization_reasons=lambda:[],initial_message=None,state={'busy':False},phase='idle',
        values=lambda *a,**kw:[1.,2.,.3],lease=SimpleNamespace(claim=lambda *a:None),convergence=SimpleNamespace(reset=lambda:None),
        initial_tf_ns=stamp,localization_id=0,initial=SimpleNamespace(publish=published.append),event=lambda text:None)
    ns['localize'](node,{'owner':'test-owner'})
    assert published[0].header.stamp==stamp
    assert int(node.initial_stamp*1e9)!=stamp  # Demonstrates why float is not a message timestamp.


@pytest.mark.parametrize('receive,source,expected',[(.35,.507,True),(.1,.8,True),(.1,.801,False),(.5,.2,False),(-.01,.2,False),(.1,-.051,False)])
def test_localization_freshness_accepts_pipeline_delay_but_rejects_lost_stream(receive,source,expected):
    from real_navigation.core import localization_fresh
    assert localization_fresh(receive,source) is expected


def stop_evidence_fixture(obj):
    obj.mode='observe';obj.executor_at=10.;obj.executor_state={'stationary':True}
    obj.sensors.settling=SimpleNamespace(mean=lambda now:[obj.sensors.planar_speed,abs(obj.sensors.velocity[1])])


@pytest.mark.parametrize('stationary,age',[(False,.01),(True,.31)])
def test_stop_requires_fresh_independent_sdk_confirmation(stationary,age):
    cls=next(n for n in web_ast().body if isinstance(n,ast.ClassDef) and n.name=='Web')
    tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_tick')
    block=next(n for n in tick.body if isinstance(n,ast.If) and ast.unparse(n.test)=="self.phase == 'stopping'")
    obj=SimpleNamespace(phase='stopping',phase_at=0.,still_since=9.,result_pending='canceled',
        state={'error':None,'busy':True},sensors=SimpleNamespace(healthy=lambda:True,planar_speed=.01,velocity=[.01,0.]))
    stop_evidence_fixture(obj);obj.mode='hardware';obj.executor_at=10.-age;obj.executor_state={'stationary':stationary}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[block],type_ignores=[])),'stop-block','exec'),{'self':obj,'now':10.})
    assert obj.phase=='stopping' and obj.still_since is None
