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

@pytest.mark.parametrize('age,missing_tf,expected',[(.1,False,1),(1.,False,0),(.1,True,0)])
def test_scan_overlay_uses_timestamped_transform_or_hides(age,missing_tf,expected):
    from scipy.spatial.transform import Rotation
    cls=next(n for n in web_ast().body if isinstance(n,ast.ClassDef) and n.name=='Web')
    fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='update_obstacles')
    ns=dict(np=np,Rotation=Rotation,FRAME={'map':'map','front':'front'},
            rclpy=SimpleNamespace(time=SimpleNamespace(Time=lambda **kw:kw)))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'overlay','exec'),ns)
    calls=[]
    def lookup(a,b,stamp):
        calls.append((a,b,stamp))
        if missing_tf:raise RuntimeError('no transform')
        return SimpleNamespace(transform=SimpleNamespace(rotation=SimpleNamespace(x=0,y=0,z=0,w=1),translation=SimpleNamespace(x=2,y=3,z=0)))
    node=SimpleNamespace(sensors=SimpleNamespace(obstacles={},latest_scan=(10.-age,[[1.,0.,0.]]),error=None),state={},
        tf=SimpleNamespace(lookup_transform=lookup),get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)))
    ns['update_obstacles'](node)
    assert len(node.state['laser_scan'])==expected
    if expected:
        assert node.state['laser_scan']==[[3.,3.]]
        assert calls==[('map','front',{'seconds':9.9})]
