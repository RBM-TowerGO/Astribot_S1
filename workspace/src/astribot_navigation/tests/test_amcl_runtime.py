"""Real runtime coordinator methods with ROS service/log stand-ins."""
import ast
import io
import json
import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from real_navigation.confidence import Confidence,StaticReassessment
from real_navigation.localization_runtime import LocalizationRuntime


def runtime():
    r=LocalizationRuntime.__new__(LocalizationRuntime)
    c=Confidence();cov=[0.]*36;cov[0]=cov[7]=.02;cov[35]=.29;c.observe(cov);c.tick(0,True,False)
    r.web=S(confidence=c,localization={'status':'recovering'},initial_message=None,
            executor_state={'rights':True,'stationary':True,'task':None},state={'busy':False},lease=S(active=False),phase='idle',
            refresh_initialization_transform=lambda:True,amcl_active=True,amcl_lifecycle_at=100,amcl_stamp=100)
    r.web.events=[];r.web.event=r.web.events.append
    r.recovery=S(status='idle');r.monitor=S(usable=lambda now:True,state={'status':'trusted'})
    r.reassessment=StaticReassessment();r.nudge_future=None;r.requests=[]
    r.nudge=S(service_is_ready=lambda:True,call_async=lambda req:(r.requests.append(req),S(done=lambda:True))[1])
    r.empty=S(Request=lambda:'request')
    return r

@pytest.mark.parametrize('failure',['moving','holding','fault','task','rights','stale_executor','busy','lease','tf','quality','manual_stop','amcl_inactive','stale_amcl_lifecycle','sensor'])
def test_no_reassessment_without_eligibility(failure):
    r=runtime();w=r.web;stationary=True;sensors=True
    if failure=='moving':stationary=False
    if failure=='stale_executor':stationary=False  # caller derives stationary only from < .3 s SDK feedback
    if failure=='holding':w.executor_state['holding']=True
    if failure=='fault':w.executor_state['fault']='SDK'
    if failure=='task':w.executor_state['task']='old'
    if failure=='rights':w.executor_state['rights']=False
    if failure=='busy':w.state['busy']=True
    if failure=='lease':w.lease.active=True
    if failure=='tf':w.refresh_initialization_transform=lambda:False
    if failure=='quality':r.monitor.usable=lambda now:False
    if failure=='manual_stop':r.recovery.status='canceled'
    if failure=='amcl_inactive':w.amcl_active=False
    if failure=='stale_amcl_lifecycle':w.amcl_lifecycle_at=90
    if failure=='sensor':sensors=False
    r.tick_reassessment(100,stationary,sensors)
    assert not r.requests and r.reassessment.started is None

def test_runtime_limits_requests_no_initialpose_or_navigation_and_expires():
    r=runtime()
    for now in range(100,111):
        r.web.amcl_lifecycle_at=now;r.web.amcl_stamp=now
        r.tick_reassessment(now,True,True)
    assert len(r.requests)==3
    assert r.web.localization['status']=='manual_required'
    assert r.web.state['initialized'] is False
    # No publisher, goal sender, or SDK exists in this fixture: unexpected use fails.

def test_no_retry_while_service_request_pending_or_same_amcl_stamp():
    r=runtime();r.tick_reassessment(100,True,True)
    r.nudge_future=S(done=lambda:False);r.web.amcl_stamp=101
    r.tick_reassessment(101,True,True);assert len(r.requests)==1
    r.nudge_future=S(done=lambda:True);r.web.amcl_stamp=100
    r.tick_reassessment(102,True,True);assert len(r.requests)==1

def test_continuous_log_records_full_covariance_and_exact_odometry_increment():
    r=runtime();r.amcl_log=io.StringIO();r.previous_amcl_odom=None
    r.expectation=S(observe=lambda *args:None)
    pose=np.eye(4)
    r.web.sensors=S(geometry=S(at=lambda stamp:pose.copy()))
    r.web.get_clock=lambda:S(now=lambda:S(nanoseconds=102000000000))
    r.web.localization_id=7;r.web.cmd=(0,0)
    covariance=[0.]*36;covariance[0]=.02;covariance[7]=.03;covariance[35]=.29;covariance[1]=covariance[6]=.005
    r.observe_amcl(100,covariance)
    pose[0,3]=.012;angle=.05;pose[:2,:2]=[[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]]
    r.observe_amcl(101,covariance)
    rows=[json.loads(v) for v in r.amcl_log.getvalue().splitlines()]
    assert rows[0]['odom_delta'] is None
    assert rows[1]['covariance']==covariance
    assert rows[1]['odom_delta']['translation_m']==pytest.approx(.012)
    assert rows[1]['odom_delta']['yaw_rad']==pytest.approx(.05)
    assert rows[1]['sigma_yaw_deg']==pytest.approx(math.degrees(math.sqrt(.29)))

@pytest.mark.parametrize('stamp',[90,100,100.1])
def test_old_amcl_never_changes_confidence(stamp):
    from real_navigation.core import localization_fresh
    cls=next(n for n in ast.parse((ROOT/'scripts/real_navigation/web.py').read_text()).body if isinstance(n,ast.ClassDef))
    fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='amcl')
    ns=dict(time=time,math=math,FRAME={'map':'map'},localization_fresh=localization_fresh)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),'amcl','exec'),ns)
    w=S(get_clock=lambda:S(now=lambda:S(nanoseconds=100200000000)),amcl_stamp=100.1,initial_stamp=100,
        sensors=S(metrics=S(add=lambda *args:None)),amcl_at=-1,confidence=Confidence())
    m=S(header=S(frame_id='map',stamp=S(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))))
    ns['amcl'](w,m)
    assert w.amcl_stamp==100.1 and w.amcl_at==-1 and not w.confidence.details['valid']
