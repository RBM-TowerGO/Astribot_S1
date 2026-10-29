"""Isolated coordinator tests: no ROS graph or hardware commands."""
import ast
from pathlib import Path
from types import SimpleNamespace as S
import pytest

ROOT=Path(__file__).resolve().parents[1]

def coordinator():
    tree=ast.parse((ROOT/'scripts/real_navigation/web.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Web')
    names={'pause_allowed','pause_for_data','tick_data_pause'}
    cls=ast.ClassDef(name='Coordinator',bases=[],keywords=[],body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in names],decorator_list=[])
    import sys
    sys.path.insert(0,str(ROOT/'scripts'))
    from real_navigation.core import sensor_fresh
    ns={'sensor_fresh':sensor_fresh,'NS':'/towergo/nav/'}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),'web','exec'),ns)
    w=ns['Coordinator']();w.sent=[];w.events=[];w.stops=[];w.starts=[]
    w.send=lambda op:w.sent.append(op);w.event=w.events.append
    w.stop=lambda *a,**kw:(w.stops.append(a),setattr(w,'data_pause',None))
    w.begin=w.starts.append
    w.count_publishers=lambda t:1
    w.sensors=S(error=None,recovering_until=0,received={k:100 for k in ('front','back','imu','lio')},stamps={k:100 for k in ('front','back','imu','lio')})
    w.quality=S(monitor=S(invalid=False,state={'status':'trusted'}),recovery=S(status='idle'))
    w.executor_at=100;w.executor_state={'rights':True,'stationary':True,'task':None}
    w.confidence=S(blocked=False);w.amcl_cov=.05;w.amcl_active=True;w.amcl_lifecycle_at=100;w.active_lifecycle=True;w.lifecycle_at=100
    w.goal_attempt=1;w.action_finished=False;w.goal_handle=S(cancel_goal_async=lambda:w.events.append('cancel'))
    w.lease=S(owner='owner',stop=lambda:w.events.append('lease revoked'))
    w.state={'goal':[1,2,.3],'requested_speed':.05};w.localization={}
    w.pause_for_data(100)
    return w

def fresh(w,t):
    w.executor_at=w.amcl_lifecycle_at=w.lifecycle_at=t
    for k in w.sensors.received:w.sensors.received[k]=w.sensors.stamps[k]=t

def test_pause_requests_hold_cancels_action_and_preserves_goal():
    w=coordinator()
    assert w.sent==['cancel_stationary'] and 'cancel' in w.events
    assert w.phase=='paused' and w.goal_attempt==2 and w.state['command_velocity']==[0,0]
    assert w.data_pause['goal']==[1,2,.3] and not w.starts

def test_resume_requires_two_continuous_seconds_and_old_action_finished():
    w=coordinator()
    w.tick_data_pause(100,100,True)
    assert w.data_pause['good_since'] is None
    w.action_finished=True
    for t,healthy in [(101,True),(102,False),(103,True),(104.99,True)]:
        fresh(w,t);w.tick_data_pause(t,t,healthy)
    assert not w.starts
    fresh(w,105);w.tick_data_pause(105,105,True)
    assert w.starts==[dict(x=1,y=2,yaw=.3,owner='owner',speed=.05)]
    assert 'lease revoked' in w.events and w.data_pause is None

@pytest.mark.parametrize('kind',['rights','fault','feedback','raw','mismatch','history','publisher','lifecycle'])
def test_other_failures_cancel_without_resume(kind):
    w=coordinator();w.action_finished=True
    if kind=='rights':w.executor_state['rights']=False
    if kind=='fault':w.executor_state['fault']='SDK fault'
    if kind=='feedback':w.executor_at=99
    if kind=='raw':w.sensors.received['imu']=99
    if kind=='mismatch':w.quality.monitor.invalid=True
    if kind=='history':w.sensors.error='history reset'
    if kind=='publisher':w.count_publishers=lambda t:2
    if kind=='lifecycle':w.active_lifecycle=False
    w.tick_data_pause(100,100,True)
    assert w.stops and not w.starts and w.data_pause is None

@pytest.mark.parametrize('field',['stationary','holding','task'])
def test_stationary_and_hold_completion_required(field):
    w=coordinator();w.action_finished=True
    w.executor_state[field]={'stationary':False,'holding':True,'task':'old'}[field]
    for t in [100,103]:
        fresh(w,t);w.tick_data_pause(t,t,True)
    assert not w.starts and w.data_pause['good_since'] is None

def test_ten_seconds_is_absolute_deadline():
    w=coordinator();w.action_finished=True
    fresh(w,110);w.tick_data_pause(110,110,True)
    assert w.stops and not w.starts

def test_stop_clears_pause_and_invalidates_old_action():
    tree=ast.parse((ROOT/'scripts/real_navigation/web.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Web')
    fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='stop')
    end=next(i+1 for i,n in enumerate(fn.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Attribute) and t.attr=='goal_attempt' for t in n.targets))
    ns={};exec(compile(ast.fix_missing_locations(ast.Module(body=fn.body[:end],type_ignores=[])),'stop-prefix','exec'),{'self':(w:=coordinator())})
    assert w.retreat is None and w.state['recovery'] is None
    assert w.data_pause is None and w.goal_attempt==3

@pytest.mark.parametrize('browser_valid,tf_ok,scan_age,expected',[(True,False,.1,'pause'),(True,True,1.01,'pause'),(False,False,.1,'stop')])
def test_actual_dispatch_prioritizes_manual_lease_loss(browser_valid,tf_ok,scan_age,expected):
    import json
    import sys
    sys.path.insert(0,str(ROOT/'scripts'))
    from real_navigation.core import sensor_fresh
    w=coordinator();w.phase='running';w.data_pause=None
    w.lease.active=True;w.lease.task='task';w.valid=lambda t:browser_valid
    w.state.update(ready=False,ready_checks={'sensors':False})
    w.sensors.received['scan']=100;w.sensors.stamps['scan']=100-scan_age
    tree=ast.parse((ROOT/'scripts/real_navigation/web.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Web')
    tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_tick')
    block=next(n for n in tick.body if isinstance(n,ast.If) and ast.unparse(n.test)=='self.lease.active')
    exec(compile(ast.fix_missing_locations(ast.Module(body=[block],type_ignores=[])),'dispatch','exec'),
         dict(self=w,now=100,ros_now=100,tf_ok=tf_ok,sensor_fresh=sensor_fresh,json=json))
    assert bool(w.stops)==(expected=='stop')
    assert (w.phase=='paused')==(expected=='pause')


def test_data_pause_does_not_reset_retreat_attempt_budget():
    w=coordinator();w.recovery_attempts=1;w.action_finished=True
    begin=w.begin
    def restarting(obj):
        w.recovery_attempts=0;begin(obj)
    w.begin=restarting
    fresh(w,100);w.tick_data_pause(100,100,True)
    fresh(w,102.01);w.tick_data_pause(102.01,102.01,True)
    assert len(w.starts)==1 and w.recovery_attempts==1
