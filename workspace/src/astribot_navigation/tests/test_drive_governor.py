"""Closed-loop fake plant; no ROS, SDK, or hardware control."""
import sys,math
from pathlib import Path
import yaml
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from real_navigation.core import Drive

class Plant:
    def __init__(self,tau=.7):
        self.p=[0.,0.,0.];self.target=[0.,0.,0.];self.now=0.;self.tau=tau
        self.stops=0;self.sent=[];self.rights=True;self.frozen=False
    def desired(self):return self.target[:]
    def feedback(self):return self.p[:],self.now
    def has_control(self):return self.rights
    def send(self,p):self.target=p[:];self.sent.append(p[:])
    def stop(self):self.stops+=1
    def advance(self,dt):
        self.now+=dt
        if not self.frozen:
            a=1-math.exp(-dt/self.tau)
            self.p=[x+(y-x)*a for x,y in zip(self.p,self.target)]

def setup(tau=.7):
    p=Plant(tau);c=yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text())
    d=Drive(p,c,0);d.prepare('task',.3);return p,d

def step(p,d,dt=.01,v=.15,w=.05):
    p.advance(dt);d.permit('task',p.now);d.command('task',v,w,p.now)
    return d.tick(p.now)

@pytest.mark.parametrize('tau',[.05,.7,2.])
def test_ten_minute_slow_plant_lead_bounded(tau):
    p,d=setup(tau);maximum=0
    for i in range(60000):
        v,w=step(p,d)
        maximum=max(maximum,d.tracking['position_error_m'])
        assert d.tracking['position_error_m']<=.10000001
        assert d.tracking['yaw_error_rad']<=.10000001
        assert d.fault is None
        if v:assert v/w==pytest.approx(3.)
    assert p.p[0]>5
    assert p.stops==0
    print(dict(tau=tau,travel=p.p[0],max_lead=maximum))

def test_blocked_plant_then_release_no_backlog_burst():
    p,d=setup();p.frozen=True
    for _ in range(2000):step(p,d)
    assert d.target[0]<=.1 and d.fault is None and p.stops==0
    p.frozen=False
    previous=d.target[:]
    for _ in range(2000):
        v,w=step(p,d)
        assert 0<=d.target[0]-previous[0]<=.001500001
        assert v<=.15 and abs(w)<=.05
        previous=d.target[:]
    assert p.p[0]>1

def test_zero_withdraws_outstanding_target():
    p,d=setup()
    for _ in range(100):step(p,d)
    assert d.target[0]-p.p[0]>.02
    assert step(p,d,v=0,w=0)==(0.,0.)
    assert d.target==p.p
    stopped=p.p[:]
    for _ in range(100):step(p,d,v=0,w=0)
    assert p.p==stopped and d.fault is None

def test_jitter_skip_no_integral_catchup():
    p,d=setup()
    step(p,d);before=d.target[:]
    assert step(p,d,dt=.2)==(0.,0.)
    assert d.target==before and d.fault is None and p.stops==0
    step(p,d)
    assert d.target[0]-before[0]<=.00150001

def test_data_loss_cancel_hold_no_auto_resume():
    p,d=setup()
    for _ in range(100):step(p,d)
    assert d.suspend(p.now,'扫描过期')
    assert d.target==p.p and d.task is None
    for _ in range(100):step(p,d)
    assert not d.holding and not d.fault and not p.stops
    assert d.task is None
    assert not d.command('task',.15,0,p.now)
    d.prepare('new',.05)
    d.permit('new',p.now+.01);d.command('new',.05,0,p.now+.01)
    p.advance(.01);assert d.tick(p.now)[0]>.0

def test_hold_that_does_not_stop_escalates():
    p,d=setup();step(p,d);d.suspend(p.now,'扫描过期')
    for _ in range(310):
        p.now+=.01;p.p[0]+=.002;d.tick(p.now)
    assert d.fault and p.stops==1 and d.task is None

def test_stale_feedback_still_stops():
    p,d=setup();step(p,d);d.tick(p.now+.2)
    assert '反馈' in d.fault and p.stops==1

def test_explicit_stop_stays_latched():
    p,d=setup();step(p,d)
    p.requires_restart=True;d.stop()
    assert d.fault and not d.command('task',.1,0,p.now)
    with pytest.raises(ValueError):d.prepare('new',.05)

def test_nonfinite_feedback_never_sent():
    p,d=setup();p.p[0]=float('nan');step(p,d)
    assert d.fault and not p.sent

def test_command_timeout_cancels_and_does_not_revive():
    p,d=setup();step(p,d)
    for _ in range(40):
        p.advance(.01);d.permit('task',p.now);d.tick(p.now)
    assert d.task is None and d.fault is None
    assert not d.command('task',.1,0,p.now)

def test_web_cancels_lease_when_executor_canceled_task():
    import ast,json
    from types import SimpleNamespace
    tree=ast.parse((ROOT/'scripts/real_navigation/web.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Web')
    tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_tick')
    block=next(n for n in tick.body if isinstance(n,ast.If) and ast.unparse(n.test)=='self.lease.active')
    stopped=[]
    node=SimpleNamespace(lease=SimpleNamespace(active=True,task='old'),
        valid=lambda task:True,pause_allowed=lambda *a:False,state={'ready':True},phase='running',
        executor_state={'task':None,'warning':'指令过期'},
        stop=lambda *a,**kw:stopped.append((a,kw)))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[block],type_ignores=[])),'web-test','exec'),
         dict(self=node,now=1.,ros_now=1.,tf_ok=False,localized=True,execution=True,json=json))
    assert stopped==[(('指令过期','fault'),{'recoverable':True})]
