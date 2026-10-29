"""Bounded backup, live/map collision checks, and recovery dispatch; no hardware."""
import ast,math,sys
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
import pytest,yaml
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from real_navigation.recovery import Retreat,backup_clearance,recoverable_navigation_failure
from real_navigation.core import Drive,limited


def test_retreat_requires_stop_then_reverses_then_stops_before_replan():
    r=Retreat(0)
    assert r.step(0,[0,0,0],False)==0
    assert r.step(.1,[0,0,0],True)==0
    assert r.step(.41,[0,0,0],True)==0 and r.phase=='backing'
    assert r.step(.5,[0,0,0],False)==-.03
    assert r.step(5.5,[-.15,0,0],False)==0 and r.phase=='settling'
    assert r.step(6,[-.15,0,0],False)==0
    assert r.step(6.1,[-.15,0,0],True)==0
    assert r.step(6.41,[-.15,0,0],True)==0 and r.phase=='replanning'


@pytest.mark.parametrize('feedback',[[.021,0,0],[0,.031,0],[0,0,.051],[float('nan'),0,0]])
def test_wrong_direction_or_invalid_feedback_aborts(feedback):
    r=Retreat(0);r.phase='backing';r.origin=[0,0,0]
    with pytest.raises(RuntimeError):r.step(1,feedback,False)


def test_recovery_has_absolute_deadline():
    with pytest.raises(RuntimeError,match='12 秒'):Retreat(0).step(12,[0,0,0],False)


def scene():
    c=yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text())
    grid=S(origin=np.array([-5.,-5.,0.]),rotation=np.eye(2),resolution=.05,width=200,height=200,free=np.ones((200,200),bool))
    clouds=[np.tile([2.,2.,.5],(30,1)),np.tile([-2.,2.,.5],(30,1))]
    return c,grid,clouds


def check(c,g,clouds):return backup_clearance(g,[0,0,0],c['footprint'],c['footprint_padding'],.15,clouds)


def test_backing_checks_whole_sweep_and_live_obstacles():
    c,g,clouds=scene();assert check(c,g,clouds)==(True,None)
    # A small real object between body and proposed retreat endpoint must stop it.
    clouds[1][0]=[-.65,0.,.5];assert not check(c,g,clouds)[0]
    c,g,clouds=scene();g.free[100,86]=False;assert not check(c,g,clouds)[0]
    c,g,clouds=scene();assert not check(c,g,clouds[:1])[0]
    clouds[1]=np.empty((0,3));assert not check(c,g,clouds)[0]


def test_recovery_only_for_navigation_failures_and_bounded_attempts():
    assert recoverable_navigation_failure(6,'路径不满足完整机身避障余量',0)
    assert recoverable_navigation_failure(6,'控制器预测到前方碰撞',1)
    assert not recoverable_navigation_failure(6,'控制器预测到前方碰撞',2)
    assert not recoverable_navigation_failure(5,'控制器预测到前方碰撞',0)
    assert not recoverable_navigation_failure(6,'动作请求应答超时',0)
    assert not recoverable_navigation_failure(6,'定位失效',0)


def test_signed_velocity_only_through_explicit_bounded_recovery_command():
    from test_drive_governor import setup
    p,d=setup()
    with pytest.raises(ValueError):limited(-.03,0,.05)
    with pytest.raises(ValueError):d.command('task',-.03,0,0)
    for bad in [-.031,.001,float('nan')]:
        with pytest.raises(ValueError):d.backup_command('task',bad,0)
    for _ in range(500):
        p.advance(.01);d.permit('task',p.now);assert d.backup_command('task',-.03,p.now)
        v,w=d.tick(p.now);assert -.03<=v<=0 and w==0
        assert d.tracking['position_error_m']<=.1000001
    assert p.p[0]<-.1 and d.fault is None
    d.suspend(p.now,'用户取消');assert not d.backup_command('task',-.03,p.now)


def coordinator():
    tree=ast.parse((ROOT/'scripts/real_navigation/web.py').read_text())
    cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='Web')
    fn=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='tick_retreat')
    ns={};exec(compile(ast.Module(body=[fn],type_ignores=[]),'retreat','exec'),ns)
    sent=[];stops=[];goals=[]
    w=S(retreat=Retreat(0),recovery_attempts=1,executor_state=dict(tracking=dict(sampled_at=0,feedback=[0,0,0]),stationary=True,task='task'),
        lease=S(task='task'),state={},send=lambda *a,**k:sent.append((a,k)),stop=lambda *a,**k:stops.append(a),submit_goal=lambda:goals.append(True),event=lambda m:None,retreat_clearance=lambda d:(True,None))
    return ns['tick_retreat'],w,sent,stops,goals


def test_dispatch_requires_fresh_feedback_clear_rear_and_stop_before_resubmit():
    tick,w,sent,stops,goals=coordinator()
    for t,x,stationary in [(0,0,True),(.31,0,True),(.4,0,False),(5.5,-.15,False),(6,-.15,True),(6.31,-.15,True)]:
        w.executor_state.update(stationary=stationary,tracking=dict(sampled_at=t,feedback=[x,0,0]))
        tick(w,t)
    assert goals==[True] and not stops
    assert any(op[0]==('backup_command',) for op in sent)
    assert sent[-1][0]==('command',) and sent[-1][1]['v']==0


@pytest.mark.parametrize('blocked,stale',[(True,False),(False,True)])
def test_dispatch_does_not_reverse_without_clearance_or_fresh_feedback(blocked,stale):
    tick,w,sent,stops,goals=coordinator();w.retreat.phase='backing';w.retreat.origin=[0,0,0]
    if blocked:w.retreat_clearance=lambda d:(False,'后方有障碍')
    tick(w,.4 if stale else .1)
    assert stops and not sent and not goals


def test_live_clearance_uses_full_cloud_not_decimated_display_and_rejects_old_data():
    import threading
    tree=ast.parse((ROOT/'scripts/real_navigation/web.py').read_text())
    cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='Web')
    fn=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='retreat_clearance')
    ns=dict(np=np,backup_clearance=backup_clearance)
    exec(compile(ast.Module(body=[fn],type_ignores=[]),'clearance','exec'),ns)
    c,g,_=scene();base_lidar=np.asarray(c['base_from_lidar'])
    geometry=S(base_lidar=base_lidar,at=lambda stamp:np.eye(4))
    clouds={}
    for key in ('front','back'):
        origin=base_lidar[:3,3].copy()
        if key=='back':origin+=base_lidar[:3,:3]@np.asarray(c['rear_origin_in_front'])
        clouds[key]=(1.,np.tile(np.array([-2.,2.,.5])-origin,(1200,1)))
    sensors=S(last_pose=(1.1,np.eye(4)),geometry=geometry,geometry_lock=threading.Lock(),recovery_clouds=clouds)
    w=S(sensors=sensors,config=c,state={'pose':[0,0,0]},grid=g,get_clock=lambda:S(now=lambda:S(nanoseconds=1_300_000_000)))
    assert ns['retreat_clearance'](w,.15)[0]
    # Odd-index obstacle disappears in a [::2] display sample but must stop backup.
    origin=base_lidar[:3,3]+base_lidar[:3,:3]@np.asarray(c['rear_origin_in_front'])
    clouds['back'][1][1]=np.array([-.65,0,.5])-origin
    assert not ns['retreat_clearance'](w,.15)[0]
    w.get_clock=lambda:S(now=lambda:S(nanoseconds=1_600_000_000))
    assert '过期' in ns['retreat_clearance'](w,.15)[1]
