"""Regression checks without ROS nodes, sockets, simulator or hardware."""
from pathlib import Path
from types import SimpleNamespace as S
import sys
import threading
import pytest
pytest.importorskip('rclpy')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from nav_web import WebNavigation


def test_discovered_action_does_not_mean_navigation_is_activated():
    fake=S(last_truth=100.,nav_active=False,lifecycle_reply_at=100.,
           action=S(server_is_ready=lambda:True))
    assert not WebNavigation.navigation_ready(fake,100.1,.1)
    fake.nav_active=True
    assert WebNavigation.navigation_ready(fake,100.1,.1)
    fake.last_truth=103.
    assert not WebNavigation.navigation_ready(fake,103.1,.1)  # stale lifecycle reply


def test_old_truth_is_not_drawn_as_a_teleport_trajectory():
    fake = S(lock=threading.RLock(), state={'initialized':False, 'truth_trajectory':[],
                                         'start':[20., 30., 0.]}, last_truth=0.)
    fake.append = lambda name, pose: WebNavigation.append(fake, name, pose)
    p = S(position=S(x=-10., y=9.), orientation=S(x=0., y=0., z=0., w=1.))
    msg = S(pose=S(pose=p),twist=S(twist=S(linear=S(x=0.),angular=S(z=0.))))
    WebNavigation.truth(fake, msg)
    assert fake.state['truth_trajectory'] == []
    fake.state['initialized'] = True; p.position.x = 20.; p.position.y = 30.
    WebNavigation.truth(fake, msg)
    assert fake.state['truth_trajectory'] == [[20., 30.]]


def test_unreachable_goal_does_not_submit_action_or_enable_motion():
    events = []; gates = []
    fake = S(busy=False, state={'initialized':True, 'ready':True, 'truth':[0.,0.,0.]},
             grid=S(safe=lambda p:True, connected=lambda s,g:False),
             event=events.append, set_gate=gates.append)
    # No action client is provided: the request must be rejected before send.
    WebNavigation.execute(fake, 'goal', [10., 10., 0.])
    assert not fake.busy and not gates
    assert fake.state['status'] == '目标不可达，未启动运动'
    assert '连续已知空闲通道' in fake.state['error']
    assert fake.state['distance_remaining'] is None


@pytest.mark.parametrize('level', [30, bytes([30])])
def test_humble_diagnostic_level_handles_integer_and_byte_bindings(level):
    fake = S(busy=True, lock=threading.RLock(), state={'diagnostics':[]})
    WebNavigation.diagnostic(fake, S(level=level, name='planner_server', msg='No valid path'))
    assert fake.state['diagnostics'] == ['planner_server: No valid path']


def test_stationary_amcl_accepts_recent_scan_before_initial_request():
    fake = S(initial_stamp=100.15, last_amcl=0., amcl_pose=None)
    pose = S(position=S(x=1., y=2.), orientation=S(x=0., y=0., z=0., w=1.))
    msg = S(header=S(stamp=S(sec=100, nanosec=0)), pose=S(pose=pose))
    WebNavigation.amcl(fake, msg)
    assert fake.last_amcl > 0 and fake.amcl_pose == [1., 2., 0.]
    fake.last_amcl = 0.; msg.header.stamp.sec = 98
    WebNavigation.amcl(fake, msg)
    assert fake.last_amcl == 0.


def test_command_is_not_reported_as_actual_motion():
    fake=S(lock=threading.RLock(),state={'initialized':False,'velocity':[0.,0.],
                                       'command_velocity':[0.,0.]},last_truth=0.)
    WebNavigation.velocity(fake,S(linear=S(x=.08),angular=S(z=.04)))
    assert fake.state['velocity']==[0.,0.] and fake.state['command_velocity']==[.08,.04]
    pose=S(position=S(x=1.,y=2.),orientation=S(x=0.,y=0.,z=0.,w=1.))
    WebNavigation.truth(fake,S(pose=S(pose=pose),twist=S(twist=S(linear=S(x=0.),angular=S(z=0.)))))
    assert fake.state['velocity']==[0.,0.]


def test_collision_revokes_simulation_permission_immediately():
    gates=[];events=[]
    fake=S(lock=threading.RLock(),state={'collisions':0},busy=True,stop_pending=False,
           goal_handle=None,set_gate=gates.append,event=events.append)
    WebNavigation.collision(fake,S(data=True))
    assert fake.stop_pending and gates==[False] and '已停止' in fake.state['error']
    WebNavigation.collision(fake,S(data=True))
    assert gates==[False] and len(events)==1


@pytest.mark.parametrize('value', [.02, .08, .1, .3])
def test_navigation_speed_range(value):
    from nav_web import navigation_speed
    assert navigation_speed(value) == value


@pytest.mark.parametrize('value', [0, -.1, .301, float('nan'), float('inf')])
def test_navigation_speed_rejects_invalid_values(value):
    from nav_web import navigation_speed
    with pytest.raises(ValueError): navigation_speed(value)


@pytest.mark.parametrize('success,cancelled,ready,expected', [
    (True,False,True,True), (False,False,True,False),
    (True,True,True,False), (True,False,False,False)])
def test_goal_requires_speed_acknowledgement(success,cancelled,ready,expected):
    sent=[]; gates=[]
    future=S(result=lambda:S(result=S(successful=success,reason='rejected')))
    fake=S(lock=threading.RLock(),speed_future=future,stop_pending=cancelled,
           state={'ready':ready},busy=True,set_gate=gates.append,
           send_navigation_goal=sent.append)
    WebNavigation.speed_applied(fake,future,[1.,2.,0.],.3)
    assert bool(sent)==expected
    if not expected: assert not fake.busy and gates==[False]


def test_stale_speed_reply_cannot_start_goal():
    fake=S(lock=threading.RLock(),speed_future=None)
    WebNavigation.speed_applied(fake,object(),[1.,2.,0.],.3)
