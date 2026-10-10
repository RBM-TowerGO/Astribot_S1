import math
import sys
from pathlib import Path
import pytest
from towergo_real.control import Controller, FakeBackend, KeyboardLease, Limits, State, checked_twist
from towergo_real.contracts import calibration, hardware_contract


def armed():
    core = Controller()
    backend = FakeBackend(lambda: .01)
    core.enable(lambda: backend, .01, 100.)
    core.command([0]*6,100.001,.01,100.001)
    return core, backend


@pytest.mark.parametrize('key,expected', [('w',(.1,0)),('s',(-.1,0)),('a',(0,.2)),('d',(0,-.2))])
def test_key_lease_and_repeat(key, expected):
    lease = KeyboardLease(); lease.feed(key, 1.)
    assert lease.sample(1.19) == expected
    assert lease.sample(1.21) == (0,0)
    lease.feed(key*20, 2.)
    assert lease.sample(2.1) == expected
    assert lease.sample(2.21) == (0,0)


@pytest.mark.parametrize('data', ['', ' ', 'x', '\x1b[A', 'w w', 'wwq', '\x03'])
def test_stop_wins_input_burst(data):
    lease = KeyboardLease(); lease.feed('w', 1); lease.feed(data, 1.1)
    assert lease.sample(1.1) == (0,0)


def test_disabled_never_constructs_backend():
    core = Controller()
    assert core.backend is None
    assert not core.command([.1,0,0,0,0,0], 1, 1, 1)
    core.tick(1.)
    assert core.state == State.DISABLED


def test_signed_limits_and_unsupported_axes():
    assert checked_twist([-2,0,0,0,0,2], Limits()) == (-.2,.3)
    for index in [1,2,3,4]:
        v=[0.]*6;v[index]=.01
        with pytest.raises(ValueError):checked_twist(v, Limits())
    for value in [float('nan'), float('inf'), -float('inf')]:
        with pytest.raises(ValueError):checked_twist([value,0,0,0,0,0], Limits())


def test_actual_dt_integration_and_reverse():
    core,backend = armed()
    assert core.command([-.1,0,0,0,0,0],100.01,.011,100.01)
    core.tick(.014)
    assert backend.p[0] < 0
    assert backend.p[0] == pytest.approx(-.2*.004*.004)
    assert backend.p[1:] == [0,0]


def test_stop_is_not_acceleration_limited():
    core,backend = armed()
    core.command([.1,0,0,0,0,0],100.01,.011,100.01);core.tick(.014)
    core.command([0]*6,100.02,.02,100.02)
    assert backend.calls[-1] == ('stop',)
    assert core.velocity == (0,0)
    assert core.state == State.READY


@pytest.mark.parametrize('stamp,now', [(100.001,100.5),(101.,100.01),(float('nan'),100.01)])
def test_expired_future_prearm_commands_lock_stop(stamp,now):
    core,backend = armed()
    assert not core.command([.1,0,0,0,0,0],stamp,.011,now)
    assert core.state == State.FAULT
    assert backend.calls[-1] == ('stop',)
    assert not core.command([.1,0,0,0,0,0],101.,.02,101.)


def test_pre_enable_packet_is_discarded_without_extending_lease():
    core,backend=armed();old_received=core.last_received
    assert not core.command([.1,0,0,0,0,0],100.,.02,100.02)
    assert core.state==State.READY and core.last_received==old_received
    assert not backend.calls


def test_duplicate_stamp_cannot_extend_deadline():
    core,backend=armed()
    core.command([.1,0,0,0,0,0],100.01,.011,100.01)
    core.command([.1,0,0,0,0,0],100.01,.02,100.02)
    assert core.state == State.FAULT


def test_nonzero_cannot_arm_a_new_session():
    core=Controller();backend=FakeBackend(lambda:.01)
    core.enable(lambda:backend,.01,100.)
    assert not core.command([.1,0,0,0,0,0],100.01,.011,100.01)
    assert core.state==State.FAULT
    assert not [c for c in backend.calls if c[0]=='send']


@pytest.mark.parametrize('failure', ['timeout','cycle','feedback','rights','tracking'])
def test_faults_stop_and_do_not_resume(failure):
    core,backend=armed()
    core.command([.1,0,0,0,0,0],100.01,.011,100.01)
    if failure=='timeout':core.last_received=-1
    if failure=='feedback':backend.feedback_stamp=-1
    if failure=='rights':backend.rights=False
    if failure=='tracking':backend.p[0]=1
    core.tick(.5 if failure=='cycle' else .014)
    assert core.state==State.FAULT
    assert not [c for c in backend.calls if c[0]=='send']
    assert backend.calls[-1]==('stop',)
    core.tick(.515)
    assert backend.calls[-1]==('stop',)


def test_disable_closes_and_reenable_has_no_old_target():
    core,backend=armed();core.disable()
    assert backend.calls[-2:]==[('stop',),('close',)]
    other=FakeBackend(lambda:2.)
    core.enable(lambda:other,2.,200.)
    assert core.target==(0,0)
    core.tick(2.004)
    assert other.calls==[]


def test_stop_failure_does_not_report_disabled_success():
    core,backend=armed()
    def fail():raise RuntimeError('stop unavailable')
    backend.stop=fail
    assert not core.disable()
    assert core.state==State.FAULT
    assert ('close',) in backend.calls


def test_unverified_templates_fail_before_sdk_import():
    root=Path(__file__).resolve().parents[1]/'towergo_real/config'
    for name,fn in [('hardware.unverified.yaml',hardware_contract),('calibration.unverified.yaml',calibration)]:
        with pytest.raises(ValueError,match='Not field-verified'):fn(root/name)
    assert not any(n.startswith('astribot_sdk') for n in sys.modules)


def test_sdk_takeover_prompt_gets_empty_answer_not_eof(monkeypatch):
    import builtins
    from towergo_real.sdk_backend import construct_without_interactive_takeover
    def no_terminal(*args, **kwargs):raise EOFError('no terminal')
    monkeypatch.setattr(builtins,'input',no_terminal)
    class VendorPrompt:
        def __init__(self,**kwargs):
            try:self.took_control=input('Take over?')=='yes'
            except Exception:self.took_control=True  # Actual vendor fallback behavior.
    result=construct_without_interactive_takeover(VendorPrompt,high_control_rights=False)
    assert not result.took_control
    assert builtins.input is no_terminal
    assert not any(n.startswith('astribot_sdk') for n in sys.modules)


def test_sdk_prompt_override_is_restored_after_constructor_failure():
    import builtins
    from towergo_real.sdk_backend import construct_without_interactive_takeover
    original=builtins.input
    def failed():raise RuntimeError('constructor failed')
    with pytest.raises(RuntimeError,match='constructor failed'):
        construct_without_interactive_takeover(failed)
    assert builtins.input is original
