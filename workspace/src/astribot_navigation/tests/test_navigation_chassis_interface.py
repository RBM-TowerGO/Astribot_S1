"""All SDK calls go to a recording fake; no ROS or robot imports."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from real_navigation.chassis_interface import ChassisInterface
from real_navigation.vendor_stop import stop_chassis

class SDK:
    chassis_name = 'chassis'
    def __init__(self): self.calls=[]; self.rights=True
    def get_control_rights_status(self): return self.rights
    def get_desired_joints_position(self,names): return [[1.,2.,1.57]]
    def get_current_joints_position(self,names): return [[1.,2.,1.57]]
    def set_joints_position(self,*args,**kwargs): self.calls.append((args,kwargs))
    def stop_robot(self): self.calls.append('stop_robot')

def test_teleop_contract_and_no_constructor_command():
    sdk=SDK(); c=ChassisInterface(sdk,stop_chassis)
    assert c.desired()==c.feedback()==[1.,2.,1.57]
    assert sdk.calls==[]
    c.send([1.001,2.,1.571])
    assert sdk.calls==[((['chassis'],[[1.001,2.,1.571]]),dict(control_way='filter',use_wbc=False,add_default_torso=False))]

def test_stop_once_and_no_automatic_resume():
    sdk=SDK(); c=ChassisInterface(sdk,stop_chassis); c.send(c.desired())
    c.stop(); c.stop()
    assert sdk.calls.count('stop_robot')==1
    with pytest.raises(RuntimeError): c.send(c.desired())

def test_unused_session_does_not_stop_whole_robot():
    sdk=SDK(); c=ChassisInterface(sdk,stop_chassis); c.stop()
    assert sdk.calls==[]
    assert not c.stopped
    c.send(c.desired())
    assert len(sdk.calls)==1

def test_control_loss_never_sends_even_stop():
    sdk=SDK(); c=ChassisInterface(sdk,stop_chassis); c.send(c.desired()); sdk.calls=[]; sdk.rights=False
    with pytest.raises(RuntimeError): c.send(c.desired())
    with pytest.raises(RuntimeError): c.stop()
    assert sdk.calls==[]

@pytest.mark.parametrize('target', [[float('nan'),0,0],[0,0],[float('inf'),0,0]])
def test_invalid_targets(target):
    sdk=SDK(); c=ChassisInterface(sdk,stop_chassis)
    with pytest.raises(ValueError): c.send(target)
    c.stop(); assert sdk.calls==[]

def test_failed_stop_stays_latched():
    sdk=SDK()
    def fail(_): raise RuntimeError('SDK timeout')
    c=ChassisInterface(sdk,fail); c.send(c.desired())
    with pytest.raises(RuntimeError): c.stop()
    with pytest.raises(RuntimeError): c.send(c.desired())
    c.stop()

from types import SimpleNamespace
from real_navigation.executor import handover

class Transfer:
    def __init__(self, ready=True, reply='success', reply_at=.1, release_at=.2):
        self.ready=ready;self.reply=reply;self.reply_at=reply_at;self.release_at=release_at
        self.t=0.;self.calls=0;self.cancelled=False
    def service_is_ready(self): return self.ready and self.t<self.release_at
    def call_async(self, request): self.calls+=1;return self
    def done(self): return self.t>=self.reply_at
    def result(self): return SimpleNamespace(response=self.reply)
    def cancel(self): self.cancelled=True
    def pump(self): self.t+=.05
    def run(self): handover(self,object(),self.pump,lambda:self.t,timeout=.5)

def test_no_owner_requires_no_handover():
    t=Transfer(ready=False);t.run();assert t.calls==0

def test_handover_waits_for_release_after_ack():
    t=Transfer();t.run();assert t.calls==1;assert t.t>=.2

def test_handover_reports_vendor_rejection():
    t=Transfer(reply='Internal failure')
    with pytest.raises(RuntimeError,match='Internal failure'): t.run()

def test_handover_reply_timeout():
    t=Transfer(reply_at=100)
    with pytest.raises(RuntimeError,match='响应超时'): t.run()
    assert t.cancelled

def test_handover_success_without_release_is_not_ownership():
    t=Transfer(release_at=100)
    with pytest.raises(RuntimeError,match='未释放'): t.run()

@pytest.mark.parametrize('reply', ['success', 'Internal failure'])
def test_executor_initialization_with_fake_ros_and_sdk(monkeypatch, reply):
    import real_navigation.executor as execution
    from types import ModuleType
    transfer=Transfer(reply=reply,reply_at=1.3,release_at=1.4)
    events=[]
    class Lock:
        def __enter__(self): events.append('lock');return self
        def __exit__(self,*_): events.append('unlock')
    class Bot(SDK):
        def __init__(self,**kwargs):
            super().__init__();events.append('sdk')
            assert transfer.t>=transfer.release_at
            assert kwargs['high_control_rights'] is False
            assert kwargs['freq']==100.
            self.astribot_interface=SimpleNamespace(shutdown=lambda:events.append('shutdown'))
    class Node:
        def create_client(self,*args): return transfer
        def destroy_client(self,c): events.append('client_closed')
        def create_subscription(self,*args): events.append('subscribed');return object()
    transfer.wait_for_service=lambda **kwargs: True
    def module(name,**attrs):
        m=ModuleType(name);m.__dict__.update(attrs);monkeypatch.setitem(sys.modules,name,m)
    module('tg_teleop',SessionLock=Lock,conflicts=lambda:[])
    module('rclpy',spin_once=lambda *a,**k:transfer.pump())
    module('rclpy.qos',qos_profile_sensor_data=object())
    module('astribot_msgs.srv',RawRequest=SimpleNamespace(Request=object))
    module('astribot_msgs.msg',RobotJointState=object)
    module('astribot_sdk.core.astribot_api.astribot_client',Astribot=Bot)
    module('towergo_real.sdk_backend',construct_without_interactive_takeover=lambda cls,**kw:cls(**kw))
    monkeypatch.setattr(execution.time,'monotonic',lambda:transfer.t)
    actual_handover=execution.handover
    monkeypatch.setattr(execution,'handover',lambda c,r,p:actual_handover(c,r,p,now=lambda:transfer.t))
    if reply!='success':
        with pytest.raises(RuntimeError,match='Internal failure'):execution.SDK({'sdk_root':'/nonexistent'},Node())
        assert 'sdk' not in events
        assert events[-1]=='unlock'
    else:
        backend=execution.SDK({'sdk_root':'/nonexistent'},Node())
        assert events.index('client_closed')<events.index('sdk')
        assert backend.desired()==[1.,2.,1.57]
        assert backend.bot.calls==[]
        backend.close();assert events[-2:]==['shutdown','unlock']

@pytest.mark.parametrize('replay,publishers,expected,calls', [
    (False,0,'requested',1),(False,1,'existing',0),(True,0,'replay',0)])
def test_navigation_sensor_startup_reuses_mapping(monkeypatch,replay,publishers,expected,calls):
    import tg_mapping
    from real_navigation.session import start_live_sensors
    called=[]
    monkeypatch.setattr(tg_mapping,'lidar_control',lambda node,on,pump:called.append(on) or 'Lidar is now ON')
    node=SimpleNamespace(count_publishers=lambda topic:publishers)
    assert start_live_sensors(node,replay,lambda:None)==expected
    assert called==[True]*calls

@pytest.mark.parametrize('available,success,complete,error', [
    (True,True,True,None),(False,True,True,'unavailable'),
    (True,False,True,'vendor failure'),(True,True,False,'timed out')])
def test_shared_lidar_service_results(monkeypatch,available,success,complete,error):
    import tg_mapping
    from types import ModuleType
    clock=[0.];requests=[];destroyed=[]
    future=SimpleNamespace(done=lambda:complete,result=lambda:SimpleNamespace(success=success,message='OK' if success else 'vendor failure'))
    client=SimpleNamespace(wait_for_service=lambda **kw:available,call_async=lambda req:requests.append(req.data) or future)
    node=SimpleNamespace(create_client=lambda *a:client,destroy_client=lambda c:destroyed.append(c))
    module=ModuleType('std_srvs.srv');module.SetBool=SimpleNamespace(Request=SimpleNamespace)
    monkeypatch.setitem(sys.modules,'std_srvs.srv',module)
    monkeypatch.setattr(tg_mapping.time,'monotonic',lambda:clock[0])
    def pump(): clock[0]+=1.
    if error:
        with pytest.raises(RuntimeError,match=error):tg_mapping.lidar_control(node,True,pump=pump)
    else: assert tg_mapping.lidar_control(node,True,pump=pump)=='OK'
    assert destroyed==[client]
    assert requests==([True] if available else [])
