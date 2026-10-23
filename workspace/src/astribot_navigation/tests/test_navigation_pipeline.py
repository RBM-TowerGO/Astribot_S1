"""Delayed poses must not starve bounded, full-coverage cloud processing."""
import sys
import threading
import time
from pathlib import Path
import numpy as np
import ast
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from real_navigation.pipeline import CloudWorker,Metrics,match_scans
from real_navigation.geometry import rotate_unit_quaternions
from real_navigation.core import Convergence


def test_localization_must_remain_valid_continuously_before_success():
    gate=Convergence()
    assert not gate.update(True,10.)
    assert not gate.update(True,11.9)
    assert not gate.update(False,12.)
    assert not gate.update(True,12.1)
    assert not gate.update(True,14.)
    assert gate.update(True,14.2)
    gate.reset()
    assert not gate.update(True,20.)


def test_vectorized_deskew_rotation_matches_independent_rotation_library():
    from scipy.spatial.transform import Rotation
    rng=np.random.default_rng(7)
    rotations=Rotation.random(512,random_state=rng)
    points=rng.normal(size=(512,3))
    np.testing.assert_allclose(rotate_unit_quaternions(rotations.as_quat(),points),
                               rotations.apply(points),atol=1e-12)


def test_idle_scan_gap_preserves_fresh_localization_but_not_invalid_pose():
    tree=ast.parse((Path(__file__).resolve().parents[1]/'scripts/real_navigation/web.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Web')
    block=next(n for n in ast.walk(cls) if isinstance(n,ast.If) and ast.unparse(n.test)=='self.initial_message')
    code=compile(ast.fix_missing_locations(ast.Module(body=[block],type_ignores=[])),'localization-state','exec')
    for valid in [True,False]:
        node=SimpleNamespace(initial_message=None,state={'initialized':True,'sensors_ready':False,'busy':False},
            sensors=SimpleNamespace(error=None),localization={'status':'succeeded'},phase='idle',mode='observe')
        exec(code,dict(self=node,localized=valid,healthy=False,settled=False))
        assert node.state['initialized']==valid
        assert node.localization['status']==('succeeded' if valid else 'recovering')
    node=SimpleNamespace(initial_message=None,state={'initialized':False,'sensors_ready':True,'busy':False},
        sensors=SimpleNamespace(error=None),localization={'status':'recovering'},phase='idle',mode='observe')
    exec(code,dict(self=node,localized=True,healthy=True,settled=True))
    assert node.state['initialized'] and node.localization['status']=='succeeded'
    # No action client or executor exists on the fake: recovery cannot start motion.


def test_pairing_uses_recent_history_not_callback_completion_order():
    history={'front':[(9.8,'f-old'),(10.,'f-new')],'back':[(9.81,'b-old')]}
    pair=match_scans(history,10.1)
    assert pair==((9.8,'f-old'),(9.81,'b-old'))
    assert match_scans(history,10.1,after=9.8) is None
    assert match_scans(history,10.6) is None
    history['back'].append((10.01,'b-new'))
    assert match_scans(history,10.1)[0][1]=='f-new'
    assert match_scans(history,9.) is None


def test_new_frames_do_not_starve_active_frame_waiting_for_lio():
    decoded=threading.Event();pose_ready=threading.Event();release=threading.Event()
    calls=[]
    class Geometry:
        continuity_epoch=0
        def at(self,stamp):return np.eye(4) if pose_ready.is_set() else None
        def deskew(self,raw,stamps,pose):return raw
        def cloud(self,raw,pose,rear):return raw,np.ones(720)
    def decode(msg):
        calls.append(msg)
        if msg==1:decoded.set()
        else:release.wait(1)
        return np.array([[msg,0.,0.]],dtype=np.float32),np.array([1000000000]),0
    worker=CloudWorker(Geometry,decode,Metrics())
    try:
        worker.submit('front',1,0);assert decoded.wait(1)
        worker.submit('front',2,0)
        time.sleep(.025)
        pose_ready.set();worker.wake()
        deadline=time.monotonic()+1
        result=[]
        while time.monotonic()<deadline and not result:
            result,error=worker.take();assert error is None
            time.sleep(.002)
        assert result and result[0][2][0,0]==1
        assert calls.count(1)==1
        assert worker.state()['pending']<=2
    finally:
        release.set();worker.close()


def test_pending_storage_remains_bounded_under_fast_inputs():
    gate=threading.Event();started=threading.Event()
    def decode(msg):
        started.set();gate.wait(1)
        return np.empty((0,3)),np.empty(0,dtype=np.int64),0
    worker=CloudWorker(lambda:None,decode,Metrics())
    try:
        worker.submit('front',0,0);assert started.wait(1)
        for i in range(100):
            worker.submit('front',i,0);worker.submit('back',i,0)
        state=worker.state()
        assert state['pending']<=2 and 1<=state['active']<=4 and state['results']==0
    finally:
        gate.set();worker.close()


def test_front_and_back_processing_can_overlap_without_unbounded_jobs():
    barrier=threading.Barrier(2);finished=threading.Event();calls=[]
    def decode(msg):
        barrier.wait(timeout=1)
        calls.append(msg)
        if len(calls)==2:finished.set()
        return np.empty((0,3)),np.empty(0,dtype=np.int64),0
    worker=CloudWorker(lambda:None,decode,Metrics())
    try:
        with worker.condition:
            worker.submit('front',1,0);worker.submit('back',2,0)
        assert finished.wait(2) and sorted(calls)==[1,2]
        assert worker.error is None and worker.state()['active']<=2
    finally:worker.close()


def test_malformed_timestamp_frame_does_not_poison_next_valid_frame():
    seen=threading.Event()
    def decode(msg):
        seen.set()
        raise ValueError('Invalid slice absolute point timestamp')
    metrics=Metrics();worker=CloudWorker(lambda:None,decode,metrics)
    try:
        worker.submit('front',1,0);assert seen.wait(1)
        deadline=time.monotonic()+1
        while metrics.snapshot()['counters'].get('front_invalid_timestamp_frames',0)<1 and time.monotonic()<deadline:
            time.sleep(.002)
        assert metrics.snapshot()['counters']['front_invalid_timestamp_frames']==1
        assert worker.take()==([],None)
        worker.decode=lambda msg:(np.empty((0,3)),np.empty(0,dtype=np.int64),0)
        worker.submit('front',2,0)
        time.sleep(.02)
        assert worker.error is None
    finally:worker.close()


def test_later_frame_can_finish_while_previous_frame_waits_without_rewinding():
    blocked=threading.Event();release=threading.Event()
    class Geometry:
        continuity_epoch=0
        def at(self,stamp):return np.eye(4)
        def deskew(self,raw,stamps,pose):return raw
        def cloud(self,raw,pose,rear):return raw,np.ones(720)
    def decode(msg):
        if msg==1:blocked.set();release.wait(2)
        return np.array([[msg,0.,0.]]),np.array([msg*1000000000]),0
    metrics=Metrics();worker=CloudWorker(Geometry,decode,metrics)
    try:
        worker.submit('front',1,0);assert blocked.wait(1)
        worker.submit('front',2,0)
        deadline=time.monotonic()+1;results=[]
        while not results and time.monotonic()<deadline:
            results,error=worker.take();assert error is None;time.sleep(.002)
        assert results and results[0][1]==2.
        release.set()
        deadline=time.monotonic()+1
        while worker.state()['active'] and time.monotonic()<deadline:time.sleep(.002)
        assert worker.take()==([],None)
        assert metrics.snapshot()['counters']['front_out_of_order_discarded']==1
    finally:release.set();worker.close()


def test_live_nav2_recovery_holds_zero_then_accepts_new_velocity():
    from real_navigation.core import navigation_command
    import pytest
    assert navigation_command((.05,.1),.05,.05)==(.05,.1)
    for age in [.3,.5,1.,5.,14.99]:
        assert navigation_command((.05,.1),age,.05)==(0.,0.)
    assert navigation_command((.03,0.),.02,.05)==(.03,0.)
    with pytest.raises(RuntimeError,match='反馈超时'):navigation_command((.05,0.),.4,1.)
    with pytest.raises(RuntimeError,match='没有速度输出'):navigation_command((.05,0.),15.,.05)


def test_adjacent_scan_must_not_publish_before_matching_other_side():
    history={'front':[(10.,'front-new')],'back':[(9.901,'back-previous')]}
    assert match_scans(history,10.2) is None
    history['back'].append((10.001,'back-matching'))
    assert match_scans(history,10.2)==((10.,'front-new'),(10.001,'back-matching'))


def test_cached_pose_snapshot_matches_scipy_and_preserves_time_bounds():
    from real_navigation.geometry import PoseSnapshot
    from mapping_slice import Poses
    from scipy.spatial.transform import Rotation,Slerp
    history=Poses(2000000000)
    times=np.array([1,2,3])*1000000000
    rotations=Rotation.from_euler('z',[0.,.7,1.2])
    positions=np.array([[0.,0.,0.],[1.,2.,3.],[2.,1.,4.]])
    for t,p,q in zip(times,positions,rotations.as_quat()):history.add(int(t),p,q,'odom')
    cached=PoseSnapshot(history)
    query=np.array([500000000,1000000000,1500000000,2500000000,3000000000,3500000000])
    valid,p,q=cached.interpolate(query)
    assert valid.tolist()==[False,True,True,True,True,False]
    expected_p=np.column_stack([np.interp(query[valid],times,positions[:,axis]) for axis in range(3)])
    np.testing.assert_allclose(p,expected_p)
    np.testing.assert_allclose(Rotation.from_quat(q).as_matrix(),Slerp(times*1e-9,rotations)(query[valid]*1e-9).as_matrix(),atol=1e-12)
    history.add(4000000000,[10,0,0],[0,0,0,1],'odom')
    assert not cached.interpolate(np.array([4000000000]))[0].any()
    cached.gap=500000000
    assert not cached.interpolate(np.array([1500000000]))[0].any()


def test_scan_latency_tolerance_is_bounded_without_weakening_other_streams():
    from real_navigation.core import sensor_fresh
    assert sensor_fresh('scan',.1,.535)
    assert sensor_fresh('scan',.1,.8)
    assert not sensor_fresh('scan',.1,.801)
    assert not sensor_fresh('scan',.5,.535)  # Actual stream loss is still rejected.
    for key in ['front','back','imu','lio']:
        assert sensor_fresh(key,.1,.135)
        assert not sensor_fresh(key,.1,.535)
        assert not sensor_fresh(key,.5,.1)
    assert not sensor_fresh('scan',.1,float('nan'))
    assert not sensor_fresh('scan',.1,-.1)


def test_browser_lease_tolerates_transport_jitter_but_still_expires():
    from real_navigation.core import Lease
    lease=Lease();task=lease.begin('a'*32,10.)
    assert lease.valid(task,10.6)
    assert lease.heartbeat('a'*32,task,11.2)
    assert lease.valid(task,13.19)
    assert not lease.valid(task,13.2)
    assert not lease.heartbeat('a'*32,task,13.2)
    lease.stop()
    assert not lease.heartbeat('a'*32,task,11.3)


def test_less_conservative_projection_preserves_real_footprint_collision():
    from real_navigation.geometry import safe_pose
    config=__import__('yaml').safe_load((Path(__file__).resolve().parents[1]/'scripts/real_navigation/navigation.yaml').read_text())
    grid=SimpleNamespace(origin=np.array([-.9834156,-1.00368438,0.]),rotation=np.eye(2),resolution=.05,
        width=60,height=60,free=np.ones((60,60),bool))
    for point in [[.44158,-.32868],[.44124,.32132]]:
        x,y=np.floor((np.array(point)-grid.origin[:2])/.05).astype(int);grid.free[y,x]=False
    footprint=np.asarray(config['footprint'])
    assert safe_pose(grid,[0.,0.,0.],footprint,config['footprint_padding'])
    # Snapshot's forward occupied cells used to be inside the 2 s projection.
    assert not safe_pose(grid,[grid.resolution,0.,0.],footprint,.06)
    assert .05*config['collision_time_horizon']>grid.resolution
    assert safe_pose(grid,[grid.resolution,0.,0.],footprint,config['footprint_padding'])
    # Approaching the same cells still produces a collision; they were not erased.
    assert not safe_pose(grid,[.15,0.,0.],footprint,config['footprint_padding'])



def test_settling_window_rejects_motion_stale_gaps_and_clock_reset():
    from real_navigation.stability import Window
    w=Window(.15)
    for i in range(22):w.add(i*.05,[i*.05*.05,0.,0.])
    span=w.span(1.05)
    assert span is not None and span[0]>.049  # 5 cm/s cannot pass 5 mm stability.
    assert w.span(1.3) is None
    w.add(1.4,[0.,0.,0.]);assert w.span(1.4) is None
    w.add(.1,[0.,0.,0.]);assert len(w.rows)==1
    w.add(.15,[float('nan'),0.,0.]);assert not w.rows


def test_settling_window_accepts_stationary_noise_and_averages_derivative_spike():
    from real_navigation.stability import Window
    sdk=Window(.15);lio=Window(.25)
    for i in range(22):sdk.add(i*.05,[.00003*(i%2),0.,.00004*(i%2)])
    assert sdk.span(1.05)[0]<.005 and sdk.span(1.05)[2]<.01
    for i in range(12):lio.add(i*.1,[.09 if i==5 else .015,.005])
    assert lio.mean(1.1)[0]<.03
    moving=Window(.25)
    for i in range(12):moving.add(i*.1,[.05,.0])
    assert moving.mean(1.1)[0]>.03
