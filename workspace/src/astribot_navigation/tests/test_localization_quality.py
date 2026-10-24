import math
from pathlib import Path
import sys
import time
import numpy as np
import pytest
from PIL import Image
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'astribot_nav_sim')]
from astribot_nav_sim.grid_world import GridWorld
from real_navigation.localization import MapMatcher,QualityMonitor,Recovery,transform

@pytest.fixture
def scene(tmp_path):
    # Asymmetric room with two wall recesses, known free interior.
    pixels=np.full((200,240),254,np.uint8)
    pixels[:5]=0;pixels[-5:]=0;pixels[:,:5]=0;pixels[:,-5:]=0
    pixels[5:45,170:190]=0;pixels[145:195,40:65]=0
    Image.fromarray(pixels[::-1]).save(tmp_path/'map.pgm')
    cfg=dict(image='map.pgm',resolution=.05,origin=[-6.,-5.,0.],negate=0,occupied_thresh=.65,free_thresh=.196)
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(cfg))
    grid=GridWorld(tmp_path/'map.yaml');pose=[.3,-.4,.15]
    a=np.linspace(-math.pi,math.pi,360,endpoint=False)
    r=grid.raycast(pose[:2],a+pose[2]);ok=np.isfinite(r)
    points=np.stack([r[ok]*np.cos(a[ok]),r[ok]*np.sin(a[ok])],axis=1)
    return grid,pose,points

def test_quality_rejects_wrong_pose_even_with_small_covariance(scene):
    grid,pose,points=scene;m=MapMatcher(grid)
    assert m.evaluate(points,pose)['good']
    bad=m.evaluate(points,[pose[0]+.8,pose[1]-.8,pose[2]+.25])
    assert not bad['good'] and bad['match_fraction']<.6

def test_unknown_and_wall_interior_are_not_good_matches(scene):
    grid,pose,points=scene;m=MapMatcher(grid)
    assert not m.evaluate(points,[100,100,0])['center_known']
    # Occupied interiors have nonzero distance to an observed wall boundary.
    grid.free[60:100,60:100]=False;grid.occupied[60:100,60:100]=True
    m=MapMatcher(grid);assert m.lookup(np.array([[-2.,-1.]]))[2][0]>.15

def test_ray_crossing_wall_rejected(scene):
    grid,pose,points=scene
    grid.free[85:115,125:130]=False;grid.occupied[85:115,125:130]=True
    result=MapMatcher(grid).evaluate(points,pose)
    assert not result['good'] and result['ray_conflict_fraction']>.1

def good():return dict(good=True,center_known=True,reason=None)

def test_monitor_debounce_stale_and_old_results():
    q=QualityMonitor()
    for t in np.arange(0,3,.2):q.update(float(t),float(t),good(),[0,0,0],True)
    assert q.usable(2.9)
    q.update(1,3,dict(good=False,center_known=False,reason='old'),[100,100,0],True)
    assert not q.invalid
    assert not q.usable(4)
    for t in np.arange(4,6.3,.2):q.update(float(t),float(t),dict(good=False,center_known=True,reason='失配'),[0,0,0],True)
    assert q.invalid and not q.usable(6.2)

def test_unknown_center_and_stationary_drift():
    q=QualityMonitor()
    for t in np.arange(0,1.3,.2):q.update(float(t),float(t),dict(good=False,center_known=False,reason='unknown'),[0,0,0],True)
    assert q.invalid
    q.reset()
    for t in np.arange(0,10,.2):q.update(float(t),float(t),good(),[t*.012,0,0],True)
    assert q.invalid and q.state['stationary_drift_m']>.1

def test_angle_wrap_is_not_drift():
    q=QualityMonitor();q.update(1,1,good(),[0,0,math.pi-.001],True)
    q.update(2,1.2,good(),[0,0,-math.pi+.001],True)
    assert not q.invalid and q.state['stationary_drift_rad']<.003

def test_recovery_once_generation_timeout_and_no_resume():
    r=Recovery();r.begin([0,0,0],0);g=r.generation
    r.observe(1,True,True)
    for t in range(5):r.add(t,[[1,0]])
    assert r.status=='searching'
    assert not r.add(4,[[1,0]])
    assert r.result(g,dict(ok=True,pose=[.1,0,0]))
    assert r.published(5) and not r.published(6)
    assert not r.verify(2,True,4)
    assert not r.verify(3,True,6)
    assert r.verify(6,True,7) and r.status=='succeeded'
    r.begin([0,0,0],10);assert not r.result(g,dict(ok=True,pose=[1,0,0]))
    r.observe(40,True,True);assert r.status=='manual_required'
    assert not hasattr(r,'goal') # Recovery cannot create or resume a motion goal.

def test_recovery_movement_and_missing_anchor():
    r=Recovery();r.begin(None,0);assert r.status=='manual_required'
    r.begin([0,0,0],0);r.observe(1,True,True);r.observe(2,False,True)
    assert r.status=='manual_required'

def test_search_recovers_known_offset(scene):
    grid,pose,points=scene;m=MapMatcher(grid)
    anchor=[pose[0]+.45,pose[1]-.35,pose[2]+.2]
    result=m.search([(i,points.copy()) for i in range(5)],anchor,time.monotonic()+25)
    assert result['ok'],result
    assert math.dist(result['pose'][:2],pose[:2])<.1
    assert abs(result['pose'][2]-pose[2])<math.radians(3)

def test_search_cancel_and_duplicate_frames(scene):
    grid,pose,points=scene;m=MapMatcher(grid)
    assert not m.search([(1,points)]*5,pose,time.monotonic()+1)['ok']
    assert not m.search([(i,points) for i in range(5)],pose,time.monotonic()+1,lambda:True)['ok']

def test_symmetric_room_refuses_ambiguous_yaw(scene):
    grid,pose,points=scene
    # No observed wall edges: no candidate can be accepted despite a supplied pose.
    grid.occupied[:]=False
    result=MapMatcher(grid).search([(i,points) for i in range(5)],pose,time.monotonic()+25)
    assert not result['ok']

def test_expected_update_static_does_not_require_repeated_amcl_messages():
    from real_navigation.localization import UpdateExpectation
    e=UpdateExpectation(.05,.05);e.observe(1,[0,0,0])
    for now in range(600):assert e.valid(now,[.004*math.sin(now),0,.002*math.sin(now)])
    assert e.valid(600,[.051,0,0])
    assert not e.valid(600.6,[.051,0,0])
    e.observe(2,[.051,0,0]);assert e.valid(601,[.051,0,0])
    e.observe(1,[100,100,1]);assert e.odom==[.051,0,0]

def test_expected_update_rotation_deadline_and_reset():
    from real_navigation.localization import UpdateExpectation
    e=UpdateExpectation();assert not e.valid(0,[0,0,0])
    e.observe(1,[0,0,math.pi-.01]);assert e.valid(1,[0,0,-math.pi+.01])
    assert e.valid(2,[0,0,-math.pi+.1]);assert not e.valid(2.6,[0,0,-math.pi+.1])
    e.reset();assert not e.valid(3,[0,0,0])

def test_long_stationary_monitor_checks_every_scan_not_frozen():
    q=QualityMonitor();accepted=0
    for i in range(3001):
        t=i*.2;p=[.002*math.sin(t),.002*math.cos(t),.001*math.sin(2*t)]
        q.update(t,t,good(),p,True)
        if q.usable(t):accepted+=1
    assert q.last_stamp==600 and accepted>2900 and not q.invalid
    bad=dict(good=False,center_known=True,reason='new obstruction/mismatch')
    for i in range(1,13):q.update(600+i*.2,600+i*.2,bad,[0,0,0],True)
    assert q.invalid

@pytest.mark.parametrize('offset',[(.9,-.9,.48),(-.9,.9,-.48)])
def test_search_near_extent_limits(scene,offset):
    grid,pose,points=scene;anchor=(np.array(pose)+offset).tolist()
    result=MapMatcher(grid).search([(i,points.copy()) for i in range(5)],anchor,time.monotonic()+25)
    assert result['ok'],result
    assert math.dist(result['pose'][:2],pose[:2])<.1
    assert abs(result['pose'][2]-pose[2])<math.radians(3)

def test_corridor_translation_ambiguity_is_rejected(tmp_path):
    pixels=np.full((100,600),254,np.uint8);pixels[:5]=0;pixels[-5:]=0
    Image.fromarray(pixels[::-1]).save(tmp_path/'map.pgm')
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.05,origin=[-15,-2.5,0])))
    grid=GridWorld(tmp_path/'map.yaml');a=np.linspace(-math.pi,math.pi,720,endpoint=False)
    ranges=grid.raycast([0,0],a,max_range=5.);ok=np.isfinite(ranges)
    points=np.stack([ranges[ok]*np.cos(a[ok]),ranges[ok]*np.sin(a[ok])],axis=1)
    result=MapMatcher(grid).search([(i,points) for i in range(5)],[0,0,0],time.monotonic()+25)
    assert not result['ok'] and '歧义' in result['reason']

def test_runtime_cancel_invalidates_search_and_blocks_automatic_restart():
    from real_navigation.localization_runtime import LocalizationRuntime
    runtime=LocalizationRuntime.__new__(LocalizationRuntime)
    runtime.generation=0;runtime.recovery=Recovery();runtime.recovery.begin([0,0,0],0)
    generation=runtime.recovery.generation;runtime.cancel()
    assert runtime.recovery.status=='canceled'
    assert not runtime.recovery.result(generation,dict(ok=True,pose=[1,0,0]))
