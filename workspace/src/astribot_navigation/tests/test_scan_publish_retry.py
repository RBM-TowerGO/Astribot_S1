"""Execute production scan publishing methods with ROS-free transport stubs."""
import ast,math,sys,time
from pathlib import Path
from types import SimpleNamespace as S,MethodType
from collections import deque
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from real_navigation.pipeline import Metrics,match_scans
from real_navigation.geometry import scan_points

class Time:
    def __init__(self,nanoseconds):self.ns=nanoseconds
    def to_msg(self):return self.ns
class Publisher:
    def __init__(self):self.rows=[]
    def publish(self,m):self.rows.append(m)

def setup():
    tree=ast.parse((ROOT/'scripts/real_navigation/sensors.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Sensors')
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('flush','publish_cached_scan')]
    for fn in methods:fn.body=[n for n in fn.body if not isinstance(n,(ast.Import,ast.ImportFrom))]
    env=dict(time=time,math=math,np=np,rclpy=S(time=S(Time=Time)),
        point_cloud2=S(create_cloud_xyz32=lambda *a:None),Header=S,
        LaserScan=lambda:S(header=S()),FRAME={'front':'front','back':'back'},
        scan_points=scan_points,match_scans=match_scans)
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods,type_ignores=[])),'production-scan','exec'),env)
    ranges=np.full(720,np.nan);ranges[360]=2.
    batch=[];clock=[10.35]
    n=S(worker=S(take=lambda:(batch[:],None)),error=None,
        geometry=S(continuity_epoch=0,at=lambda t:np.eye(4),base_lidar=np.eye(4)),
        metrics=Metrics(),node=S(get_clock=lambda:S(now=lambda:S(nanoseconds=int(clock[0]*1e9)))),
        cloud_pub={k:Publisher() for k in ('front','back')},obstacles={},recovery_clouds={},
        scans={k:(10.1,ranges) for k in ('front','back')},
        scan_history={k:deque([(10.1,ranges)],maxlen=6) for k in ('front','back')},
        latest_scan_source_stamp=10.,last_pose=(10.3,np.eye(4)),last_pose_stamp_ns=10300000000,
        latest_scan=(10.2,[]),scan_pub=Publisher(),received={},stamps={})
    n.fail=lambda text:setattr(n,'error',text)
    for name in ('flush','publish_cached_scan'):setattr(n,name,MethodType(env[name],n))
    def result(key,stamp=10.2,epoch=0):
        values=ranges.copy()
        if key=='back':values[:]=np.nan;values[0]=3.
        return key,stamp,np.empty((0,3)),values,epoch,0.
    return n,batch,clock,result

@pytest.mark.parametrize('order',[('front','back'),('back','front')])
def test_batch_selects_newest_pair_once_preserving_both_sensors(order):
    n,batch,clock,result=setup();batch.extend(result(k) for k in order);n.flush()
    assert n.error is None
    assert n.latest_scan_source_stamp==10.2
    assert len(n.scan_pub.rows)==1
    values=n.scan_pub.rows[0].ranges
    assert values[360]==pytest.approx(2.) and values[0]==pytest.approx(3.)
    assert n.stamps['scan']==10.2 and n.latest_scan[0]==10.3

def test_cached_pair_retries_without_any_new_cloud():
    n,batch,clock,result=setup();n.latest_scan=(10.3,[])
    batch.extend(result(k) for k in ('front','back'));n.flush()
    assert not n.scan_pub.rows
    batch.clear();clock[0]=10.45;n.last_pose=(10.4,np.eye(4));n.last_pose_stamp_ns=10400000000
    n.flush()
    assert len(n.scan_pub.rows)==1 and n.latest_scan_source_stamp==10.2
    assert n.latest_scan[0]==10.4
    for _ in range(10):n.flush()
    assert len(n.scan_pub.rows)==1  # Never refresh the age of an old source pair.

def test_cache_expires_while_waiting_no_timestamp_renewal():
    n,batch,clock,result=setup();n.latest_scan=(10.3,[])
    batch.extend(result(k) for k in ('front','back'));n.flush()
    batch.clear();clock[0]=10.71;n.last_pose=(10.6,np.eye(4));n.last_pose_stamp_ns=10600000000
    n.flush();assert not n.scan_pub.rows
    assert n.latest_scan_source_stamp==10.

def test_missing_source_pose_can_retry_but_never_publish_partial_scan():
    n,batch,clock,result=setup()
    n.geometry.at=lambda t:None if t==10.2 else np.eye(4)
    batch.extend(result(k) for k in ('front','back'));n.flush()
    assert not n.scan_pub.rows
    batch.clear();n.geometry.at=lambda t:np.eye(4);n.flush()
    assert len(n.scan_pub.rows)==1 and n.latest_scan_source_stamp==10.2

def test_adjacent_frames_cannot_be_combined():
    n,batch,clock,result=setup();n.latest_scan_source_stamp=10.1
    batch.extend([result('front',10.2),result('back',10.301)]);n.flush()
    assert not n.scan_pub.rows
    batch[:]=[result('back',10.201)];n.flush()
    assert len(n.scan_pub.rows)==1 and n.latest_scan_source_stamp==10.2

def test_old_epoch_results_and_error_cannot_publish():
    n,batch,clock,result=setup();n.scans.clear()
    for h in n.scan_history.values():h.clear()
    batch.extend(result(k,epoch=-1) for k in ('front','back'));n.flush()
    assert not n.scan_pub.rows
    assert not any(n.scan_history.values())
    batch[:]=[result(k) for k in ('front','back')];n.error='history interrupted';n.flush()
    assert not n.scan_pub.rows

def test_reference_cannot_be_extrapolated():
    n,batch,clock,result=setup();n.latest_scan_source_stamp=10.1
    n.last_pose=(10.15,np.eye(4));n.latest_scan=(10.1,[])
    batch.extend(result(k) for k in ('front','back'));n.flush()
    assert not n.scan_pub.rows
    batch.clear();n.last_pose=(10.3,np.eye(4));n.flush()
    assert len(n.scan_pub.rows)==1
