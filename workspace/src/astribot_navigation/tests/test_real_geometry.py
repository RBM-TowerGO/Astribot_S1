import math
from types import SimpleNamespace
import numpy as np
import pytest
from towergo_real.geometry import PoseSession, scan_from_body, transform, points_from_cloud


def config():
    t=np.eye(4);t[:3,3]=[.4,0,.2]
    return {'base_from_lidar':t.tolist(),'imu_from_lidar':np.eye(4).tolist(),
            'body_polygon':[[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]],
            'obstacle_height_min':.1,'obstacle_height_max':1.6,'range_min':.5,'range_max':12.,
            'max_pose_step_m':.3,'max_pose_step_rad':.3}


def test_scan_origin_is_lidar_not_base():
    # Input point 2 m in front of IMU/LiDAR, 2.4 m in front of base.
    scan=scan_from_body(np.array([[2.,0,.3]]),config())
    assert scan[360]==pytest.approx(2.)
    assert np.isfinite(scan).sum()==1
    assert np.isnan(scan[0])


def test_filter_body_height_range_invalid_and_nearest():
    points=np.array([[2.,0,.3],[3.,0,.3],[-.4,0,.1],[1.,0,3.],[20.,0,.2],[np.nan,0,0]])
    scan=scan_from_body(points,config())
    assert np.isfinite(scan).sum()==1
    assert scan[360]==2.


def test_extrinsic_lever_arm_changes_base_position_on_turn():
    c=config();session=PoseSession(c)
    session.accept(np.eye(4),10**9,b'one')
    raw=transform([0,0,0],[0,0,math.sin(.1),math.cos(.1)])
    pose,vel=session.accept(raw,11*10**8,b'one')
    assert pose[1,3]==pytest.approx(-.4*math.sin(.2))
    assert abs(vel[1])>0


@pytest.mark.parametrize('problem',['stamp','publisher','jump'])
def test_lio_reset_ends_session(problem):
    s=PoseSession(config());s.accept(np.eye(4),10**9,b'one')
    pose=np.eye(4)
    if problem=='jump':pose[0,3]=5
    with pytest.raises(ValueError):
        s.accept(pose,10**9 if problem=='stamp' else 11*10**8,b'two' if problem=='publisher' else b'one')
    with pytest.raises(ValueError,match='faulted'):s.accept(np.eye(4),12*10**8,b'one')


def test_cloud_row_padding_and_field_validation():
    data=bytearray(40)
    np.ndarray((2,3),dtype='<f4',buffer=data,strides=(20,4))[:]=[[1,2,3],[4,5,6]]
    msg=SimpleNamespace(height=2,width=1,row_step=20,point_step=12,data=data,is_bigendian=False,
      fields=[SimpleNamespace(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')])
    assert np.array_equal(points_from_cloud(msg),[[1,2,3],[4,5,6]])
    msg.fields.pop()
    with pytest.raises(ValueError):points_from_cloud(msg)
