"""No SDK or hardware: slice coordinates, physical rays, evidence and export conventions."""
import base64
import io
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mapping_slice import Engine,Grid,Poses,defaults,png,raw_points


def pose(ns,p=(0,0,0),q=(0,0,0,1)):
    return NS(header=NS(frame_id='world',stamp=NS(sec=ns//10**9,nanosec=ns%10**9)),
              pose=NS(pose=NS(position=NS(**dict(zip('xyz',p))),orientation=NS(**dict(zip('xyzw',q))))))


def config():
    c=defaults();c.update(imu_from_front_T=[0,0,0],base_from_front=np.eye(4).tolist(),
                         body_xy=[-.2,.2,-.2,.2],gravity_samples=1)
    return c


def value(g,x,y):
    cell=np.floor(np.array([x,y])/g.r).astype(int)-g.origin
    if (cell<0).any() or cell[0]>=g.score.shape[1] or cell[1]>=g.score.shape[0]:return -1
    return g.raster()[cell[1],cell[0]]


class SliceTests(unittest.TestCase):
    def test_parameter_validation(self):
        for x in [float('nan'),float('inf'),3]:
            with self.assertRaises(ValueError):defaults(x)
        for x in [0,-1,float('nan'),1]:
            with self.assertRaises(ValueError):defaults(0,x)

    def test_repeated_evidence_and_unknown(self):
        g=Grid(config());o=np.array([[0,0,0.]]);p=np.array([[2,0,0.]])
        g.update(o,p);self.assertEqual(value(g,2,0),-1)
        g.update(o,p);self.assertEqual(value(g,2,0),100)
        self.assertEqual(value(g,1,0),0);self.assertEqual(value(g,1,.5),-1)

    def test_high_rays_never_clear_low_wall(self):
        g=Grid(config());o=np.array([[0,0,0.]]);p=np.array([[2,0,0.]])
        for _ in range(2):g.update(o,p)
        for _ in range(20):g.update(o+[0,0,.2],np.array([[4,0,.2]]))
        self.assertEqual(value(g,2,0),100);self.assertEqual(value(g,3,0),-1)

    def test_confirmed_static_obstacles_stay_occupied(self):
        g=Grid(config());o=np.array([[0,0,0.]])
        for _ in range(2):g.update(o,np.array([[2,0,0.]]))
        for _ in range(30):g.update(o,np.array([[4,0,.04]]))
        self.assertEqual(value(g,2,0),100)

    def test_rear_origin_above_slab_only_clears_intersection(self):
        g=Grid(config());o=np.array([[0,0,.084]]);p=np.array([[4,0,0.]])
        for _ in range(2):g.update(o,p)
        self.assertEqual(value(g,.5,0),-1)
        self.assertEqual(value(g,3,0),0);self.assertEqual(value(g,4,0),100)

    def test_outside_endpoint_not_obstacle_but_crossing_ray_is_observed(self):
        g=Grid(config());o=np.array([[0,0,.2]]);p=np.array([[4,0,-.2]])
        for _ in range(3):g.update(o,p)
        self.assertEqual(value(g,2,0),0)
        self.assertEqual(value(g,4,0),-1);self.assertEqual(value(g,.5,0),-1)

    def test_growth_preserves_negative_coordinates(self):
        g=Grid(config())
        for _ in range(2):g.update(np.array([[0,0,0.]]),np.array([[-3,-2,0.]]))
        for _ in range(2):g.update(np.array([[0,0,0.]]),np.array([[5,4,0.]]))
        self.assertEqual(value(g,-3,-2),100);self.assertEqual(value(g,5,4),100)

    def test_allocation_is_bounded(self):
        c=config();c['max_cells']=4096;g=Grid(c)
        with self.assertRaises(RuntimeError):g.ensure(np.array([[0,0],[100,100]]))

    def test_slerp_and_no_extrapolation(self):
        p=Poses(300000000);p.add(1000000000,[0,0,0],[0,0,0,1],'w')
        p.add(1200000000,[2,0,0],Rotation.from_euler('z',90,degrees=True).as_quat(),'w')
        ok,pos,q=p.interpolate(np.array([900000000,1100000000,1300000000]))
        np.testing.assert_array_equal(ok,[False,True,False]);np.testing.assert_allclose(pos,[[1,0,0]])
        self.assertAlmostEqual(Rotation.from_quat(q).as_euler('xyz',degrees=True)[0,2],45)

    def test_pose_gap_is_not_interpolated(self):
        p=Poses(300000000);p.add(10**9,[0,0,0],[0,0,0,1],'w');p.add(2*10**9,[0,0,0],[0,0,0,1],'w')
        self.assertFalse(p.interpolate(np.array([1500000000]))[0].any())

    def test_gravity_level_fixed_through_robot_tilt(self):
        e=Engine(config());tilt=Rotation.from_euler('x',20,degrees=True)
        up=tilt.apply([0,0,1]);e.imu(NS(linear_acceleration=NS(**dict(zip('xyz',up)))))
        e.pose(pose(10**9));before=e.level.copy()
        np.testing.assert_allclose(e.level@up,[0,0,1],atol=1e-10)
        e.pose(pose(1100000000,q=tilt.as_quat()));np.testing.assert_array_equal(e.level,before)

    def test_both_sensors_share_slice_and_retain_contributions(self):
        e=Engine(config());e.level=np.eye(3);e.anchor=np.zeros(3)
        p=np.array([[3,0,0.],[3,1,.2]])
        for sensor in (0,1):
            for _ in range(2):e.integrate(sensor,p,np.zeros((2,3)),np.array([[0,0,0,1.]]*2))
        self.assertEqual(value(e.grid,3,0),100);self.assertEqual(value(e.grid,3,1),-1)
        self.assertGreater(e.stats['front_hits'],0);self.assertGreater(e.stats['back_hits'],0)

    def test_moving_pose_transforms_point_without_flattening(self):
        e=Engine(config());e.level=np.eye(3);e.anchor=np.zeros(3)
        rot=Rotation.from_euler('z',90,degrees=True);q=rot.as_quat()[None,:]
        for _ in range(2):e.integrate(0,np.array([[3,0,0.]]),np.array([[1,1,0.]]),q)
        self.assertEqual(value(e.grid,1,4),100)

    def test_self_filter_has_no_free_ray(self):
        e=Engine(config());e.level=np.eye(3);e.anchor=np.zeros(3)
        e.integrate(0,np.array([[.1,.1,0.]]),np.zeros((1,3)),np.array([[0,0,0,1.]]))
        self.assertEqual(e.stats['self_filtered'],1);self.assertFalse(e.grid.score.size)

    def test_png_is_top_down_while_ros_grid_is_bottom_up(self):
        from PIL import Image
        g=Grid(config())
        for _ in range(2):g.update(np.array([[0,0,0.]]),np.array([[2,1,0.]]))
        image=np.array(Image.open(io.BytesIO(png(g))))
        a=g.raster();self.assertTrue(np.all(image[::-1][a==100]==0))
        self.assertTrue(np.all(image[::-1][a==-1]==205));self.assertTrue(np.all(image[::-1][a==0]==254))



class SliceLifecycleTests(unittest.TestCase):
    def test_shutdown_stops_slice_before_recorder(self):
        from mapping_support import shutdown_order
        order=shutdown_order({k:None for k in ['slice','adapter','fastlio','teleop','lio_record','raw_record']})
        self.assertLess(order.index('teleop'),order.index('slice'))
        self.assertLess(order.index('slice'),order.index('lio_record'))

    def test_fault_stale_and_lag_are_session_faults(self):
        import json,tempfile,time
        from unittest.mock import Mock,patch
        import tg_mapping as mapping
        from mapping_support import State
        with tempfile.TemporaryDirectory() as tmp:
            session=mapping.Session(NS(replay='/unused',output_root=tmp,mapping_only=True))
            session.signal=None;session.procs={'slice':Mock()};session.procs['slice'].poll.return_value=None
            session.run=Path(tmp);session.state=State('test');session.state.stamps['odom']=10**10
            cases=[({'error':'bad ray'},'bad ray'),
                   ({'stamp_ns':10**10,'worker_updated':time.time()-10},'未更新'),
                   ({'stamp_ns':10**9,'lio_stamp_ns':10**10,'worker_updated':time.time()},'落后')]
            for data,message in cases:
                session.last_slice_read=0;(session.run/'slice_live.json').write_text(json.dumps(data))
                with patch.object(session.state,'healthy',return_value=True):
                    with self.assertRaisesRegex(RuntimeError,message):session.check(require_streams=True)

    def test_raw_timestamps_remain_per_point(self):
        import struct
        fields=[NS(name=k,offset=o,datatype=d,count=1) for k,o,d in
                [('x',0,7),('y',4,7),('z',8,7),('timestamp',16,8),('tag',24,2),('line',25,2)]]
        data=bytearray(64)
        for i in range(2):
            struct.pack_into('<fff',data,i*32,3.,0.,0.)
            struct.pack_into('<d',data,i*32+16,10**9+i*50000000)
        m=NS(width=2,height=1,row_step=64,point_step=32,data=data,fields=fields,is_bigendian=False,
             header=NS(stamp=NS(sec=1,nanosec=0)))
        p,t,bad=raw_points(m);np.testing.assert_array_equal(t,[10**9,1050000000]);self.assertEqual(bad,0)
        struct.pack_into('<d',data,16,2*10**9)
        with self.assertRaises(ValueError):raw_points(m)

    def test_old_products_are_not_overwritten(self):
        import tempfile,json
        from unittest.mock import patch
        from mapping_slice_finalize import finalize_slice
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);out=d/'old';out.mkdir();(out/'map.pgm').write_bytes(b'original')
            (d/'session.json').write_text(json.dumps({'topics':{'odom':'o'}}))
            (d/'slice_config.json').write_text(json.dumps(defaults()))
            with patch('mapping_slice_finalize.check_metadata'):
                with self.assertRaisesRegex(RuntimeError,'already exists'):finalize_slice(d,output=out)
            self.assertEqual((out/'map.pgm').read_bytes(),b'original')

if __name__=='__main__':unittest.main()
