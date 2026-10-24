"""No hardware imports, control rights or ROS graph required."""
import copy
import math
from pathlib import Path
import sys
import unittest
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'towergo_real')]
from real_navigation.core import Lease,Drive,speed,limited,controller_parameters,validate_config
from real_navigation.geometry import Geometry,matrix,safe_pose


class Backend:
    def __init__(self):self.p=[0.,0.,1.57];self.now=0.;self.rights=True;self.sent=[];self.stops=0
    def desired(self):return self.p[:]
    def feedback(self):return self.p[:],self.now
    def has_control(self):return self.rights
    def send(self,p):self.sent.append(p[:]);self.p=p[:]
    def stop(self):self.stops+=1


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.config=yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text())
        self.config['max_speed']=.3

    def test_tracking_warning_records_errors_without_latching(self):
        b,d=self.drive();b.now=.01;d.tick(.01)
        b.p[0]-=.12;b.now=.02
        before=d.target[:];d.tick(.02)
        self.assertIsNone(d.fault);self.assertIn('偏差',d.warning)
        self.assertAlmostEqual(d.tracking['position_error_m'],.12)
        self.assertEqual(d.target,before);self.assertEqual(b.stops,0)

    def test_stationary_cancel_allows_only_explicit_new_task(self):
        b,d=self.drive();b.now=.01;d.command('t',0.,0.,.01)
        self.assertTrue(d.cancel_stationary(.01,True))
        self.assertIsNone(d.task);self.assertIsNone(d.fault)
        self.assertFalse(d.command('t',.05,0.,.02))
        with self.assertRaises(ValueError):d.prepare('early',.05)
        for now in (.02,.08,.14,.18):b.now=now;d.tick(now)
        self.assertFalse(d.holding);self.assertEqual(b.stops,0)
        d.prepare('new',.05);d.permit('new',.19);d.command('new',.05,0.,.19)
        b.now=.19;d.tick(.19);self.assertEqual(len(b.sent),1)

    def test_data_cancel_with_bad_feedback_or_rights_still_stops(self):
        for kind in ('stale','lost_rights'):
            b,d=self.drive();b.now=.01
            if kind=='stale':b.now=-1.
            if kind=='lost_rights':b.rights=False
            self.assertFalse(d.cancel_stationary(.01,False))
            self.assertIsNone(d.task);self.assertIsNotNone(d.fault)
            self.assertEqual(b.stops,0 if kind=='lost_rights' else 1)

    def test_speed_boundaries(self):
        for x in (0,.001,.02,.3):self.assertEqual(speed(x),x)
        for x in (-.001,.301,float('nan'),float('inf'),True):
            with self.assertRaises(ValueError):speed(x)

    def test_zero_blocks_rotations_and_goal(self):
        self.assertEqual(limited(.2,.1,0),(0,0))
        with self.assertRaises(ValueError):controller_parameters(0)

    def test_small_speed_no_internal_floor(self):
        p=controller_parameters(.001)
        self.assertLessEqual(p['FollowPath.min_approach_linear_velocity'],.001)
        self.assertLessEqual(p['FollowPath.regulated_linear_scaling_min_speed'],.001)

    def test_curvature_preserved(self):
        v,w=limited(.6,.3,.3)
        self.assertAlmostEqual(v/w,2)
        self.assertAlmostEqual(w,.1)

    def test_reverse_rejected(self):
        with self.assertRaises(ValueError):limited(-.1,0,.3)

    def test_owner_exclusion(self):
        lease=Lease();lease.begin('a'*32,0)
        with self.assertRaises(ValueError):lease.claim('b'*32,.1)

    def test_heartbeat_cannot_revive(self):
        lease=Lease();task=lease.begin('a'*32,0)
        self.assertFalse(lease.heartbeat('a'*32,task,2.))
        self.assertFalse(lease.valid(task,2.))

    def test_cancel_invalidates_old_response(self):
        lease=Lease();old=lease.begin('a'*32,0);lease.stop();new=lease.begin('a'*32,.1)
        self.assertNotEqual(old,new)
        self.assertFalse(lease.valid(old,.2));self.assertFalse(lease.heartbeat('a'*32,old,.2))

    def drive(self):
        b=Backend();d=Drive(b,self.config,0);d.prepare('t',.3);d.permit('t',0);d.command('t',.1,.05,0)
        return b,d

    def test_local_coordinates_no_yaw_rotation(self):
        b,d=self.drive();b.now=.01;d.tick(.01)
        self.assertAlmostEqual(b.p[0],.001);self.assertEqual(b.p[1],0)
        self.assertAlmostEqual(b.p[2],1.5705)

    def test_no_catchup(self):
        b,d=self.drive();b.now=.2;d.tick(.2)
        self.assertIsNone(d.fault);self.assertIn('延迟',d.warning);self.assertEqual(b.sent,[]);self.assertEqual(b.stops,0)

    def test_timeout(self):
        b,d=self.drive()
        for t in (.05,.1,.15,.2,.25,.31):b.now=t;d.tick(t)
        self.assertIn('指令超时',d.warning);self.assertIsNone(d.fault);self.assertIsNone(d.task)

    def test_control_loss_no_send_or_stop(self):
        b,d=self.drive();b.rights=False;b.now=.01;d.tick(.01)
        self.assertTrue(d.fault);self.assertEqual(b.sent,[]);self.assertEqual(b.stops,0)

    def test_stale_feedback(self):
        b,d=self.drive();d.last_tick=.15;d.tick(.16)
        self.assertIn('反馈',d.fault)

    def test_following_error(self):
        b,d=self.drive();b.p[0]=-.2;b.now=.01;d.tick(.01)
        self.assertIsNone(d.fault);self.assertAlmostEqual(d.target[0],-.199)

    def test_stop_invalidates_command(self):
        b,d=self.drive();d.stop()
        self.assertFalse(d.command('t',.2,0,.01))

    def test_zero_pause_holds_target_and_resumes(self):
        b,d=self.drive();b.now=.01;d.tick(.01)
        target=d.target[:]
        for now in (.02,.03,.04):
            b.now=now;d.permit("t",now);d.command("t",0.,0.,now)
            self.assertEqual(d.tick(now),(0.,0.))
            self.assertEqual(d.target,target)
            self.assertEqual(b.sent[-1],target)
        self.assertEqual(b.stops,0);self.assertEqual(d.task,"t");self.assertIsNone(d.fault)
        b.now=.05;d.command("t",.1,.05,.05);d.tick(.05)
        self.assertAlmostEqual(d.target[0],target[0]+.001)
        self.assertAlmostEqual(d.target[2],target[2]+.0005)

    def test_initial_zero_does_not_send_existing_sdk_target(self):
        b,d=self.drive();b.now=.01;d.command("t",0.,0.,.01);d.tick(.01)
        self.assertEqual(b.sent,[]);self.assertEqual(b.stops,0)

    def test_pause_still_requires_fresh_commands(self):
        b,d=self.drive();b.now=.01;d.tick(.01)
        d.command("t",0.,0.,.02)
        for now in (.02,.1,.18,.26,.33):
            b.now=now;d.permit("t",now);d.tick(now)
        self.assertIn("指令超时",d.warning);self.assertIsNone(d.fault);self.assertEqual(b.stops,0)
        self.assertIsNone(d.task)

    def test_cancel_during_pause_calls_stop_and_rejects_old_task(self):
        b,d=self.drive();b.now=.01;d.tick(.01)
        b.now=.02;d.command("t",0.,0.,.02);d.tick(.02);d.stop()
        count=len(b.sent)
        self.assertEqual(b.stops,1)
        self.assertFalse(d.command("t",.1,0.,.03))
        b.now=.03;d.tick(.03);self.assertEqual(len(b.sent),count)

    def test_config_without_acceptance_metadata(self):
        validate_config(self.config, True)
        for key in ('verified', 'reviewed_by', 'reviewed_at', 'evidence', 'stop_plugin'):
            self.assertNotIn(key, self.config)

    def test_invalid_maximum_speed(self):
        for value in (0, -.1, .301, float('nan')):
            self.config['max_speed'] = value
            with self.assertRaises(ValueError): validate_config(self.config, True)

    def test_bad_rotation_denied(self):
        self.config['base_from_lidar'][0][0]=-1
        with self.assertRaises(ValueError):validate_config(self.config)

    def test_initial_base_origin_and_interpolation(self):
        g=Geometry(self.config)
        for _ in range(100):g.imu([0,0,1])
        a=g.pose(1.,matrix([5,3,0],[0,0,0,1]))
        b=g.pose(1.1,matrix([5.01,3,0],[0,0,0,1]))
        self.assertTrue(np.allclose(a[:3,3],0))
        self.assertTrue(np.allclose(g.at(1.05)[:3,3],b[:3,3]/2))
        self.assertIsNone(g.at(.9))

    def test_clock_reset_fault(self):
        g=Geometry(self.config)
        for _ in range(100):g.imu([0,0,1])
        g.pose(1.,np.eye(4))
        with self.assertRaises(ValueError):g.pose(.9,np.eye(4))

    def test_confirmed_body_envelope_filters_all_heights_preserves_external_points(self):
        g=Geometry(self.config)
        body=np.array([[.38,.32,.81],[.45,-.32,.81],[.38,.32,.10],
                       [.38,.32,1.8],[.45,0.,.81],[-.39,-.39,.81]])
        external=np.array([[.52,.32,.81],[.38,.41,.81],[.38,-.41,.81],
                           [-.41,0.,.81],[.52,-.32,.81]])
        points=(np.vstack([body,external])-g.base_lidar[:3,3])@np.linalg.inv(g.base_lidar[:3,:3]).T
        for rear in (False,True):
            obstacle,_=g.cloud(points,np.eye(4),rear)
            origin=g.base_lidar[:3,3].copy()
            if rear:origin+=g.base_lidar[:3,:3]@np.asarray(self.config['rear_origin_in_front'])
            self.assertTrue(np.allclose(obstacle+origin,external))
        body[:,2]=self.config['slice_height']
        points=(body-g.base_lidar[:3,3])@np.linalg.inv(g.base_lidar[:3,:3]).T
        _,ranges=g.cloud(points,np.eye(4))
        self.assertFalse(np.isfinite(ranges).any())

    def test_invalid_structure_filter_box_rejected(self):
        for box in ({'min':[0,0,1],'max':[1,1,0]}, {'min':[0,0,0],'max':[1,1,float('nan')]}):
            config=copy.deepcopy(self.config);config['self_filter_boxes']=[box]
            with self.assertRaisesRegex(ValueError,'self filter box'):validate_config(config)

    def test_rear_cloud_preserves_ray_origin(self):
        g=Geometry(self.config);points=np.array([[2.,0.,.1],[3.,0.,.1]])
        front,_=g.cloud(points,np.eye(4),False);rear,_=g.cloud(points,np.eye(4),True)
        expected=g.base_lidar[:3,:3]@np.asarray(self.config['rear_origin_in_front'])
        self.assertTrue(np.allclose(front-rear,expected))

class GeometrySafetyTests(unittest.TestCase):
    def test_full_polygon_corner_and_unknown(self):
        from types import SimpleNamespace
        grid=SimpleNamespace(origin=np.array([0.,0.,0.]),rotation=np.eye(2),resolution=.1,width=30,height=30,free=np.ones((30,30),bool))
        footprint=[[.4,.35],[.4,-.35],[-.4,-.35],[-.4,.35]]
        self.assertTrue(safe_pose(grid,[1.5,1.5,0],footprint))
        grid.free[18,18]=False
        self.assertFalse(safe_pose(grid,[1.5,1.5,0],footprint))
        self.assertFalse(safe_pose(grid,[.1,.1,0],footprint))

    def test_per_point_deskew_and_no_extrapolation(self):
        config=yaml.safe_load((ROOT/'scripts/real_navigation/navigation.yaml').read_text())
        config['base_from_lidar']=np.eye(4).tolist();config['imu_from_lidar']=np.eye(4).tolist()
        g=Geometry(config)
        for _ in range(100):g.imu([0,0,1])
        g.pose(1.,np.eye(4));t=np.eye(4);t[0,3]=.1;g.pose(1.1,t)
        points=np.array([[2.,0.,0.],[1.9,0.,0.]])
        result=g.deskew(points,np.array([1000000000,1100000000]),t)
        self.assertTrue(np.allclose(result,[[1.9,0,0],[1.9,0,0]]))
        self.assertIsNone(g.deskew(points,np.array([900000000,1100000000]),t))

if __name__=='__main__':unittest.main()
