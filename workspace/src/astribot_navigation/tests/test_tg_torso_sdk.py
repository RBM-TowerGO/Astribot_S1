#!/usr/bin/env python3
"""Installed SDK compatibility fixture; no robot/ROS/control constructors.

Run with the SDK import environment. Model measurements are synthetic, and the
facade has no node, publisher, command interface or control-rights interface.
"""
import sys
from pathlib import Path
from types import MethodType, SimpleNamespace
import unittest
import time
import copy
import json
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import tg_teleop as t
try:
    from astribot_sdk.core.common.util.astribot_function import AstribotFunction
    from astribot_sdk.core.common.astribot_base import AstribotBase
except ImportError as exc:
    raise unittest.SkipTest('requires installed SDK import environment: ' + str(exc))


class FilterFixture(AstribotBase):
    """Real compiled set/filter methods; downstream control is a memory sink."""
    def __init__(self):
        super().__init__()  # names only; never call AstribotFunction.__init__
        self.hight_level_stop_robot_flag = False
        self.control_frequency = 100.
        self.delta_t = .01
        self.smooth_duration = 1.
        self.filter_scale = .01  # controlled test coefficient, not a field measurement
        self.control_mode = None
        self.received_time = {'joint_position': time.time(), 'time_out_times': 0}
        self.output = {n: np.zeros((d, 1)) for n, d in zip(self.whole_body_names, self.whole_body_dofs)}
        self.last_desired_joints_position_dict = copy.deepcopy(self.output)
        self.robot_dict = {n: SimpleNamespace(joint_interface_=SimpleNamespace(
            GetDesiredJointPosition=lambda name=n: self.output[name].copy(),
            SetDesiredJointPosition=lambda q, interpolate, dt, name=n: self.receive(name, q, interpolate, dt))) for n in self.whole_body_names}
        self._AstribotFunction__control_function_mapping = {'filter': self.filter_control, 'direct': self.direct_control}
        self.writes = []

    def __getattr__(self, name):
        if name in {'set_joints_position', '_AstribotFunction__set_joints_position_init',
                    'filter_control', 'direct_control', 'list_to_numpy', 'joints_position_control', '_AstribotFunction__control_joints'}:
            return MethodType(getattr(AstribotFunction, name), self)
        raise AssertionError('unexpected SDK dependency: ' + name)

    def receive(self, name, q, interpolate, dt):
        # Native set/filter AND joints_position_control run above this boundary.
        # Only the final robot joint interface is replaced, with no publisher.
        assert interpolate is True and dt == .01
        self.writes.append((name,))
        self.output[name] = np.array(q, copy=True)

    def send(self, name, joints, way='filter'):
        # Supply nominal 100 Hz timing without sleeping or creating a worker.
        self.received_time['joint_position'] = time.time() - self.delta_t
        self.set_joints_position(dict(name_list=[name], command_list=[joints],
            control_way=way, use_wbc=False, add_default_torso=False))


class SDKCompatibility(unittest.TestCase):
    def test_sdk_backend_direct_dispatch_reaches_joint_interface_without_filter_lag(self):
        native = FilterFixture()
        def set_joints(names, values, control_way, use_wbc, add_default_torso):
            self.assertFalse(use_wbc); self.assertFalse(add_default_torso)
            self.assertEqual(names, [native.torso_name])
            native.send(names[0], values[0], way=control_way)
        b = t.SDKBackend.__new__(t.SDKBackend)
        b.bot = SimpleNamespace(torso_name=native.torso_name, set_joints_position=set_joints)
        for q in ([.0016, -.0016, 0., 0.], [.0032, -.0032, 0., 0.]):
            b.send_torso(q)
            np.testing.assert_allclose(native.output[native.torso_name].ravel(), q)
        self.assertEqual(native.writes, [(native.torso_name,), (native.torso_name,)])
        np.testing.assert_array_equal(native.output[native.chassis_name], np.zeros((3, 1)))

    def test_compiled_torso_fk_is_read_only_and_returns_chassis_frame(self):
        q = [.29, -.58, .29, 0.]
        pose = [0., 0., 1.2, 0., 0., 0., 1.]
        calls = []
        class FKFixture(AstribotBase):
            def __init__(self):
                super().__init__()
                def fk(joints):
                    calls.append(np.array(joints).ravel().tolist())
                    return np.array(pose).reshape(7, 1)
                self.torso = SimpleNamespace(kinodynamic_=SimpleNamespace(
                    CalForwardKinematicsWithQuaternionPose=fk))
            def __getattr__(self, name):
                if name in {'FK', 'cal_whole_body_forward_kinematics_numpy', 'list_to_numpy', 'numpy_to_list'}:
                    return MethodType(getattr(AstribotFunction, name), self)
                raise AssertionError('unexpected FK dependency: '+name)
        result = json.loads(AstribotFunction.get_forward_kinematics(FKFixture(),
                            {'name_list':['astribot_torso'], 'joints_list':[q]}))
        self.assertEqual(result, {'astribot_torso': pose})
        self.assertEqual(calls, [q])

    def test_filter_advances_per_call_and_chassis_does_not_advance_torso_filter(self):
        native = FilterFixture()
        target = np.array([.03, -.03, .03, 0.]).reshape(4, 1)
        native.send(native.torso_name, target.ravel().tolist())
        first = native.output[native.torso_name].copy()
        np.testing.assert_allclose(first, target * .01)
        for _ in range(50): native.send(native.chassis_name, [0., 0., 0.])
        np.testing.assert_array_equal(native.output[native.torso_name], first)
        # Waiting / sending only chassis cannot advance torso's stored filter.
        for _ in range(150): native.send(native.torso_name, target.ravel().tolist())
        self.assertLess(np.max(abs(native.output[native.torso_name] - target)), .007)

    def test_height_wait_resends_through_installed_sdk_filter_and_recovers(self):
        from tg_torso import TorsoHeight
        native = FilterFixture()
        b = t.FakeBackend()
        b.torso_state = lambda: ([0., 0., 1.2 + native.output[native.torso_name][0, 0] / 8,
                                  0., 0., 0., 1.], native.output[native.torso_name].ravel().tolist())
        b.solve_torso = lambda pose: [(pose[2] - 1.2) * 8, 0., 0., 0.]
        b.send_torso = lambda joints: native.send(native.torso_name, joints)
        now = [1.]
        h = TorsoHeight(b, clock=lambda: now[0])
        waited = recovered = False
        for _ in range(500):
            now[0] += .01
            # Match the teleop loop's chassis hold followed by torso command.
            native.send(native.chassis_name, [0., 0., 0.])
            previous = list(h.joints) if h.joints else None
            h.tick(-.01, now[0] + .2)
            self.assertFalse(h.blocked)
            if h.wait_since is not None:
                waited = True
                self.assertEqual(h.joints, previous)
                self.assertEqual(native.writes[-1], (native.torso_name,))
            elif waited:
                recovered = True
        self.assertTrue(waited)
        self.assertTrue(recovered)
        self.assertLess(h.pose[2], 1.19)


    def test_native_missing_name_map_and_dedicated_read_only_fallback(self):
        class Fixture(AstribotBase):
            def __init__(self):
                # This base only assigns names; it does not connect to ROS.
                super().__init__()
                self.cartesian_dof = dict(zip(self.whole_body_names, self.whole_body_cartesian_dofs))
                del self.cartesian_dof[self.torso_name]  # controlled missing-name-map fixture
                self.torso = SimpleNamespace(kinodynamic_=SimpleNamespace(
                    GetCurrentPoseWeldToTool=lambda: np.array(
                        [0., 0., 1.2, 0., 0., 0., 1.]).reshape(7, 1)))

            def __getattr__(self, name):
                # Only pure getters/conversions used by these compiled methods.
                allowed = {'get_cartesian_pose_by_names', 'pose_to_list',
                           'get_pose_chassis_to_torso_current', 'numpy_to_list'}
                if name in allowed:
                    return MethodType(getattr(AstribotFunction, name), self)
                raise AssertionError('unexpected SDK dependency: ' + name)

            def get_current_joint_position_list(self, names):
                dofs = dict(zip(self.whole_body_names, [3, 4, 7, 1, 7, 1, 2]))
                return [[0.] * dofs[name] for name in names]

            def get_pose_world_to_robot_current_dict(self, **kw):
                return {name: [0., 0., 1.2, 0., 0., 0., 1.] for name in self.whole_body_names}

        native = Fixture()
        def generic(names, frame):
            return AstribotFunction.get_cartesian_pose(native, {
                'pose_type': 'current_cartesian_pose_from_chassis', 'name_list': names})
        with self.assertRaises(KeyError) as err:
            generic(['astribot_torso'], 'chassis')
        self.assertEqual(err.exception.args, ('astribot_torso',))
        bot = SimpleNamespace(torso_name='astribot_torso',
            get_current_cartesian_pose=generic,
            get_current_joints_position=lambda names: [[.29, -.58, .29, 0.]],
            astribot_interface=SimpleNamespace(_AstribotInterface__astribot_class=native))
        backend = t.SDKBackend.__new__(t.SDKBackend)
        backend.bot = bot
        self.assertEqual(backend.torso_state(),
                         ([0., 0., 1.2, 0., 0., 0., 1.], [.29, -.58, .29, 0.]))


if __name__ == '__main__':
    unittest.main(verbosity=2)
