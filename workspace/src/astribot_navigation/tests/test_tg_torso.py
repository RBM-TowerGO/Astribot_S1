#!/usr/bin/env python3
"""No hardware: torso interpolation, SDK contract and rejection paths."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import tg_teleop as t
from tg_torso import TorsoHeight, HeightLimit, LEAD_JOINT, LEAD_Z, DEFAULT_SPEED, HZ


class Backend(t.FakeBackend):
    def __init__(self):
        super().__init__()
        self.calls = []
        self.targets = []

    def torso_state(self):
        pose, joints = super().torso_state()
        pose[0:2] = [.13, -.02]
        pose[3:] = [0., 0., .6, .8]
        return pose, joints

    def solve_torso(self, target):
        self.targets.append(target)
        return super().solve_torso(target)

    def send_torso(self, joints):
        self.calls.append(joints)
        super().send_torso(joints)


class HeightTests(unittest.TestCase):
    def delayed_backend(self, scale=8.):
        b = Backend()
        b.torso_state = lambda: ([0., 0., 1.2 + b.torso_q[0] / scale,
                                  0., 0., 0., 1.], list(b.torso_q))
        b.solve_torso = lambda target: [(target[2] - 1.2) * scale, 0., 0., 0.]
        # Sending changes the setpoint only, not the measured state.
        b.send_torso = lambda joints: b.calls.append(list(joints))
        return b

    def test_slow_feedback_bounds_lead_and_resumes_without_catchup(self):
        for scale, direction in ((1., 1), (1., -1), (8., 1), (8., -1)):
            # Exercise height and joint budgets for both R and F.
            b = self.delayed_backend(scale)
            now = [1.]
            h = TorsoHeight(b, clock=lambda: now[0])
            paused = resumed = False
            for _ in range(600):
                now[0] += .01
                if b.calls:
                    remaining = b.calls[-1][0] - b.torso_q[0]
                    step = scale * .000025
                    b.torso_q[0] += max(-step, min(step, remaining))
                count = len(b.calls)
                previous_z = h.pose[2] if h.pose else 1.2
                h.tick(direction * .01, now[0] + .2)
                if '等待反馈' in h.message:
                    paused = True
                    self.assertIn('等待反馈', h.message)
                    self.assertEqual(h.pose[2], previous_z)
                    self.assertEqual(b.calls[-1], b.calls[-2])
                    self.assertEqual(len(b.calls), count + 1)
                else:
                    resumed |= paused
                    self.assertAlmostEqual(h.pose[2] - previous_z, direction * .0001)
                    self.assertLessEqual(abs(b.calls[-1][0] - b.torso_q[0]), LEAD_JOINT + 1e-12)
                    self.assertLessEqual(abs(h.pose[2] - (1.2 + b.torso_q[0] / scale)), LEAD_Z + 1e-12)
            self.assertTrue(paused)
            self.assertTrue(resumed)

    def test_steady_tracking_offset_does_not_latch_waiting(self):
        # Field read-only sample: received q2 -0.603765, measured -0.590397,
        # about 0.0134 rad offset, larger than the old 0.0125 resume threshold.
        for direction in (-1., 1.):
            b = self.delayed_backend(); now = [1.]
            h = TorsoHeight(b, clock=lambda: now[0])
            goal = [.03]
            offset = .0134
            h.pose, h.joints = [0., 0., 1.2, 0., 0., 0., 1.], [.03, 0., 0., 0.]
            h.wait_since, h.wait_error = 1., 1.
            b.torso_state = lambda: ([0., 0., 1.2+(goal[0]-.03-offset)/8, 0., 0., 0., 1.],
                                     [goal[0]-offset, 0., 0., 0.])
            b.solve_torso = lambda pose: [.03+(pose[2]-1.2)*8, 0., 0., 0.]
            def send(joints):
                b.calls.append(list(joints)); goal[0] = joints[0]
            b.send_torso = send
            for _ in range(300):
                now[0] += .01
                h.tick(direction*.01, now[0]+.2)
                self.assertFalse(h.blocked)
                self.assertIsNone(h.wait_since)
                self.assertIn('腰部升高' if direction > 0 else '腰部降低', h.message)
            self.assertAlmostEqual(h.pose[2], 1.2+direction*.03)
            self.assertEqual(len(b.calls), 300)
            # Releasing must stop all target advancement/repetition immediately.
            h.tick(0., now[0]+.2)
            self.assertEqual(len(b.calls), 300)

    def test_reported_convergence_renews_stall_watchdog(self):
        h = TorsoHeight(self.delayed_backend())
        h.joints = [.03, 0., 0., 0.]
        h.wait_for_feedback(1., [0.]*4, .025, .0028)
        for i, qerr in enumerate([.0243, .0240, .0231, .0219, .0207, .0197,
                                  .0189, .0180, .0176, .0175], 1):
            h.wait_for_feedback(1.+i*.2, [0.]*4, qerr, .002)
            self.assertFalse(h.blocked)
        self.assertGreater(h.wait_since, 1.)

    def test_noise_or_non_limiting_axis_progress_cannot_renew_wait(self):
        for joint_limited in (True, False):
            h = TorsoHeight(self.delayed_backend())
            h.joints = [0.] * 4
            initial = (.024, .008) if joint_limited else (.02, .0096)
            h.wait_for_feedback(1., [0.]*4, *initial)
            # The limiting error improves less than the noise budget; the other
            # error improves substantially, which must not extend the timeout.
            later = (.0238, .001) if joint_limited else (.001, .0095)
            h.wait_for_feedback(2.5, [0.]*4, *later)
            self.assertEqual(h.wait_since, 1.)
            h.wait_for_feedback(3.01, [0.]*4, *later)
            self.assertTrue(h.blocked)

    def test_stalled_feedback_times_out_without_sending_more_targets(self):
        b = self.delayed_backend()
        now = [1.]
        h = TorsoHeight(b, clock=lambda: now[0])
        for _ in range(100):
            now[0] += .01
            h.tick(.01, now[0] + .2)
            if h.wait_since is not None:
                break
        self.assertIsNotNone(h.wait_since)
        count = len(b.calls)
        now[0] += 2.01
        h.tick(.01, now[0] + .2)
        self.assertTrue(h.blocked)
        self.assertIn('反馈连续 2 秒无明显收敛', h.message)
        self.assertEqual(len(b.calls), count)
        h.tick(.01, now[0] + .2)
        self.assertEqual(len(b.calls), count)  # held key cannot auto-retry
        driver = t.Driver(b, now[0])
        driver.tick(now[0] + .01, (.03, 0., 0.))
        self.assertGreater(b.p[0], 0.)  # base remains available
        h.tick(0., now[0] + .2)
        h.tick(-.01, now[0] + .2)
        self.assertFalse(h.blocked)
        self.assertEqual(len(b.calls), count + 1)

    def test_release_or_cancel_during_feedback_wait_never_sends(self):
        for cause in ('release', 'expiry', 'cancel', 'rights'):
            b = self.delayed_backend(); h = TorsoHeight(b, clock=lambda: 1.)
            for _ in range(40): h.tick(.01, 2.)
            self.assertIsNotNone(h.wait_since)
            count = len(b.calls)
            if cause == 'rights':
                b.has_control = lambda: False
                with self.assertRaisesRegex(RuntimeError, '控制权丢失'): h.tick(.01, 2.)
            else:
                h.tick(0. if cause == 'release' else .01,
                       .9 if cause == 'expiry' else 2., allowed=lambda: cause != 'cancel')
                self.assertIsNone(h.wait_since)
                self.assertIsNone(h.pose)
            self.assertEqual(len(b.calls), count)

    def test_held_command_rechecks_collision_and_input_deadline(self):
        for failure in ('collision', 'expiry'):
            b = self.delayed_backend(); now = [1.]
            h = TorsoHeight(b, clock=lambda: now[0])
            for _ in range(40): h.tick(.01, 2.)
            count = len(b.calls)
            def check(joints):
                if failure == 'collision': raise HeightLimit('collision')
                now[0] = 1.01
            b.check_torso = check
            h.tick(.01, 1.005)
            self.assertEqual(len(b.calls), count)
            self.assertIn('受限' if failure == 'collision' else '超时', h.message)

    def test_default_speed_is_two_cm_per_second_target_rate(self):
        self.assertEqual(DEFAULT_SPEED, .02)
        b = Backend(); h = TorsoHeight(b, clock=lambda: 1.)
        for _ in range(int(HZ)): h.tick(DEFAULT_SPEED, 2.)
        self.assertAlmostEqual(h.pose[2], 1.22)

    def test_start_and_restart_continue_last_sdk_target_without_jump(self):
        b = self.delayed_backend(); h = TorsoHeight(b, clock=lambda: 1.)
        command = [.0134, 0., 0., 0.]  # actual is zero; existing command is ahead
        b.torso_reference = lambda: ([0., 0., 1.2+command[0]/8, 0., 0., 0., 1.], list(command))
        def send(joints):
            b.calls.append(list(joints)); command[:] = joints
        b.send_torso = send
        for velocity in (.02, -.02):
            previous = list(command)
            h.tick(velocity, 2.)
            self.assertAlmostEqual(b.calls[-1][0]-previous[0], velocity/HZ*8)
            h.tick(0., 2.)
        self.assertEqual(len(b.calls), 2)
        # Existing goal already outside hard tracking limit must not be sent.
        command[0] = .1
        h.tick(.02, 2.)
        self.assertTrue(h.blocked)
        self.assertEqual(len(b.calls), 2)

    def test_sdk_reference_reads_last_command_and_fk_without_sending(self):
        b = t.SDKBackend.__new__(t.SDKBackend); b.bot = Mock(torso_name='astribot_torso')
        q = [.29, -.58, .29, 0.]
        pose = [0., 0., 1.2, 0., 0., 0., 1.]
        b.bot.get_desired_joints_position.return_value = [q]
        b.bot.get_forward_kinematics.return_value = {'astribot_torso': pose}
        self.assertEqual(b.torso_reference(), (pose, q))
        b.bot.get_forward_kinematics.assert_called_once_with(['astribot_torso'], [q])
        b.bot.set_joints_position.assert_not_called()
        b.bot.get_forward_kinematics.return_value = {}
        with self.assertRaisesRegex(RuntimeError, '正解结果格式无效'): b.torso_reference()

    def test_up_down_preserve_xy_orientation_and_chassis(self):
        b = Backend(); h = TorsoHeight(b, clock=lambda: 1.)
        for speed, z in [(.01, 1.2001), (-.01, 1.2)]:
            h.tick(speed, 2.)
            self.assertAlmostEqual(b.targets[-1][2], z)
            self.assertEqual(b.targets[-1][:2], [.13, -.02])
            self.assertEqual(b.targets[-1][3:], [0., 0., .6, .8])
            self.assertEqual(b.p, [0., 0., 0.])

    def test_release_does_not_advance_and_new_action_uses_feedback(self):
        b = Backend(); h = TorsoHeight(b, clock=lambda: 1.)
        h.tick(.01, 2.); count = len(b.calls)
        h.tick(0., 2.)
        self.assertIsNone(h.pose)
        self.assertEqual(len(b.calls), count)
        b.torso_q[0] = -.1
        h.tick(.01, 2.)
        self.assertAlmostEqual(b.targets[-1][2], 1.1001)

    def test_invalid_speed_and_invalid_feedback(self):
        for speed in [.021, float('nan'), float('inf')]:
            b = Backend(); h = TorsoHeight(b, clock=lambda: 1.)
            with self.assertRaises(RuntimeError): h.tick(speed, 2.)
            self.assertEqual(b.calls, [])
        for pose in [[0.] * 6, [0., 0., float('nan'), 0., 0., 0., 1.], [0.] * 7]:
            b = Backend(); b.torso_state = lambda: (pose, [0.] * 4)
            with self.assertRaises(RuntimeError): TorsoHeight(b).tick(.01, float('inf'))
            self.assertEqual(b.calls, [])

    def test_unreachable_and_branch_jump_never_send(self):
        for result in [None, [1., 0., 0., 0.]]:
            b = Backend(); h = TorsoHeight(b, clock=lambda: 1.)
            def solve(target):
                if result is None: raise HeightLimit('unreachable')
                return result
            b.solve_torso = solve
            h.tick(.01, 2.)
            self.assertEqual(b.calls, [])
            self.assertIn('受限', h.message)
            self.assertEqual(h.pose[2], 1.2)

    def test_feedback_lag_stops_accumulation(self):
        b = Backend(); h = TorsoHeight(b, clock=lambda: 1.)
        h.tick(.01, 2.); b.torso_q[0] = -.1
        h.tick(.01, 2.)
        self.assertTrue(h.blocked)
        self.assertIn('反馈滞后', h.message)
        h.tick(.01, 2.)
        self.assertEqual(len(b.calls), 1)

    def test_expired_input_slow_ik_or_control_loss_never_send(self):
        for cause in ('expiry', 'slow', 'rights', 'cancel'):
            b = Backend()
            times = iter([1., 1.11 if cause == 'slow' else 1.01])
            h = TorsoHeight(b, clock=lambda: next(times))
            if cause == 'rights': b.has_control = lambda: False
            if cause == 'expiry':
                h.tick(.01, 1.005)
            elif cause == 'cancel':
                h.tick(.01, 2., allowed=lambda: False)
            else:
                with self.assertRaises(RuntimeError): h.tick(.01, 2.)
            self.assertEqual(b.calls, [])

    def test_sdk_contract_and_limits(self):
        b = t.SDKBackend.__new__(t.SDKBackend); b.bot = Mock(torso_name='astribot_torso')
        b.bot.get_inverse_kinematics.return_value = (True, {'astribot_torso': [.3, -.6, .3, 0.]})
        self.assertEqual(b.solve_torso([0., 0., 1.2, 0., 0., 0., 1.]), [.3, -.6, .3, 0.])
        b.bot.get_inverse_kinematics.return_value = (False, None)
        with self.assertRaises(HeightLimit): b.solve_torso([0.] * 7)
        b.bot.get_joints_position_limit.return_value = ([[-1.] * 4], [[1.] * 4])
        b.bot.get_self_closest_point.return_value = (.1, None, None, None, None)
        b.check_torso([0.] * 4)
        with self.assertRaises(HeightLimit): b.check_torso([2., 0., 0., 0.])
        b.bot.get_self_closest_point.return_value = (.01, None, None, None, None)
        with self.assertRaises(HeightLimit): b.check_torso([0.] * 4)
        b.send_torso([.3, -.6, .3, 0.])
        b.bot.set_joints_position.assert_called_once_with(
            ['astribot_torso'], [[.3, -.6, .3, 0.]],
            control_way='direct', use_wbc=False, add_default_torso=False)

    def test_sdk_missing_torso_pose_map_uses_read_only_fallback(self):
        b = t.SDKBackend.__new__(t.SDKBackend); b.bot = Mock(torso_name='astribot_torso')
        b.bot.get_current_cartesian_pose.side_effect = KeyError('astribot_torso')
        b.bot.get_current_joints_position.return_value = [[.29, -.58, .29, 0.]]
        native = b.bot.astribot_interface._AstribotInterface__astribot_class
        native.get_pose_chassis_to_torso_current.return_value = [0., 0., 1.2, 0., 0., 0., 1.]
        pose, joints = b.torso_state()
        self.assertEqual(pose, [0., 0., 1.2, 0., 0., 0., 1.])
        self.assertEqual(joints, [.29, -.58, .29, 0.])
        native.get_pose_chassis_to_torso_current.assert_called_once_with()
        b.bot.set_joints_position.assert_not_called()
        b.torso_state()
        self.assertEqual(b.bot.get_current_cartesian_pose.call_count, 1)
        b._torso_pose_compat_noted = False
        b.bot.get_current_cartesian_pose.side_effect = KeyError('other_part')
        with self.assertRaisesRegex(RuntimeError, '位姿读取失败.*other_part'):
            b.torso_state()
        self.assertEqual(native.get_pose_chassis_to_torso_current.call_count, 2)

    def test_sdk_errors_identify_stage_and_keep_cause(self):
        b = t.SDKBackend.__new__(t.SDKBackend); b.bot = Mock(torso_name='astribot_torso')
        b.bot.get_inverse_kinematics.side_effect = KeyError('astribot_torso')
        with self.assertRaisesRegex(RuntimeError, '逆解失败.*KeyError') as err:
            b.solve_torso([0.] * 7)
        self.assertIsInstance(err.exception.__cause__, KeyError)
        b.bot.get_joints_position_limit.side_effect = KeyError('astribot_torso')
        with self.assertRaisesRegex(RuntimeError, '限位读取失败.*KeyError'):
            b.check_torso([0.] * 4)
        b.bot.set_joints_position.assert_not_called()

    def test_keys_height_and_pose_are_exclusive(self):
        k = t.Keys()
        for key in ('r', 'F'):
            k.feed(key, 1.)
            self.assertEqual(k.sample(1.1), (0., 0., 0.))
            self.assertEqual(k.pose(1.1), (None, None))
            self.assertEqual(k.key, key.lower())
            k.sample(1.21); self.assertEqual(k.key, '')
        k.feed('rh', 2.); self.assertEqual(k.key, '')


if __name__ == '__main__':
    unittest.main(verbosity=2)
