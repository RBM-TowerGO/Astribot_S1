#!/usr/bin/env python3
"""Pure fake SDK tests; never create an Astribot instance."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import tg_pose as p
import tg_teleop as t


class Backend(t.FakeBackend):
    def __init__(self):
        super().__init__()
        self.enter = threading.Event()
        self.stopped = threading.Event()
        self.allow_exit = threading.Event()
        self.calls = []
        self.rights = True

    def has_control(self):
        return self.rights

    def prepare_pose(self):
        self.calls.append('prepare')

    def execute_pose(self, name, cancel):
        self.calls.append(name)
        self.enter.set()
        self.allow_exit.wait(3)

    def stop_pose(self):
        self.calls.append('stop')
        self.stopped.set()


class PoseTests(unittest.TestCase):
    def test_hold_requires_time_and_separate_repeats(self):
        for key, name in [('h', 'home'), ('P', 'pack')]:
            k = t.Keys(); k.feed(key * 100, 1.)
            self.assertEqual(k.pose(1.01), (None, name))
            self.assertEqual(k.pose(1.21), (None, None))
            for i in range(8):
                k.feed(key, 2 + i * .1)
            self.assertEqual(k.pose(2.71), (name, name))
            self.assertEqual(k.sample(2.71), (0., 0., 0.))
            self.assertEqual(k.pose(2.91), (None, None))

    def test_mixed_stop_exit_and_speed_never_extend_pose(self):
        for batch in ['hp', 'wh', 'h ', 'h+', '\x1b[A', 'hx', '+']:
            k = t.Keys()
            for i in range(8): k.feed('h', 1 + .1*i)
            k.feed(batch, 1.71)
            self.assertEqual(k.pose(1.72), (None, None), batch)

    def controller(self):
        b = Backend(); now = [10.]
        c = p.PoseControl(b, clock=lambda: now[0])
        def cleanup():
            b.allow_exit.set()
            try: c.close()
            except RuntimeError: pass
        self.addCleanup(cleanup)
        c.start('home', 10.2)
        self.assertTrue(b.enter.wait(1))
        return b, c, now

    def test_stop_requires_worker_exit_and_quiet_before_resume(self):
        b, c, now = self.controller()
        now[0] = 10.1
        self.assertTrue(c.tick(None, None, 0, 10.1, 10.1))
        self.assertTrue(b.stopped.is_set())
        with self.assertRaises(RuntimeError): c.resume()
        b.allow_exit.set(); c.worker.join(1)
        c.tick(None, None, 0, 10.1, 10.11)
        with self.assertRaises(RuntimeError): c.resume()
        c.tick(None, None, 0, 10.1, 10.5)
        c.resume()
        self.assertEqual(b.calls.count('prepare'), 2)
        self.assertFalse(c.needs_resume)

    def test_stuck_worker_latches_instead_of_restarting(self):
        b, c, now = self.controller()
        c.stop('stop'); now[0] = 12.1
        with self.assertRaisesRegex(RuntimeError, '禁止恢复底盘'):
            c.tick(None, None, 0, 10., now[0])
        with self.assertRaises(RuntimeError): c.resume()
        self.assertEqual(b.calls.count('prepare'), 1)

    def test_control_loss_does_not_send_stop_under_someone_else(self):
        b, c, now = self.controller(); b.rights = False
        c.stop('rights lost')
        b.allow_exit.set(); c.worker.join(1)
        self.assertNotIn('stop', b.calls)

    def test_watchdog_stops_without_main_loop_tick(self):
        b = Backend(); c = p.PoseControl(b)
        try:
            c.start('home', time.monotonic() + .15)
            self.assertTrue(b.enter.wait(1))
            self.assertTrue(b.stopped.wait(1))
            self.assertTrue(c.cancel.is_set())
        finally:
            b.allow_exit.set(); c.close()

    def test_cancel_during_prepare_does_not_launch_pose(self):
        b = Backend(); ready = threading.Event(); release = threading.Event()
        def prepare(): ready.set(); release.wait(1)
        b.prepare_pose = prepare
        c = p.PoseControl(b)
        try:
            c.start('home', time.monotonic() + 1)
            self.assertTrue(ready.wait(1)); c.stop('cancel'); release.set()
            c.worker.join(1)
            self.assertNotIn('home', b.calls)
            self.assertTrue(b.stopped.is_set())
        finally:
            release.set(); b.allow_exit.set(); c.close()

    def test_missing_pack_no_backend_call(self):
        b = Backend(); b.pose_available = lambda name: name == 'home'
        c = p.PoseControl(b)
        try:
            self.assertTrue(c.tick('pack', 'pack', 2., 1., 1.))
            self.assertIn('未配置', c.message)
            self.assertEqual(b.calls, [])
        finally: c.close()

    def test_config_validation(self):
        self.assertIsNone(p.load_pack(None))
        # Synthetic values used only for parsing, never installed as a pose.
        good = dict(source='unit test only', steps=[dict(duration=5.,
                    joints={name: [0.] * n for name, n in zip(p.NAMES, p.DOFS)})])
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'test.json'
            path.write_text(json.dumps(good)); self.assertEqual(len(p.load_pack(path)), 1)
            for field, value in [('duration', .1), ('duration', float('nan')),
                                 ('joints', {'astribot_chassis': [0, 0, 0]})]:
                bad = json.loads(json.dumps(good)); bad['steps'][0][field] = value
                path.write_text(json.dumps(bad))
                with self.assertRaises(ValueError): p.load_pack(path)

    def test_sdk_home_and_pack_rejection(self):
        b = t.SDKBackend.__new__(t.SDKBackend); b.pack = None; b.bot = Mock()
        b.bot.move_to_home.return_value = 'move failed'
        with self.assertRaisesRegex(RuntimeError, '初始姿态失败'):
            b.execute_pose('home', threading.Event())
        with self.assertRaisesRegex(RuntimeError, '未配置'):
            b.execute_pose('pack', threading.Event())
        b.bot.move_joints_position.assert_not_called()

    def test_pack_limits_collision_and_cancel(self):
        commands = {name: [0.] * n for name, n in zip(p.NAMES, p.DOFS)}
        b = t.SDKBackend.__new__(t.SDKBackend)
        b.pack = [dict(duration=5., joints=commands)]
        b.bot = Mock()
        b.bot.get_joints_position_limit.return_value = (
            [[-1.] * n for n in p.DOFS], [[1.] * n for n in p.DOFS])
        native = b.bot.astribot_interface._AstribotInterface__astribot_class
        native.check_to_joint_position.return_value = False
        with self.assertRaisesRegex(RuntimeError, '碰撞'):
            b.execute_pose('pack', threading.Event())
        b.bot.move_joints_position.assert_not_called()
        commands[p.NAMES[0]][0] = 2.
        with self.assertRaisesRegex(RuntimeError, '限位'):
            b.execute_pose('pack', threading.Event())
        commands[p.NAMES[0]][0] = 0.
        cancel = threading.Event()
        def checked(*args, **kwargs): cancel.set(); return True
        native.check_to_joint_position.side_effect = checked
        b.execute_pose('pack', cancel)
        b.bot.move_joints_position.assert_not_called()


if __name__ == '__main__':
    unittest.main(verbosity=2)
