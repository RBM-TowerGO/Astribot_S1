#!/usr/bin/env python3
"""No hardware: fake backend unit tests and real PTY --dry-run integration."""
import importlib.util
import math
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import tempfile
import termios
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/tg_teleop.py'
LAUNCH = ROOT / 'scripts/tg-teleop.sh'
spec = importlib.util.spec_from_file_location('tg_teleop', SCRIPT)
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)


class Backend(t.FakeBackend):
    def __init__(self):
        super().__init__()
        self.rights, self.broken, self.closed = True, False, False
        self.calls = []

    def has_control(self):
        return self.rights

    def send(self, p):
        if self.broken:
            raise RuntimeError('fake send failed')
        self.calls.append(list(p))
        super().send(p)

    def close(self):
        self.closed = True


class Logic(unittest.TestCase):
    def test_directions_and_case(self):
        for key, velocity in [('W', (.05, 0, 0)), ('S', (-.05, 0, 0)),
                              ('A', (0, .05, 0)), ('D', (0, -.05, 0)),
                              ('Q', (0, 0, .10)), ('E', (0, 0, -.10))]:
            k = t.Keys(); k.feed(key, 1)
            self.assertEqual(k.sample(1.1), velocity)
            self.assertFalse(k.quit)
            self.assertEqual(k.sample(1.2), (0, 0., 0))

    def test_renew_and_restart(self):
        k = t.Keys(); k.feed('w', 1); k.feed('w', 1.15)
        self.assertEqual(k.sample(1.3), (.05, 0., 0))
        self.assertEqual(k.sample(1.36), (0, 0., 0))
        k.feed('e', 2); self.assertEqual(k.sample(2.01), (0, 0., -.1))

    def test_stop_priority_and_unknown(self):
        for batch in ['w ', ' w', 'a ', ' d', '\x1b[A', '?w']:
            k = t.Keys(); k.feed('w', 1); k.feed(batch, 1.05)
            self.assertEqual(k.sample(1.1), (0, 0., 0), batch)
        k = t.Keys(); k.feed('AW', 1)
        self.assertEqual(k.sample(1.1), (.05, 0., 0.))

    def test_exit_priority(self):
        for batch in ['wx', 'Xq', '\x03w', '\x04']:
            k = t.Keys(); k.feed(batch, 1)
            self.assertTrue(k.quit)
            self.assertEqual(k.sample(1.01), (0, 0., 0))

    def test_last_direction_no_queue(self):
        k = t.Keys(); k.feed('wsqe', 1)
        self.assertEqual(k.sample(1.1), (0, 0., -.1))

    def test_speed_bounds_and_lease(self):
        k = t.Keys(); k.feed('w', 1); k.feed('+' * 100, 1.1)
        self.assertEqual((k.linear, k.angular), (1.0, .25))
        self.assertEqual(k.sample(1.21), (0, 0., 0))
        k.feed('-' * 100, 2)
        self.assertEqual((k.linear, k.angular), (.01, .025))
        self.assertEqual(k.sample(2.01), (0, 0., 0))

    def test_invalid_speed(self):
        for v in ['nan', 'inf', '-inf', '0', '-1', '1.001']:
            with self.assertRaises(Exception): t.speed(v, .01, t.MAX_LINEAR_SPEED)

    def test_linear_speed_limit(self):
        self.assertEqual(t.speed('1.0', .01, t.MAX_LINEAR_SPEED), 1.0)
        for v in [-1.0, 1.0]:
            b = Backend(); d = t.Driver(b, 0)
            d.tick(.01, (v, 0., 0))
            self.assertAlmostEqual(b.p[0], v / 100)
        for v in [-1.001, 1.001]:
            b = Backend(); d = t.Driver(b, 0)
            with self.assertRaises(RuntimeError): d.tick(.01, (v, 0., 0))
            self.assertEqual(b.calls, [])

    def test_vendor_local_commands_independent_of_yaw(self):
        # Reference: SDK example 202 integrates local x/y/yaw directly.
        # Native values are not world odometry; sideways commands must stay zero.
        for yaw in [0, math.pi/4, math.pi/2, math.pi, -math.pi/2, 9*math.pi]:
            for v in [-.05, .05]:
                b = Backend(); b.p = [1, 2, yaw]; d = t.Driver(b, 0)
                d.tick(.01, (v, 0., 0))
                self.assertAlmostEqual(b.p[0], 1 + v/100)
                self.assertEqual(b.p[1], 2)
                self.assertEqual(b.p[2], yaw)
        for w in [-.1, .1]:
            b = Backend(); d = t.Driver(b, 0); d.tick(.01, (0, 0., w))
            self.assertEqual(b.p[:2], [0, 0]); self.assertAlmostEqual(b.p[2], w/100)

    def test_lateral_local_direction_limits_and_hold(self):
        for yaw in [0., math.pi/2, -math.pi/2, math.pi]:
            for key, sign in [('a', 1), ('d', -1)]:
                b = Backend(); b.p = [2., 3., yaw]; d = t.Driver(b, 0)
                k = t.Keys(linear=1.); k.feed(key, 0)
                d.tick(.01, k.sample(.01))
                self.assertEqual(b.p, [2., 3. + sign*.01, yaw])
                k.feed(' ', .02); d.tick(.02, k.sample(.02))
                self.assertEqual(b.p, [2., 3. + sign*.01, yaw])
                k.feed(key, .03)
                self.assertEqual(k.sample(.24), (0., 0., 0.))
        for velocity in [(0., 1.001, 0.), (0., -1.001, 0.),
                         (0., float('nan'), 0.), (0., float('inf'), 0.),
                         (1., 1., 0.)]:
            b = Backend(); d = t.Driver(b, 0)
            with self.assertRaises(RuntimeError): d.tick(.01, velocity)
            self.assertEqual(b.calls, [])

    def test_fixed_step_no_catchup_and_hold(self):
        b = Backend(); d = t.Driver(b, 0)
        d.tick(.05, (.05, 0., 0))
        self.assertEqual(b.p, [.0005, 0, 0])
        d.tick(.06, (0, 0., 0)); self.assertEqual(b.p, [.0005, 0, 0])

    def test_idle_does_not_send(self):
        b = Backend(); d = t.Driver(b, 0); d.tick(.01, (0, 0., 0))
        self.assertEqual(b.calls, [])

    def test_faults_latch_without_new_targets(self):
        for cause in ['rights', 'overrun', 'send']:
            b = Backend(); d = t.Driver(b, 0); d.tick(.01, (.05, 0., 0))
            count = len(b.calls)
            if cause == 'rights': b.rights = False
            if cause == 'send': b.broken = True
            with self.assertRaises(RuntimeError):
                d.tick(.2 if cause == 'overrun' else .02, (.05, 0., 0))
            self.assertEqual(len(b.calls), count)
            b.rights, b.broken = True, False
            with self.assertRaises(RuntimeError): d.tick(.03, (.05, 0., 0))

    def test_finish_holds_and_releases(self):
        b = Backend(); d = t.Driver(b, 0); d.tick(.01, (.05, 0., 0))
        now = [0.]
        t.finish(d, b, False, clock=lambda: now[0], sleep=lambda dt: now.__setitem__(0, now[0]+dt))
        self.assertTrue(b.closed); self.assertGreaterEqual(len(b.calls), 300)
        self.assertTrue(all(p == [.0005, 0, 0] for p in b.calls))

    def test_key_to_native_direction_regression(self):
        # Positive local x follows vendor 202, without a heading-dependent sign.
        for key, expected in [('w', [.0005, 0., 0.]), ('s', [-.0005, 0., 0.]),
                              ('a', [0., .0005, 0.]), ('d', [0., -.0005, 0.]),
                              ('q', [0., 0., .001]), ('e', [0., 0., -.001])]:
            b = Backend(); d = t.Driver(b, 0); k = t.Keys(); k.feed(key, 0)
            d.tick(.01, k.sample(.01))
            self.assertEqual(b.p, expected)

    def test_forward_after_turn_never_changes_lateral_target(self):
        b = Backend(); b.p = [4., -7., math.pi/2]
        d = t.Driver(b, 0); k = t.Keys()
        k.feed('q', 0)
        d.tick(.01, k.sample(.01))
        yaw_after_turn = b.p[2]
        k.feed('w', .02)
        for i in range(2, 15):
            d.tick(i/100, k.sample(i/100))
            self.assertEqual(b.p[1], -7.)
            self.assertEqual(b.p[2], yaw_after_turn)
        self.assertGreater(b.p[0], 4.)
        k.feed('s', .15); before = b.p[0]
        d.tick(.15, k.sample(.15))
        self.assertLess(b.p[0], before)
        self.assertEqual(b.p[1], -7.)

    def test_finish_releases_on_loss_and_failure(self):
        for cause in ['rights', 'send']:
            b = Backend(); d = t.Driver(b, 0); d.tick(.01, (.05, 0., 0))
            if cause == 'rights': b.rights = False
            else: b.broken = True
            with self.assertRaises(RuntimeError): t.finish(d, b, False)
            self.assertTrue(b.closed); self.assertEqual(len(b.calls), 1)

    def test_conflict_scan(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            commands = [b'python3\0/home/astribot/tg-chassis.py\0info\0',
                        b'ros2\0run\0towergo_real\0chassis\0',
                        b'python3\0/a/install/towergo_real/lib/towergo_real/chassis\0',
                        b'python3\0unrelated.py\0',
                        b'python3\0/sdk/examples/202-chassis_joy_control_local.py\0',
                        b'python3\0/sdk/examples/203-chassis_joy_control_global.py\0',
                        b'python3\0/scripts/tg_teleop.py\0',
                        b'python3\0/scripts/tg_teleop.py\0--dry-run\0',
                        b'python3\0-m\0towergo_real.chassis\0']
            for i, cmd in enumerate(commands, 100000):
                (root/str(i)).mkdir(); (root/str(i)/'cmdline').write_bytes(cmd)
            self.assertEqual(set(t.conflicts(root)),
                             {'100000', '100001', '100002', '100004', '100005', '100006', '100008'})
            self.assertNotIn('100006', t.conflicts(root, include_teleop=False))

    def test_sdk_send_only_chassis(self):
        class Bot:
            chassis_name = 'base'
            def set_joints_position(self, *args, **kw): self.args, self.kw = args, kw
        b = t.SDKBackend.__new__(t.SDKBackend); b.bot = Bot(); b.send([1, 2, 3])
        self.assertEqual(b.bot.args, (['base'], [[1, 2, 3]]))
        self.assertEqual(b.bot.kw, dict(control_way='filter', use_wbc=False, add_default_torso=False))


class Terminal(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.env = dict(os.environ, XDG_RUNTIME_DIR=self.tmp.name)
        self.children = []

    def tearDown(self):
        for p, master, slave, saved in self.children:
            if p.poll() is None: p.kill(); p.wait(timeout=3)
            for fd in (master, slave):
                if fd >= 0: os.close(fd)
        self.tmp.cleanup()

    def start(self):
        master, slave = pty.openpty(); saved = termios.tcgetattr(slave)
        p = subprocess.Popen([str(LAUNCH), '--dry-run'], stdin=slave, stdout=slave,
                             stderr=slave, env=self.env, start_new_session=True)
        child = (p, master, slave, saved); self.children.append(child)
        return child

    def read(self, child, duration=.25):
        end = time.monotonic()+duration; data = b''
        while time.monotonic() < end:
            if select.select([child[1]], [], [], .02)[0]:
                try: data += os.read(child[1], 65536)
                except OSError: break
        return data.decode(errors='replace')

    def ready(self, child):
        text = self.read(child, .4)
        self.assertIn('READY', text)

    def test_keys_timeout_exit_restore(self):
        c = self.start(); self.ready(c)
        for key, label in [(b'w', '前进'), (b's', '后退'), (b'a', '左移'), (b'd', '右移'), (b'q', '左转'), (b'e', '右转')]:
            output = ''
            for _ in range(4):
                os.write(c[1], key); output += self.read(c, .07)
            self.assertIn(label, output)
            self.assertIsNone(c[0].poll())
        self.assertIn('输入超时', self.read(c, .45))
        os.write(c[1], b'wa'); self.assertIn('左移', self.read(c))
        os.write(c[1], b'+'); self.assertIn('v=0.060', self.read(c))
        os.write(c[1], b'w '); self.assertIn('保持目标', self.read(c))
        os.write(c[1], b'x'); self.assertEqual(c[0].wait(timeout=3), 0)
        self.assertEqual(termios.tcgetattr(c[2]), c[3])

    def test_signals_restore(self):
        for sig in [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]:
            c = self.start(); self.ready(c); os.write(c[1], b'w'); time.sleep(.03)
            c[0].send_signal(sig)
            self.assertEqual(c[0].wait(timeout=3), 128 + sig)
            self.assertEqual(termios.tcgetattr(c[2]), c[3])

    def test_eof_key(self):
        c = self.start(); self.ready(c); os.write(c[1], b'\x04')
        self.assertEqual(c[0].wait(timeout=3), 0)
        self.assertEqual(termios.tcgetattr(c[2]), c[3])

    def test_ctrl_s_does_not_freeze_terminal(self):
        c = self.start(); self.ready(c)
        self.assertFalse(termios.tcgetattr(c[2])[0] & (termios.IXON | termios.IXOFF))
        os.write(c[1], b'w'); time.sleep(.03)
        os.write(c[1], b'\x13')
        self.assertIn('保持目标', self.read(c, .35))
        os.write(c[1], b'x')
        self.assertEqual(c[0].wait(timeout=3), 0)
        self.assertEqual(termios.tcgetattr(c[2]), c[3])

    def test_duplicate_instance(self):
        c = self.start(); self.ready(c)
        second = self.start()
        self.assertEqual(second[0].wait(timeout=3), 2)
        self.assertIn('持有锁', self.read(second))
        self.assertIsNone(c[0].poll())

    def test_stall_aborts_and_restores(self):
        c = self.start(); self.ready(c)
        c[0].send_signal(signal.SIGSTOP)
        time.sleep(.2)
        os.write(c[1], b'wwww')
        c[0].send_signal(signal.SIGCONT)
        self.assertEqual(c[0].wait(timeout=3), 2)
        self.assertIn('不补发动作', self.read(c))
        self.assertEqual(termios.tcgetattr(c[2]), c[3])

    def test_terminal_disconnect_exits_and_unlocks(self):
        c = self.start(); self.ready(c)
        os.write(c[1], b'w'); time.sleep(.03)
        os.close(c[1])
        self.children[-1] = (c[0], -1, c[2], c[3])
        c[0].wait(timeout=3)
        # Closed terminal may yield EOF or EIO; either must terminate and unlock.
        second = self.start(); self.ready(second)
        os.write(second[1], b'x')
        self.assertEqual(second[0].wait(timeout=3), 0)

    def test_non_tty_and_invalid_arguments(self):
        for args in [[], ['--linear-speed', 'nan'], ['--angular-speed', '.3']]:
            result = subprocess.run([str(LAUNCH), '--dry-run', *args], input='',
                                    capture_output=True, text=True, env=self.env)
            self.assertEqual(result.returncode, 2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
