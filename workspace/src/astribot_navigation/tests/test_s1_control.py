"""Hardware-free tests for the S1 CLI's action boundaries and failure paths."""
import builtins
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 's1_control.py'
if not SCRIPT.exists():
    SCRIPT = Path(__file__).with_name('s1_control.py')
spec = importlib.util.spec_from_file_location('s1_control', SCRIPT)
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)


class FakeObserver:
    def __init__(self, state=2, owner=False):
        self.current = state
        self.owner = owner
        self.events = []

    def state(self, node=c.CONTROLLER):
        self.events.append(('state', node))
        value = self.current if node == c.CONTROLLER else 3
        return value, {1: 'unconfigured', 2: 'inactive', 3: 'active'}.get(value, 'unknown')

    def trigger(self, action):
        self.events.append(('trigger', action))
        self.current = 3 if action == 'activate' else 2

    def wait_feedback(self):
        self.events.append(('feedback',))

    def has_owner(self):
        return self.owner

    def verify_home(self):
        self.events.append(('verify',))

    def spin(self, seconds):
        pass

    def positions(self):
        return {}

    def diagnostics(self):
        return {'joystick_device_exists': False}


class ControlTests(unittest.TestCase):
    def test_default_status_does_not_send_action_or_construct_sdk(self):
        observer = FakeObserver()
        with patch.object(c, 'perform_home') as home, patch.object(c, 'log'):
            self.assertEqual(c.run('status', observer), 0)
        home.assert_not_called()
        self.assertFalse(any(e[0] == 'trigger' for e in observer.events))

    def test_unknown_state_stops_before_activation_and_sdk(self):
        observer, factory = FakeObserver(0), Mock()
        with self.assertRaises(c.ControlError):
            c.perform_home(observer, factory)
        factory.assert_not_called()
        self.assertFalse(any(e[0] == 'trigger' for e in observer.events))

    def test_state_timeout_stops_before_activation(self):
        observer, factory = FakeObserver(), Mock()
        observer.state = Mock(side_effect=c.ControlError('timeout'))
        with self.assertRaises(c.ControlError):
            c.perform_home(observer, factory)
        self.assertEqual(observer.events, [])
        factory.assert_not_called()

    def test_unconfigured_requires_explicit_configure(self):
        observer, factory = FakeObserver(1), Mock()
        with self.assertRaisesRegex(c.ControlError, 'configure'):
            c.perform_home(observer, factory)
        factory.assert_not_called()
        self.assertFalse(any(e[0] == 'trigger' for e in observer.events))

    def test_failed_activation_never_reaches_sdk(self):
        observer, factory = FakeObserver(), Mock()
        observer.trigger = Mock(side_effect=c.ControlError('failed'))
        with self.assertRaises(c.ControlError):
            c.perform_home(observer, factory)
        factory.assert_not_called()

    def test_missing_feedback_never_reaches_sdk(self):
        observer, factory = FakeObserver(), Mock()
        observer.wait_feedback = Mock(side_effect=c.ControlError('stale'))
        with self.assertRaises(c.ControlError):
            c.perform_home(observer, factory)
        factory.assert_not_called()

    def test_existing_owner_never_reaches_sdk(self):
        observer, factory = FakeObserver(3, owner=True), Mock()
        with self.assertRaisesRegex(c.ControlError, '控制者'):
            c.perform_home(observer, factory)
        factory.assert_not_called()

    def test_success_order_and_no_force_takeover(self):
        observer = FakeObserver()
        robot = Mock()
        robot.get_control_rights_status.return_value = True
        robot.move_to_home.return_value = 'move success'
        old_input = builtins.input
        def factory(**kwargs):
            self.assertFalse(kwargs['high_control_rights'])
            self.assertEqual(builtins.input('take over?'), '')
            self.assertEqual(observer.events[-1], ('feedback',))
            return robot
        c.perform_home(observer, factory)
        self.assertIs(builtins.input, old_input)
        robot.move_to_home.assert_called_once_with()
        robot.stop_robot.assert_not_called()
        robot.astribot_interface.shutdown.assert_called_once()
        self.assertEqual(observer.events[-1], ('verify',))
        self.assertEqual([e for e in observer.events if e[0] == 'trigger'], [('trigger', 'activate')])

    def test_control_rights_denied_does_not_move(self):
        observer, robot = FakeObserver(3), Mock()
        robot.get_control_rights_status.return_value = False
        with self.assertRaises(c.ControlError):
            c.perform_home(observer, lambda **kw: robot)
        robot.move_to_home.assert_not_called()

    def test_sdk_failure_requests_stop_and_does_not_report_success(self):
        observer, robot = FakeObserver(3), Mock()
        robot.get_control_rights_status.return_value = True
        robot.move_to_home.return_value = 'move failed'
        with self.assertRaises(c.ControlError):
            c.perform_home(observer, lambda **kw: robot)
        robot.stop_robot.assert_called_once()
        self.assertNotIn(('verify',), observer.events)

    def test_failed_arrival_verification_requests_stop(self):
        observer, robot = FakeObserver(3), Mock()
        observer.verify_home = Mock(side_effect=c.ControlError('not at home'))
        robot.get_control_rights_status.return_value = True
        robot.move_to_home.return_value = None
        with self.assertRaises(c.ControlError):
            c.perform_home(observer, lambda **kw: robot)
        robot.stop_robot.assert_called_once()

    def test_nan_short_and_missing_feedback_rejected(self):
        self.assertFalse(c.finite_vector([math.nan], 1))
        self.assertFalse(c.finite_vector([0.0], 7))
        self.assertTrue(c.home_errors({}))
        self.assertFalse(c.home_errors(c.HOME))
        wrong = dict(c.HOME, astribot_head=[0.0, 0.2])
        self.assertIn('astribot_head', c.home_errors(wrong))

    def test_request_timeout_is_unknown_and_not_retried(self):
        observer = object.__new__(c.Observer)
        observer.timeout = 1.0
        observer.node = Mock()
        observer.ros = Mock()
        observer.executor = Mock()
        client = observer.node.create_client.return_value
        client.wait_for_service.return_value = True
        future = client.call_async.return_value
        future.done.return_value = False
        with patch.object(c.time, 'monotonic', side_effect=[0.0, 2.0]):
            with self.assertRaisesRegex(c.ControlError, '执行结果未知'):
                observer.request('/supervisor/activate_system', SimpleNamespace(Request=Mock()), operation=True)
        client.call_async.assert_called_once()
        future.cancel.assert_called_once()
        observer.node.destroy_client.assert_called_once_with(client)

    def test_query_timeout_does_not_imply_an_action_was_sent(self):
        observer = object.__new__(c.Observer)
        observer.timeout = 1.0
        observer.node = Mock()
        observer.executor = Mock()
        client = observer.node.create_client.return_value
        client.wait_for_service.return_value = True
        client.call_async.return_value.done.return_value = False
        with patch.object(c.time, 'monotonic', side_effect=[0.0, 2.0]):
            with self.assertRaisesRegex(c.ControlError, '只读查询'):
                observer.request('/node/get_state', SimpleNamespace(Request=Mock()))


if __name__ == '__main__':
    unittest.main()
