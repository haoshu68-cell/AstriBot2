"""Faults during an action handshake must retain cancellation ownership."""
from types import SimpleNamespace as N
from unittest.mock import Mock
import unittest

from astribot_s1_transport.core import TaskFailure
from astribot_s1_transport.ros_backend import RosBackend


class CancellationTests(unittest.TestCase):
    def backend(self):
        sent, result = object(), object()
        handle = N(accepted=True, get_result_async=lambda: result)
        client = N(send_goal_async=Mock(return_value=sent))
        node = N(nav=object(), nav_through=object(), fixed_v2=True, hold_id='',
                 pending_goal=None, active=None, check=Mock(),
                 ledger=N(stage='TRANSPORT_POSTURE', emit=Mock()))
        node.future = Mock(side_effect=lambda value, *args, **kwargs:
                           handle if value is sent else N(status=4, result='OK'))
        node.cancel_active = Mock()
        return node, client, sent, handle, result

    def test_guard_fault_during_acceptance_retains_pending_handle(self):
        node, client, sent, *_ = self.backend()
        def fail(value, timeout, checked=True):
            self.assertTrue(checked)
            self.assertIs(node.pending_goal, sent)
            raise TaskFailure('MANIPULATION_TRACKING_ERROR')
        node.future = fail
        with self.assertRaisesRegex(TaskFailure, 'MANIPULATION_TRACKING_ERROR'):
            RosBackend.action(node, client, object(), 30.)
        node.cancel_active.assert_called_once()
        self.assertIs(node.pending_goal, sent)

    def test_cancel_timeout_preserves_original_fault_and_owner(self):
        node, client, _, handle, result = self.backend()
        node.check.side_effect = TaskFailure('GUARD_STALE')
        node.cancel_active.side_effect = TaskFailure('WAIT_TIMEOUT')
        with self.assertRaisesRegex(TaskFailure, 'GUARD_STALE'):
            RosBackend.action(node, client, object(), 30.)
        self.assertEqual(node.active, (handle, result))
        self.assertEqual(node.ledger.emit.call_args.kwargs['original_fault'], 'GUARD_STALE')

    def test_already_finished_result_still_checks_guard(self):
        node, client, *_ = self.backend()
        node.check.side_effect = [None, TaskFailure('BASE_MOVED')]
        with self.assertRaisesRegex(TaskFailure, 'BASE_MOVED'):
            RosBackend.action(node, client, object(), 30.)
        node.cancel_active.assert_called_once()

    def test_success_releases_owner(self):
        node, client, *_ = self.backend()
        self.assertEqual(RosBackend.action(node, client, object(), 30.), 'OK')
        self.assertIsNone(node.active)
        self.assertIsNone(node.pending_goal)
        node.cancel_active.assert_not_called()

    def test_cancel_response_is_not_terminal_confirmation(self):
        node, _, _, handle, result = self.backend()
        handle.cancel_goal_async = lambda: object()
        node.active = (handle, result)
        node.future = Mock(side_effect=[N(return_code=0), TaskFailure('WAIT_TIMEOUT')])
        with self.assertRaisesRegex(TaskFailure, 'WAIT_TIMEOUT'):
            RosBackend.cancel_active(node)
        self.assertEqual(node.active, (handle, result))

    def test_unknown_result_status_keeps_executor_owner(self):
        node, _, _, handle, result = self.backend()
        handle.cancel_goal_async = lambda: object()
        node.active = (handle, result)
        node.future = Mock(side_effect=[N(return_code=0), N(status=0)])
        with self.assertRaisesRegex(TaskFailure, 'EXECUTOR_TERMINAL_UNCONFIRMED'):
            RosBackend.cancel_active(node)
        self.assertEqual(node.active, (handle, result))


if __name__ == '__main__':
    unittest.main()
