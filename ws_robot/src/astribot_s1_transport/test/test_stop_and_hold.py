"""Exercise cleanup with real cancellation/envelope logic and offline ROS I/O."""
import copy
from concurrent.futures import Future
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as N
import unittest

from astribot_navigation_msgs.msg import RobotEnvelope
from astribot_navigation_msgs.srv import SetRobotEnvelope
from astribot_s1_transport.core import Ledger, TaskFailure, TransportTask
from astribot_s1_transport.ros_backend import RosBackend


class CleanupBackend:
    stop_and_hold = RosBackend.stop_and_hold
    cancel_active = RosBackend.cancel_active
    change_envelope = RosBackend.change_envelope
    wait = RosBackend.wait

    def __init__(self, ledger, cancel_error=None, hold_error=None):
        self.ledger = ledger
        self.ledger.object.state = 'ATTACHED'
        self.fixed_v2 = False
        self.attached = True
        self.scene_created = True
        self.c = {'mass_kg': .2}
        self.envelope = RobotEnvelope(epoch=5, transport_ready=True, lease_s=.3,
                                      payload_mass_kg=.2)
        self.envelope.stamp.sec = 10
        self.pending_goal = None
        self.hold_error = hold_error
        self.set_envelope = object()
        self.cancel_response = Future()
        self.cancel_response.set_result(N(return_code=0))
        self.result = Future()
        if cancel_error is None:
            self.result.set_result(N(status=5))
        else:
            self.result.set_exception(cancel_error)
        handle = N(cancel_goal_async=lambda: self.cancel_response)
        self.active = (handle, self.result)

    def future(self, future, timeout=20., checked=True):
        # Offline action responses are already resolved; errors model the ROS
        # wait boundary without starting a node or waiting on wall time.
        return future.result()

    def call(self, client, request, checked=True):
        if self.hold_error is not None:
            raise self.hold_error
        self.envelope = copy.deepcopy(request.envelope)
        self.envelope.epoch = 6
        return SetRobotEnvelope.Response(accepted=True, epoch=6)

    def get_clock(self):
        return N(now=lambda: N(nanoseconds=10_000_000_000))

    def stopped(self):
        return True


class StopAndHoldTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        self.ledger = Ledger(self.directory, 'box')

    def test_cancel_terminal_timeout_still_revokes_navigation(self):
        error = TaskFailure('EXECUTOR_TERMINAL_TIMEOUT')
        backend = CleanupBackend(self.ledger, cancel_error=error)
        active = backend.active
        with self.assertRaises(TaskFailure) as caught:
            backend.stop_and_hold()
        self.assertFalse(backend.envelope.transport_ready)
        self.assertIs(caught.exception, error)
        self.assertIs(backend.active, active)
        self.assertEqual(backend.envelope.payload_mass_kg, .2)
        self.assertEqual(self.ledger.object.state, 'ATTACHED')

    def test_pending_acceptance_timeout_still_revokes_navigation(self):
        error = TaskFailure('GOAL_ACCEPTANCE_TIMEOUT')
        backend = CleanupBackend(self.ledger)
        backend.active = None
        pending = Future()
        pending.set_exception(error)
        backend.pending_goal = pending
        with self.assertRaises(TaskFailure) as caught:
            backend.stop_and_hold()
        self.assertFalse(backend.envelope.transport_ready)
        self.assertIs(caught.exception, error)
        self.assertIs(backend.pending_goal, pending)

    def test_cancel_and_hold_failures_both_reach_caller(self):
        cancel_error = TaskFailure('EXECUTOR_TERMINAL_TIMEOUT')
        hold_error = TaskFailure('ENVELOPE_SERVICE_UNAVAILABLE')
        backend = CleanupBackend(self.ledger, cancel_error, hold_error)
        active = backend.active
        with self.assertRaises(TaskFailure) as caught:
            backend.stop_and_hold()
        self.assertIn('EXECUTOR_TERMINAL_TIMEOUT', str(caught.exception))
        self.assertIn('ENVELOPE_SERVICE_UNAVAILABLE', str(caught.exception))
        self.assertIs(caught.exception.__cause__, cancel_error)
        self.assertIs(backend.active, active)
        self.assertTrue(backend.envelope.transport_ready)

    def test_hold_failure_after_cancel_is_not_reported_as_success(self):
        error = TaskFailure('ENVELOPE_SERVICE_UNAVAILABLE')
        backend = CleanupBackend(self.ledger, hold_error=error)
        with self.assertRaises(TaskFailure) as caught:
            backend.stop_and_hold()
        self.assertIs(caught.exception, error)
        self.assertIsNone(backend.active)
        self.assertTrue(backend.envelope.transport_ready)

    def test_success_requires_terminal_and_confirmed_hold(self):
        backend = CleanupBackend(self.ledger)
        backend.stop_and_hold()
        self.assertIsNone(backend.active)
        self.assertFalse(backend.envelope.transport_ready)
        self.assertEqual(backend.envelope.epoch, 6)

    def test_task_ledger_keeps_original_fault_and_both_cleanup_failures(self):
        backend = CleanupBackend(self.ledger, TaskFailure('EXECUTOR_TERMINAL_TIMEOUT'),
                                 TaskFailure('ENVELOPE_SERVICE_UNAVAILABLE'))
        backend.check = lambda: None
        def admission_failure():
            raise TaskFailure('ORIGINAL_TASK_FAULT')
        backend.admit = admission_failure
        config = json.loads((Path(__file__).parents[1] / 'config/warehouse_transfer.json').read_text())
        self.assertFalse(TransportTask(backend, config, self.ledger).run())
        state = json.loads((self.directory / 'state.json').read_text())
        self.assertEqual(state['reason'], 'ORIGINAL_TASK_FAULT')
        self.assertIn('EXECUTOR_TERMINAL_TIMEOUT', state['stop_error'])
        self.assertIn('ENVELOPE_SERVICE_UNAVAILABLE', state['stop_error'])
        self.assertTrue(state['recovery_required'])
        self.assertEqual(state['object']['state'], 'ATTACHED')
        self.assertIsNotNone(backend.active)


if __name__ == '__main__':
    unittest.main()
