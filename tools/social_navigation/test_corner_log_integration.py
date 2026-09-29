"""Consume actual offline C++ probe logs; receive context is synthetic, not ROS evidence."""
import argparse
from pathlib import Path
import unittest

from corner_phase_evidence import CornerPhaseGate, parse_state


class ControllerLogIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.events = []
        for line in cls.log_path.read_text().splitlines():
            if line.startswith('CORNER_STATE '):
                event = parse_state(line, session_id='offline-object-probe', clock_epoch=1)
                event.update(sequence=event['event_id'], received_ros_s=event['ros_s']+.001,
                             received_wall_s=event['ros_s']+1000.+.001)
                cls.events.append(event)
        if len(cls.events) < 8:
            raise ValueError('incomplete controller lifecycle probe log')

    def event(self, **matches):
        return next(event for event in self.events if all(event[k] == v for k, v in matches.items()))

    def gate(self, execution=1, revision=1, armed=200.):
        return CornerPhaseGate('offline-object-probe', 1, revision, 0, 'ALIGN_CORNER',
                               armed, armed+1000., execution=execution)

    def decision(self, gate, event, phase='ALIGN_CORNER'):
        now = event['ros_s']+.01
        return gate.check(now, now+1000., now, phase)

    def test_actual_stage_transition_matches(self):
        event = self.event(execution=1, revision=1, state='TURNING')
        gate = self.gate(); gate.observe(event)
        self.assertEqual(self.decision(gate, event)['status'], 'READY_FOR_INJECTION')

    def test_actual_equivalent_refresh_retires_old_revision(self):
        event = self.event(reason='plan_refresh')
        old = self.gate(); old.observe(event)
        self.assertEqual(self.decision(old, event)['reason'], 'PLAN_CHANGED')
        current = self.gate(revision=2); current.observe(event)
        self.assertEqual(self.decision(current, event)['status'], 'READY_FOR_INJECTION')

    def test_actual_deactivate_invalidates_without_pose(self):
        active = self.event(reason='plan_refresh')
        reset = self.event(execution=1, revision=2, pose_valid=False, ros_s=200.2)
        gate = self.gate(revision=2); gate.observe(active)
        verdict = self.decision(gate, active)
        self.assertEqual(verdict['status'], 'READY_FOR_INJECTION')
        gate.observe(reset)
        self.assertNotEqual(self.decision(gate, reset)['status'], 'READY_FOR_INJECTION')
        with self.assertRaises(ValueError):
            gate.dispatch(verdict, 200.21, 1200.21)

    def test_actual_new_execution_retires_old_gate(self):
        event = self.event(execution=2, revision=3, pose_valid=False)
        gate = self.gate(revision=2); gate.observe(event)
        self.assertEqual(self.decision(gate, event)['reason'], 'EXECUTION_CHANGED')

    def test_actual_cleanup_retires_same_execution(self):
        active = self.event(execution=2, revision=4, state='TURNING')
        reset = self.event(execution=2, revision=4, pose_valid=False)
        gate = self.gate(execution=2, revision=4, armed=200.3); gate.observe(active)
        self.assertEqual(self.decision(gate, active)['status'], 'READY_FOR_INJECTION')
        gate.observe(reset)
        self.assertNotEqual(self.decision(gate, reset)['status'], 'READY_FOR_INJECTION')

    def test_hold_and_missing_fresh_event_cannot_be_inferred(self):
        event = self.event(execution=1, revision=1, state='TURNING')
        gate = self.gate(); gate.observe(event)
        self.assertNotEqual(self.decision(gate, event, 'POLICY_HOLD')['status'], 'READY_FOR_INJECTION')
        self.assertNotEqual(gate.check(200.5, 1200.5, 200.5, 'ALIGN_CORNER')['status'], 'READY_FOR_INJECTION')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', required=True, type=Path)
    args, rest = parser.parse_known_args()
    ControllerLogIntegration.log_path = args.log
    unittest.main(argv=[__file__, *rest])
