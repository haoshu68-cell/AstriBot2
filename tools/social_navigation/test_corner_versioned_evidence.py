import unittest

from corner_phase_evidence import CornerPhaseGate, parse_state


class VersionedEvidenceChecks(unittest.TestCase):
    def message(self, **changes):
        values = dict(state='TURNING', cursor=0, revision=7, pose_s=10.1,
                      schema=1, execution=4, event=1, ros_s=10.1,
                      pose_valid=1, active=1, reason='transition')
        values.update(changes)
        return 'CORNER_STATE ' + ' '.join(f'{key}={value}' for key, value in values.items())

    def event(self, **changes):
        return parse_state(self.message(**changes), session_id='session-A', clock_epoch=2,
                           received_ros_s=10.2, received_wall_s=100.2, sequence=changes.get('event', 1))

    def gate(self):
        return CornerPhaseGate('session-A', 2, 7, 0, 'ALIGN_CORNER', 10., 100., execution=4)

    def decision(self, gate, **changes):
        values = dict(now_ros_s=10.2, now_wall_s=100.2, pose_s=10.2, phase='ALIGN_CORNER')
        values.update(changes)
        return gate.check(**values)

    def test_parser_preserves_controller_identity(self):
        event = self.event()
        self.assertEqual(event.get('execution'), 4)
        self.assertEqual(event.get('event_id'), 1)
        self.assertEqual(event.get('ros_s'), 10.1)
        self.assertIs(event.get('pose_valid'), True)

    def test_fresh_post_arm_control_event_can_reference_previous_sensor_tick(self):
        gate = self.gate()
        gate.observe(self.event(pose_s=9.98, ros_s=10.02, reason='plan_refresh'))
        self.assertEqual(self.decision(gate)['status'], 'READY_FOR_INJECTION')

    def test_post_arm_control_event_does_not_rescue_stale_pose(self):
        gate = self.gate()
        gate.observe(self.event(pose_s=9.69, ros_s=10.02, reason='plan_refresh'))
        self.assertNotEqual(self.decision(gate)['status'], 'READY_FOR_INJECTION')

    def test_pre_arm_control_event_remains_rejected_with_recent_pose(self):
        gate = self.gate()
        gate.observe(self.event(pose_s=9.98, ros_s=9.99, reason='plan_refresh'))
        self.assertNotEqual(self.decision(gate)['status'], 'READY_FOR_INJECTION')

    def test_matching_execution_is_ready(self):
        gate = self.gate(); gate.observe(self.event())
        verdict = self.decision(gate)
        self.assertEqual(verdict['status'], 'READY_FOR_INJECTION')
        self.assertEqual(verdict['execution'], 4)

    def test_zero_pose_reset_revokes_ready_verdict(self):
        gate = self.gate(); gate.observe(self.event()); verdict = self.decision(gate)
        gate.observe(self.event(state='IDLE', pose_s=0, pose_valid=0, active=0,
                                event=2, ros_s=10.2, reason='reset'))
        self.assertNotEqual(self.decision(gate)['status'], 'READY_FOR_INJECTION')
        with self.assertRaises(ValueError):
            gate.dispatch(verdict, 10.21, 100.21)

    def test_unavailable_pose_cannot_make_active_state_ready(self):
        gate = self.gate(); gate.observe(self.event(pose_s=0, pose_valid=0))
        self.assertNotEqual(self.decision(gate)['status'], 'READY_FOR_INJECTION')

    def test_new_execution_invalidates_even_with_zero_pose(self):
        gate = self.gate(); gate.observe(self.event())
        gate.observe(self.event(state='IDLE', pose_s=0, pose_valid=0, active=0,
                                event=2, execution=5, reason='reset'))
        self.assertEqual(self.decision(gate)['reason'], 'EXECUTION_CHANGED')

    def test_old_execution_is_not_observed(self):
        gate = self.gate(); gate.observe(self.event(execution=3))
        self.assertEqual(self.decision(gate)['reason'], 'NO_STATE_EVENT')

    def test_duplicate_publisher_event_cannot_refresh_receive_time(self):
        gate = self.gate(); gate.observe(self.event())
        event = self.event(); event.update(sequence=2, received_ros_s=10.4, received_wall_s=100.4)
        gate.observe(event)
        self.assertNotEqual(self.decision(gate, now_ros_s=10.45, now_wall_s=100.45,
                                         pose_s=10.45)['status'], 'READY_FOR_INJECTION')

    def test_legacy_payload_cannot_satisfy_explicit_execution(self):
        gate = self.gate(); gate.observe(dict(state='TURNING', cursor=0, revision=7, pose_s=10.1,
            session_id='session-A', clock_epoch=2, received_ros_s=10.2, received_wall_s=100.2, sequence=1))
        self.assertEqual(self.decision(gate)['reason'], 'NO_STATE_EVENT')

    def test_context_cannot_promote_legacy_log_to_versioned_evidence(self):
        line = 'CORNER_STATE state=TURNING cursor=0 revision=7 pose_s=10.1'
        with self.assertRaises(ValueError):
            parse_state(line, schema=1, execution=4, event_id=1, ros_s=10.1,
                        pose_valid=True, active=True, reason='transition')

    def test_versioned_log_needs_declared_execution(self):
        gate = CornerPhaseGate('session-A', 2, 7, 0, 'ALIGN_CORNER', 10., 100.)
        gate.observe(self.event())
        self.assertNotEqual(self.decision(gate)['status'], 'READY_FOR_INJECTION')

    def test_context_failure_revokes_previously_checked_legacy_verdict(self):
        gate = CornerPhaseGate('session-A', 2, 7, 0, 'ALIGN_CORNER', 10., 100.)
        gate.observe(dict(state='TURNING', cursor=0, revision=7, pose_s=10.1,
            session_id='session-A', clock_epoch=2, received_ros_s=10.2, received_wall_s=100.2, sequence=1))
        verdict = self.decision(gate)
        self.assertEqual(verdict['status'], 'READY_FOR_INJECTION')
        gate.observe(self.event(event=2))
        with self.assertRaises(ValueError):
            gate.dispatch(verdict, 10.21, 100.21)

    def test_invalid_or_conflicting_metadata_is_rejected(self):
        for changes in [dict(schema=2), dict(event=0), dict(execution=-1), dict(ros_s='nan'),
                        dict(active=0), dict(pose_s=0), dict(pose_valid=0),
                        dict(reason='unknown'), dict(reason='reset')]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.event(**changes)
        with self.assertRaises(ValueError):
            parse_state(self.message(), execution=99)
        for message in [self.message().replace(' event=1', ''), self.message()+' execution=4']:
            with self.assertRaises(ValueError):
                parse_state(message)

    def test_control_time_and_pose_regression_do_not_refresh_state(self):
        for changes in [dict(ros_s=9., pose_s=8.9), dict(ros_s=11.)]:
            with self.subTest(changes=changes):
                gate = self.gate(); gate.observe(self.event(**changes))
                self.assertNotEqual(self.decision(gate)['status'], 'READY_FOR_INJECTION')
        gate = self.gate(); gate.observe(self.event(pose_s=10.15, ros_s=10.15))
        gate.observe(self.event(event=2, pose_s=10.1, ros_s=10.2))
        self.assertNotEqual(self.decision(gate)['status'], 'READY_FOR_INJECTION')

    def test_reanchor_does_not_require_active_corner_flag(self):
        gate = CornerPhaseGate('session-A', 2, 7, 0, 'REANCHOR_SETTLING', 10., 100., execution=4)
        gate.observe(self.event(state='REANCHOR_SETTLING', active=0))
        self.assertEqual(self.decision(gate, phase='REANCHOR_SETTLING')['status'], 'READY_FOR_INJECTION')

    def test_dispatch_rechecks_control_time_age(self):
        gate = self.gate()
        event = self.event(ros_s=10.1, pose_s=10.14)
        event.update(received_ros_s=10.15, received_wall_s=100.15)
        gate.observe(event); verdict = self.decision(gate)
        self.assertEqual(verdict['status'], 'READY_FOR_INJECTION')
        with self.assertRaises(ValueError):
            gate.dispatch(verdict, 10.42, 100.42)

    def test_dispatch_rechecks_receive_ros_age(self):
        gate = self.gate()
        event = self.event(ros_s=10.14, pose_s=10.14)
        event.update(received_ros_s=10.1, received_wall_s=100.1)
        gate.observe(event); verdict = self.decision(gate)
        self.assertEqual(verdict['status'], 'READY_FOR_INJECTION')
        with self.assertRaises(ValueError):
            gate.dispatch(verdict, 10.42, 100.39)


if __name__ == '__main__':
    unittest.main()
