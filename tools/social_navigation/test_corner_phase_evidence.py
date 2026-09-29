import unittest
from corner_phase_evidence import CornerPhaseGate, parse_state

class PhaseEvidenceChecks(unittest.TestCase):
    def gate(self,requested='CORNER_RECOVERY'):
        return CornerPhaseGate('session-A',2,7,0,requested,10.,100.,timeout_s=2.)
    def event(self,state='RECOVERING',**changes):
        e=dict(session_id='session-A',clock_epoch=2,revision=7,cursor=0,state=state,
               pose_s=10.1,received_ros_s=10.1,received_wall_s=100.1,sequence=1)
        e.update(changes);return e
    def decision(self,g,**changes):
        x=dict(now_ros_s=10.2,now_wall_s=100.2,pose_s=10.2,phase='ALIGN_CORNER')
        x.update(changes);return g.check(**x)
    def test_follow_is_not_recovery(self):
        g=self.gate();self.assertEqual(self.decision(g,phase='FOLLOW')['reason'],'NO_STATE_EVENT')
    def test_real_recovery_state(self):
        g=self.gate();g.observe(self.event());self.assertEqual(self.decision(g)['status'],'READY_FOR_INJECTION')
    def test_exact_states(self):
        for requested,state,phase in [('CORNER_APPROACH','APPROACH','CORNER_APPROACH'),('CORNER_SETTLING','SETTLING_BEFORE_TURN','CORNER_APPROACH'),('ALIGN_CORNER','TURNING','ALIGN_CORNER'),('CORNER_SETTLING_AFTER','SETTLING_AFTER_TURN','ALIGN_CORNER')]:
            with self.subTest(requested=requested):
                g=self.gate(requested);g.observe(self.event(state));self.assertEqual(self.decision(g,phase=phase)['status'],'READY_FOR_INJECTION')
    def test_old_context_and_cursor(self):
        for changes in [dict(session_id='B'),dict(clock_epoch=1),dict(revision=6),dict(cursor=1),dict(pose_s=9.9)]:
            with self.subTest(changes=changes):
                g=self.gate();g.observe(self.event(**changes));self.assertNotEqual(self.decision(g)['status'],'READY_FOR_INJECTION')
    def test_new_revision_invalidates_original_request(self):
        g=self.gate();g.observe(self.event());g.observe(self.event(revision=8,sequence=2));self.assertEqual(self.decision(g)['reason'],'PLAN_CHANGED')
    def test_wall_and_source_freshness(self):
        for changes in [dict(now_ros_s=10.6,pose_s=10.6),dict(now_wall_s=100.6),dict(pose_s=9.9),dict(pose_s=10.4),dict(phase='POLICY_HOLD')]:
            with self.subTest(changes=changes):
                g=self.gate();g.observe(self.event());self.assertNotEqual(self.decision(g,**changes)['status'],'READY_FOR_INJECTION')
    def test_clock_rollback_latches_invalid(self):
        g=self.gate();g.observe(self.event());self.decision(g,now_ros_s=9.9,pose_s=9.9)
        self.assertEqual(self.decision(g)['reason'],'CLOCK_ROLLBACK')
    def test_duplicate_cannot_refresh_deadline(self):
        g=self.gate();g.observe(self.event());g.observe(self.event(received_wall_s=101.9,received_ros_s=10.2));self.assertEqual(self.decision(g,now_wall_s=102.01)['reason'],'INJECTION_DEADLINE')
    def test_out_of_order_cannot_replace_latest(self):
        g=self.gate();g.observe(self.event('TURNING',sequence=2));g.observe(self.event(sequence=1));self.assertEqual(self.decision(g)['reason'],'STATE_MISMATCH')
    def test_one_dispatch_and_ack_are_distinct(self):
        g=self.gate();g.observe(self.event());verdict=self.decision(g);g.dispatch(verdict,10.21,100.21)
        self.assertEqual(self.decision(g)['reason'],'ALREADY_DISPATCHED')
        self.assertEqual(g.confirm(False)['status'],'INJECTION_FAILED')
        with self.assertRaises(ValueError):g.dispatch(verdict,10.21,100.21)
    def test_ack_does_not_prove_physical_effect(self):
        g=self.gate();g.observe(self.event());g.dispatch(self.decision(g),10.21,100.21);result=g.confirm(True)
        self.assertEqual(result['status'],'INJECTION_ACKNOWLEDGED');self.assertFalse(result['effect_validated'])
    def test_delayed_dispatch_rejected(self):
        g=self.gate();g.observe(self.event());verdict=self.decision(g)
        with self.assertRaises(ValueError):g.dispatch(verdict,10.5,100.5)
    def test_state_change_invalidates_checked_verdict(self):
        g=self.gate();g.observe(self.event());verdict=self.decision(g)
        g.observe(self.event('TURNING',sequence=2,pose_s=10.21,received_ros_s=10.21,received_wall_s=100.21))
        with self.assertRaises(ValueError):g.dispatch(verdict,10.22,100.22)
    def test_duplicate_cannot_refresh_freshness(self):
        g=self.gate();g.observe(self.event());g.observe(self.event(received_wall_s=100.4,received_ros_s=10.4))
        self.assertNotEqual(self.decision(g,now_wall_s=100.45)['status'],'READY_FOR_INJECTION')
    def test_nonfinite_time_invalidates(self):
        g=self.gate();g.observe(self.event())
        self.assertEqual(self.decision(g,now_ros_s=float('nan'))['status'],'INVALID_EVIDENCE')
    def test_conflicting_acknowledgement_rejected(self):
        g=self.gate();g.observe(self.event());g.dispatch(self.decision(g),10.21,100.21);g.confirm(False)
        with self.assertRaises(ValueError):g.confirm(True)
    def test_invalid_target_and_timeout_rejected(self):
        with self.assertRaises(ValueError):self.gate('FOLLOW')
        with self.assertRaises(ValueError):CornerPhaseGate('a',0,1,0,'ALIGN_CORNER',10.,100.,timeout_s=0.)
    def test_missing_clock_epoch_rejected(self):
        with self.assertRaises(ValueError):CornerPhaseGate('a',None,1,0,'ALIGN_CORNER',10.,100.)
    def test_other_cursor_cannot_leave_old_ready_state(self):
        g=CornerPhaseGate('session-A',2,7,1,'CORNER_RECOVERY',10.,100.)
        g.observe(self.event(cursor=1));g.observe(self.event('TURNING',cursor=0,sequence=2))
        self.assertNotEqual(self.decision(g)['status'],'READY_FOR_INJECTION')
    def test_future_receive_ros_is_not_ready(self):
        g=self.gate();g.observe(self.event(received_ros_s=1000.))
        self.assertNotEqual(self.decision(g)['status'],'READY_FOR_INJECTION')
    def test_late_source_cannot_replace_newer_state(self):
        g=self.gate();g.observe(self.event('TURNING',pose_s=10.15,received_ros_s=10.15));g.observe(self.event(sequence=2))
        self.assertNotEqual(self.decision(g)['status'],'READY_FOR_INJECTION')
    def test_parser_and_malformed(self):
        e=parse_state('CORNER_STATE state=RECOVERING cursor=0 revision=7 pose_s=10.100000',session_id='session-A',clock_epoch=2,received_ros_s=10.1,received_wall_s=100.1,sequence=1)
        self.assertEqual(e['state'],'RECOVERING')
        for text in ['FOLLOW','CORNER_STATE state=BOGUS cursor=0 revision=7 pose_s=10.1','CORNER_STATE state=TURNING cursor=0 revision=7 pose_s=nan']:
            with self.assertRaises(ValueError):parse_state(text,session_id='a',clock_epoch=1,received_ros_s=10.,received_wall_s=100.,sequence=1)

if __name__=='__main__':unittest.main()
