"""Offline stop-evidence rejection cases for the live fixed navigation probe."""
import math
import unittest
from verify_fixed_navigation import measured_stop, navigation_entry_observed, transform_goal


def stationary():
    return [dict(t=10+i*.1,wall=20+i*.1,x=.1,y=.2,yaw=.3,
                 cmd_vx=0.,cmd_vy=0.,cmd_wz=0.,cmd_wall=20+i*.1,
                 frame='map',pose_source='/slam/pose') for i in range(8)]


class StopEvidence(unittest.TestCase):
    def test_goal_transform_rotates_position_and_heading(self):
        actual=transform_goal([1.,0.,.2],[2.,3.,math.pi/2])
        for a,b in zip(actual,[2.,4.,.2+math.pi/2]):self.assertAlmostEqual(a,b)

    def test_unobserved_status_requires_verified_cold_start(self):
        self.assertFalse(navigation_entry_observed({},False))
        self.assertFalse(navigation_entry_observed({'navigate_to_pose':set()},False))
        self.assertTrue(navigation_entry_observed({},True))
        self.assertTrue(navigation_entry_observed({'navigate_to_pose':set(),'navigate_through_poses':set()},False))

    def test_stationary_slam_and_ignored_odometry_speed(self):
        rows=stationary()
        self.assertTrue(measured_stop(rows,19.9,20.71)['passed'])
        for row in rows:row.update(vx=100.,vy=math.nan,wz=100.)
        self.assertTrue(measured_stop(rows,19.9,20.71)['passed'])

    def test_slam_cadence_does_not_require_odometry_rate(self):
        rows=stationary()[::2]
        self.assertTrue(measured_stop(rows,19.9,20.71)['passed'])
        # Observation age does not revoke an already received pose window.
        self.assertTrue(measured_stop(rows,19.9,22.)['passed'])
        self.assertFalse(measured_stop(rows,20.5,22.)['passed'])

    def test_pose_motion_cannot_pass_with_zero_commands(self):
        for key,delta in (('x',.006),('yaw',.011)):
            rows=stationary();rows[-1][key]+=delta
            self.assertFalse(measured_stop(rows,19.9,20.71)['passed'])

    def test_frozen_source_single_sample_and_duplicate_frames_rejected(self):
        rows=stationary()
        for row in rows:row['t']=10.
        self.assertFalse(measured_stop(rows,19.9,20.71)['passed'])
        self.assertFalse(measured_stop(stationary()[:1],19.9,22.)['passed'])
        self.assertFalse(measured_stop([],19.9,22.)['passed'])

    def test_wrong_source_frame_invalid_pose_and_nonzero_command_rejected(self):
        for change in ({'frame':'odom'},{'pose_source':'/odom'},{'yaw':math.nan},
                       {'cmd_vx':.04},{'cmd_wz':.04},{'cmd_wall':20.8}):
            rows=stationary();rows[-1].update(change)
            self.assertFalse(measured_stop(rows,19.9,20.71)['passed'],change)

    def test_repeated_bidirectional_jitter_cancels_in_pose_window(self):
        rows=stationary()
        for i,row in enumerate(rows):
            row['x']=.002 if i%2 else -.002
            row['yaw']=.003 if i%2 else -.003
        self.assertGreater(sum(abs(b['x']-a['x']) for a,b in zip(rows,rows[1:])),.005)
        self.assertGreater(sum(abs(b['yaw']-a['yaw']) for a,b in zip(rows,rows[1:])),.01)
        result=measured_stop(rows,19.9,20.71)
        self.assertTrue(result['passed'])
        self.assertLessEqual(result['drift_m'],.004000001)
        self.assertLessEqual(result['rotation_rad'],.006000001)

    def test_large_excursion_is_not_erased_by_returning_to_origin(self):
        rows=stationary();rows[4]['x']+=.006
        self.assertFalse(measured_stop(rows,19.9,20.71)['passed'])

    def test_wraparound_is_not_rotation(self):
        rows=stationary()
        for i,row in enumerate(rows):row['yaw']=math.pi-.001 if i%2 else -math.pi+.001
        self.assertTrue(measured_stop(rows,19.9,20.71)['passed'])

    def test_event_stop_accepts_real_zero_without_periodic_command(self):
        rows=stationary()
        for row in rows:row.update(cmd_wall=19.,command_observed=True)
        self.assertFalse(measured_stop(rows,19.9,20.71)['passed'])
        result=measured_stop(rows,19.9,20.71,event_driven_command=True)
        self.assertTrue(result['passed']);self.assertTrue(result['command_observed'])

    def test_unobserved_command_only_allowed_for_explicit_idle_context(self):
        rows=stationary()
        for row in rows:row.update(command_observed=False,cmd_vx=math.nan,cmd_vy=math.nan,cmd_wz=math.nan,cmd_wall=-math.inf)
        self.assertFalse(measured_stop(rows,19.9,20.71,event_driven_command=True)['passed'])
        result=measured_stop(rows,19.9,20.71,event_driven_command=True,idle_command_silence=True)
        self.assertTrue(result['passed']);self.assertFalse(result['command_observed'])
        for row in rows:del row['command_observed']
        self.assertFalse(measured_stop(rows,19.9,20.71,event_driven_command=True,idle_command_silence=True)['passed'])

    def test_slam_sample_source_boundary(self):
        from types import SimpleNamespace as S
        from verify_fixed_navigation import slam_pose_sample
        msg=S(header=S(frame_id='map',stamp=S(sec=10,nanosec=0)),
              pose=S(pose=S(position=S(x=0.,y=0.,z=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))))
        row=slam_pose_sample(msg,20.,None)
        self.assertEqual(row['pose_source'],'/slam/pose')
        self.assertNotIn('vx',row)
        msg.header.frame_id='odom'
        with self.assertRaisesRegex(RuntimeError,'INVALID_SLAM_POSE'):slam_pose_sample(msg,20.,None)


class HoldRenewalOwnership(unittest.TestCase):
    def test_foreign_feedback_and_global_status_do_not_authorize_renewal(self):
        from verify_fixed_navigation import OwnedHoldLease
        binding=OwnedHoldLease('own')
        binding.observe('foreign','lease','epoch','hold')
        binding.accepted=True
        self.assertFalse(binding.matches(dict(lease_id='lease',epoch='epoch',hold_id='hold',phase='2')))

    def test_feedback_before_admission_cannot_renew(self):
        from verify_fixed_navigation import OwnedHoldLease
        binding=OwnedHoldLease('own');binding.observe('own','lease','epoch','hold')
        status=dict(lease_id='lease',epoch='epoch',hold_id='hold',phase='2')
        self.assertFalse(binding.matches(status));binding.accepted=True
        self.assertTrue(binding.matches(status))
        for key in ('lease_id','epoch','hold_id'):
            self.assertFalse(binding.matches(dict(status,**{key:'foreign'})))

    def test_changed_own_identity_is_rejected(self):
        from verify_fixed_navigation import OwnedHoldLease
        binding=OwnedHoldLease('own');binding.observe('own','lease','epoch','hold')
        with self.assertRaises(RuntimeError):binding.observe('own','different','epoch','hold')

    def test_incomplete_and_terminal_feedback_does_not_grant_renewal(self):
        from verify_fixed_navigation import OwnedHoldLease
        binding=OwnedHoldLease('own');binding.accepted=True
        binding.observe('own','lease','','hold')
        self.assertFalse(binding.matches(dict(lease_id='lease',epoch='',hold_id='hold',phase='2')))
        binding.observe('own','lease','epoch','hold')
        self.assertFalse(binding.matches(dict(lease_id='lease',epoch='epoch',hold_id='hold',phase='0')))

    def test_changed_feedback_latches_without_interrupting_cleanup_callbacks(self):
        import ast
        from pathlib import Path
        from types import SimpleNamespace as S
        from verify_fixed_navigation import OwnedHoldLease
        tree=ast.parse(Path(__file__).with_name('verify_fixed_navigation.py').read_text())
        main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        callback=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='hold_feedback')
        class Scope(ast.NodeTransformer):
            def visit_Nonlocal(self,node):return ast.copy_location(ast.Global(names=node.names),node)
        callback=Scope().visit(callback)
        binding=OwnedHoldLease('01');binding.observe('01','mine','epoch','hold');binding.accepted=True
        namespace=dict(hold_binding=binding,renew_enabled=True,latest={},cleanup_health_errors=[])
        exec(compile(ast.fix_missing_locations(ast.Module(body=[callback],type_ignores=[])), '<actual callback>', 'exec'),namespace)
        msg=S(goal_id=S(uuid=[1]),feedback=S(lease_id='changed',resource_epoch='epoch',hold_id='hold'))
        for _ in range(2):namespace['hold_feedback'](msg)
        self.assertFalse(namespace['renew_enabled'])
        self.assertEqual(namespace['latest']['hold_ownership_error'],'OWN_HOLD_FEEDBACK_IDENTITY_CHANGED')
        self.assertEqual(namespace['cleanup_health_errors'],['OWN_HOLD_FEEDBACK_IDENTITY_CHANGED'])

if __name__=='__main__': unittest.main()
