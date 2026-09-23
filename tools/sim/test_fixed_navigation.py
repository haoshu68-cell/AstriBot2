"""Offline stop-evidence rejection cases for the live fixed navigation probe."""
import math
import unittest
from verify_fixed_navigation import measured_stop, navigation_entry_observed, transform_goal


def stationary():
    return [dict(t=10+i*.05, wall=20+i*.05, x=.1, y=.2, yaw=.3, vx=0., vy=0., wz=0., cmd_vx=0., cmd_vy=0., cmd_wz=0., cmd_wall=20+i*.05, frame='odom') for i in range(15)]


class StopEvidence(unittest.TestCase):
    def test_goal_transform_rotates_position_and_heading(self):
        actual=transform_goal([1.,0.,.2],[2.,3.,math.pi/2])
        for a,b in zip(actual,[2.,4.,.2+math.pi/2]):self.assertAlmostEqual(a,b)
        self.assertEqual(transform_goal([1.,2.,0.],[0.,0.,0.]),[1.,2.,0.])

    def test_unobserved_status_requires_verified_cold_start(self):
        self.assertFalse(navigation_entry_observed({}, False))
        self.assertFalse(navigation_entry_observed({'navigate_to_pose': set()}, False))
        self.assertTrue(navigation_entry_observed({}, True))
        self.assertTrue(navigation_entry_observed({'navigate_to_pose': set(), 'navigate_through_poses': set()}, False))

    def test_fresh_continuous_stop(self):
        self.assertTrue(measured_stop(stationary(), 9.9, 20.71, 10.71)['passed'])

    def test_zero_command_with_pose_drift_is_not_stop(self):
        rows=stationary()
        for i,row in enumerate(rows): row['x'] += i*.004
        self.assertFalse(measured_stop(rows, 9.9, 20.71, 10.71)['passed'])

    def test_frozen_source_and_duplicate_frames_rejected(self):
        rows=stationary()
        for row in rows: row['t']=10.
        self.assertFalse(measured_stop(rows, 9.9, 20.71, 10.71)['passed'])

    def test_stale_source_receipt_rejected(self):
        self.assertFalse(measured_stop(stationary(), 9.9, 22., 10.71)['passed'])

    def test_delayed_source_time_cannot_be_refreshed_by_receipt(self):
        self.assertFalse(measured_stop(stationary(), 9.9, 20.71, now_ros_s=12.)['passed'])

    def test_command_from_before_hold_rejected(self):
        rows=stationary()
        for row in rows: row['cmd_wall']=19.
        self.assertFalse(measured_stop(rows, 9.9, 20.71, 10.71)['passed'])

    def test_nonzero_actual_speed_or_command_rejected(self):
        for key in ('vx','wz','cmd_vx','cmd_wz'):
            rows=stationary(); rows[-1][key]=.04
            self.assertFalse(measured_stop(rows,9.9,20.71, 10.71)['passed'], key)

    def test_gap_invalid_frame_nan_and_before_trigger_rejected(self):
        rows=stationary(); rows[-1]['t']+=.2
        self.assertFalse(measured_stop(rows,9.9,20.71, 10.71)['passed'])
        rows=stationary(); rows[-1]['frame']='map'
        self.assertFalse(measured_stop(rows,9.9,20.71, 10.71)['passed'])
        rows=stationary(); rows[-1]['yaw']=math.nan
        self.assertFalse(measured_stop(rows,9.9,20.71, 10.71)['passed'])
        self.assertFalse(measured_stop(stationary(),10.5,20.71, 10.71)['passed'])

if __name__=='__main__': unittest.main()
