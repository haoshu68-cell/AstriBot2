"""Offline fixture setup preconditions. No ROS graph or Gazebo is used."""
import copy
import math
from types import SimpleNamespace as N
import unittest

from verify_kinematic_inventory import setup_sample,PreparationWatchdog,fixture_executions,failed_fixture_cleanup


class KinematicPreparationTest(unittest.TestCase):
    def setUp(self):
        vector=lambda: N(x=0.,y=0.,z=0.)
        self.motion=N(header=N(stamp=N(sec=1,nanosec=0),frame_id='odom'),
                      twist=N(twist=N(linear=vector(),angular=vector())),
                      pose=N(pose=N(position=vector(),orientation=N(x=0.,y=0.,z=0.,w=1.))))
        self.command=N(linear=vector(),angular=vector())
        self.status={'phase':'0'}
        self.received={k:1. for k in ('motion','command','hold_executor')}

    def sample(self,ros=1.1,wall=1.1):
        return setup_sample(self.motion,self.command,self.status,self.received,ros,wall)

    def test_stopped_idle_sample(self):
        self.assertEqual(self.sample()['stamp'],1.)

    def test_active_or_unknown_resource_rejected(self):
        for status in (None,{}, {'phase':'3'}, {'phase':'5'}):
            self.status=status
            self.assertIsNone(self.sample())

    def test_stale_future_and_missing_sources(self):
        self.assertIsNone(self.sample(ros=1.3))
        self.assertIsNone(self.sample(ros=.9))
        self.assertIsNone(self.sample(wall=1.3))
        del self.received['command']
        self.assertIsNone(self.sample())

    def test_wrong_frame_and_nonfinite(self):
        self.motion.header.frame_id='map'
        self.assertIsNone(self.sample())
        self.motion.header.frame_id='odom'
        self.motion.twist.twist.linear.x=math.nan
        self.assertIsNone(self.sample())

    def test_reported_motion_or_nonzero_command_rejected(self):
        self.motion.twist.twist.linear.x=.011
        self.assertIsNone(self.sample())
        self.motion.twist.twist.linear.x=0.
        self.command.angular.z=.01
        self.assertIsNone(self.sample())

    def test_invalid_orientation_rejected(self):
        self.motion.pose.pose.orientation.w=0.
        self.assertIsNone(self.sample())

    def test_unused_command_axes_also_must_be_zero(self):
        self.command.linear.z=.1
        self.assertIsNone(self.sample())

    def test_motion_after_entry_is_latched(self):
        guard=PreparationWatchdog(self.sample())
        self.motion.twist.twist.linear.x=.011
        self.assertFalse(guard.observe(self.sample()))
        self.motion.twist.twist.linear.x=0.
        self.assertFalse(guard.observe(self.sample()))

    def test_freeze_repeated_odom_cannot_renew_window(self):
        guard=PreparationWatchdog(self.sample())
        sample=self.sample();sample['wall']+=.31
        self.assertFalse(guard.observe(sample))

    def test_drift_rotation_or_time_regression_revokes(self):
        for change in ({'x':.006},{'yaw':.011},{'stamp':.99},{'wall':1.}):
            guard=PreparationWatchdog(self.sample())
            self.assertFalse(guard.observe({**self.sample(),**change}))

    def test_advancing_stopped_samples_stay_valid(self):
        first=self.sample();guard=PreparationWatchdog(first)
        for i in range(1,20):
            self.assertTrue(guard.observe({**first,'stamp':first['stamp']+i*.1,'wall':first['wall']+i*.1}))

    def test_exact_fixture_set_is_required(self):
        registry=[{'model':'a','object_id':'object_a'},{'model':'b','object_id':'object_b'}]
        def execution(entity):return {'entity':entity,'epoch':'source','attached':False,'accepted':2,'applied':2}
        diagnostic={'models':[[1,'a'],[2,'b'],[3,'foreign']],'execution':[execution(1),execution(2)]}
        self.assertEqual(fixture_executions(registry,diagnostic),{'a':2,'b':2})
        for executions in ([execution(1)],[execution(1),execution(3)],[execution(1),execution(1)],
                           [execution(1),{**execution(2),'attached':True}],
                           [execution(1),{**execution(2),'accepted':3}]):
            with self.assertRaises(ValueError):fixture_executions(registry,{**diagnostic,'execution':executions})

    def test_failed_prepare_never_detaches_or_changes_scene(self):
        writes=[];fixtures=[{'model':'a','object_id':'object_a'}]
        result=failed_fixture_cleanup(True,fixtures,['a'],lambda *a:writes.append(a),lambda **kw:writes.append(kw))
        self.assertEqual(result,[]);self.assertEqual(writes,[])

    def test_existing_fault_matrix_still_cleans_its_owned_models(self):
        writes=[];fixtures=[{'model':'a','object_id':'object_a'},{'model':'foreign','object_id':'foreign'}]
        result=failed_fixture_cleanup(False,fixtures,['a'],lambda *a:writes.append(a),lambda **kw:writes.append(kw))
        self.assertEqual(len(writes),2);self.assertEqual(result,[{'action':'detach_a','ok':True}])


if __name__=='__main__':unittest.main()
