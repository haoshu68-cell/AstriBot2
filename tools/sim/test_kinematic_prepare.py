"""Offline fixture setup preconditions. No ROS graph or Gazebo is used."""
import copy
import math
from types import SimpleNamespace as N
import unittest

from verify_kinematic_inventory import setup_sample,PreparationWatchdog,fixture_executions,failed_fixture_cleanup


class KinematicPreparationTest(unittest.TestCase):
    def setUp(self):
        vector=lambda: N(x=0.,y=0.,z=0.)
        self.motion=N(header=N(stamp=N(sec=1,nanosec=0),frame_id='map'),
                      pose=N(pose=N(position=vector(),orientation=N(x=0.,y=0.,z=0.,w=1.))))
        self.command=N(linear=vector(),angular=vector())
        self.status={'phase':'0'}
        self.received={k:1. for k in ('motion','command','hold_executor')}
        self.constraint=N(stamp=N(sec=1,nanosec=0),lease_s=.3,hold=True,
                          max_linear_speed=0.,max_angular_speed=0.)
        self.received['constraint']=1.

    def sample(self,ros=1.1,wall=1.1,**kwargs):
        return setup_sample(self.motion,self.command,self.status,self.received,ros,wall,**kwargs)

    def test_stopped_idle_sample(self):
        self.assertEqual(self.sample()['stamp'],1.)

    def test_active_or_unknown_resource_rejected(self):
        for status in (None,{}, {'phase':'3'}, {'phase':'5'}):
            self.status=status
            self.assertIsNone(self.sample())

    def test_latest_age_future_and_missing_sources(self):
        self.assertIsNotNone(self.sample(ros=1.3))
        self.assertIsNotNone(self.sample(ros=.9))
        self.assertIsNotNone(self.sample(wall=1.3))
        del self.received['command']
        self.assertIsNone(self.sample())

    def test_wrong_frame_and_nonfinite(self):
        self.motion.header.frame_id='odom'
        self.assertIsNone(self.sample())
        self.motion.header.frame_id='map'
        self.motion.pose.pose.position.x=math.nan
        self.assertIsNone(self.sample())

    def test_nonzero_command_rejected(self):
        self.command.angular.z=.01
        self.assertIsNone(self.sample())

    def test_missing_slam_has_no_odometry_fallback(self):
        self.motion=None
        self.assertIsNone(self.sample())

    def test_invalid_orientation_rejected(self):
        self.motion.pose.pose.orientation.w=0.
        self.assertIsNone(self.sample())

    def test_unused_command_axes_also_must_be_zero(self):
        self.command.linear.z=.1
        self.assertIsNone(self.sample())

    def test_event_mode_is_opt_in(self):
        self.assertEqual(self.sample(),self.sample(event_driven_command=False,idle_constraint=self.constraint))
        self.command=None
        self.assertIsNone(self.sample(idle_constraint=self.constraint))

    def test_event_idle_constraint_allows_unobserved_command_without_fabricating_it(self):
        self.command=None;del self.received['command'];before=copy.deepcopy(self.received)
        sample=self.sample(event_driven_command=True,idle_constraint=self.constraint)
        self.assertFalse(sample['command_observed'])
        self.assertEqual(sample['stamp'],1.)
        self.assertEqual(self.received,before)
        self.assertIsNone(self.command)

    def test_event_mode_requires_fresh_zero_hold_constraint(self):
        self.command=None
        self.assertIsNone(self.sample(event_driven_command=True))
        for change in ({'hold':False},{'max_linear_speed':.01},{'max_angular_speed':math.nan},
                       {'lease_s':0.},{'lease_s':.500001},{'lease_s':math.nan}):
            with self.subTest(change=change):
                constraint=copy.deepcopy(self.constraint)
                for key,value in change.items():setattr(constraint,key,value)
                self.assertIsNone(self.sample(event_driven_command=True,idle_constraint=constraint))
        self.received['constraint']=.7
        self.assertIsNotNone(self.sample(event_driven_command=True,idle_constraint=self.constraint))
        del self.received['constraint']
        self.assertIsNone(self.sample(event_driven_command=True,idle_constraint=self.constraint))

    def test_event_observed_command_can_be_silent_but_must_be_finite_and_zero(self):
        self.received['command']=.1
        self.assertTrue(self.sample(event_driven_command=True,idle_constraint=self.constraint)['command_observed'])
        for field in ('linear','angular'):
            for axis in ('x','y','z'):
                for value in (.01,math.nan):
                    with self.subTest(field=field,axis=axis,value=value):
                        setattr(getattr(self.command,field),axis,value)
                        self.assertIsNone(self.sample(event_driven_command=True,idle_constraint=self.constraint))
                        setattr(getattr(self.command,field),axis,0.)

    def test_event_idle_does_not_bypass_slam_or_executor_checks(self):
        self.command=None
        for key in ('motion','hold_executor'):
            self.received[key]=.7
            self.assertIsNotNone(self.sample(event_driven_command=True,idle_constraint=self.constraint))
            self.received[key]=1.
        self.motion.header.stamp=N(sec=0,nanosec=700_000_000)
        self.assertIsNotNone(self.sample(event_driven_command=True,idle_constraint=self.constraint))
        self.motion.header.stamp=N(sec=1,nanosec=0);self.status['phase']='3'
        self.assertIsNone(self.sample(event_driven_command=True,idle_constraint=self.constraint))

    def test_motion_after_entry_is_latched(self):
        guard=PreparationWatchdog(self.sample())
        self.motion.header.stamp.nanosec=100_000_000
        self.motion.pose.pose.position.x=.006
        self.assertFalse(guard.observe(self.sample()))
        self.motion.header.stamp.nanosec=200_000_000
        self.motion.pose.pose.position.x=0.
        self.assertFalse(guard.observe(self.sample()))

    def test_many_opposed_jitters_use_net_displacement(self):
        first=self.sample();guard=PreparationWatchdog(first)
        for i in range(1,41):
            sign=1 if i%2 else -1
            self.assertTrue(guard.observe({**first,'stamp':1.+i*.1,'wall':1.1+i*.1,
                'x':sign*.002,'y':-sign*.002,'yaw':sign*.005}))

    def test_single_direction_net_drift_fails(self):
        first=self.sample();guard=PreparationWatchdog(first)
        for i in range(1,5):
            self.assertTrue(guard.observe({**first,'stamp':1.+i*.1,'wall':1.1+i*.1,'x':i*.001}))
        self.assertFalse(guard.observe({**first,'stamp':1.6,'wall':1.7,'x':.006}))

    def test_yaw_wrap_uses_signed_net_change(self):
        first={**self.sample(),'yaw':math.pi-.002};guard=PreparationWatchdog(first)
        self.assertTrue(guard.observe({**first,'stamp':1.1,'wall':1.2,'yaw':-math.pi+.002}))

    def test_duplicate_sample_does_not_expire_latest_state(self):
        guard=PreparationWatchdog(self.sample())
        sample=self.sample();sample['wall']+=.31
        self.assertTrue(guard.observe(sample))

    def test_drift_rotation_or_time_regression_revokes(self):
        for change in ({'x':.006},{'yaw':.011},{'wall':1.}):
            guard=PreparationWatchdog(self.sample())
            self.assertFalse(guard.observe({**self.sample(),'stamp':1.1,**change}))

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
