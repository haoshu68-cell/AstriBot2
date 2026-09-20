"""Executor ownership evidence must accompany stationary geometry."""
import copy
from types import SimpleNamespace as N
import unittest
from unittest.mock import Mock
from concurrent.futures import Future
from rclpy.time import Time
from astribot_navigation_msgs.msg import RobotGeometryState
from astribot_s1_transport.ros_backend import RosBackend
from astribot_s1_transport.fixed_hold import StableGeometryReference
from astribot_s1_transport.core import TaskFailure

class HoldTests(unittest.TestCase):
    def make(self,claimed=True,active=True):
        s=RobotGeometryState(complete=True,source_id='s',model_revision='m',attachment_revision='a')
        s.joints.name=['arm'];s.joints.position=[.1];s.joint_position_error_bounds=[.003];s.valid_until=Time(seconds=10.3).to_msg()
        controller=N(type='joint_trajectory_controller/JointTrajectoryController',state='active' if active else 'inactive',
            claimed_interfaces=['arm/position'] if claimed else [])
        future=N(done=lambda:True,result=lambda:N(controller=[controller]))
        self.sent=[]
        return N(hold_id='hold',hold_owner='owner',hold_reference=copy.deepcopy(s),geometry_state=s,
            get_clock=lambda:N(now=lambda:Time(seconds=10.)),controller_future=future,
            controller_query=N(service_is_ready=lambda:False),hold_claims=set(),claims_at=-1.,hold_pub=N(publish=self.sent.append))
    def test_active_position_claims_required(self):
        for claimed,active,expected in [(True,True,True),(False,True,False),(True,False,False)]:
            node=self.make(claimed,active);RosBackend.publish_arm_hold(node)
            self.assertEqual(self.sent[-1].hold_confirmed,expected)
    def test_stale_controller_or_posture_drift_revokes(self):
        node=self.make();node.geometry_state.joints.position=[.2];RosBackend.publish_arm_hold(node)
        self.assertFalse(self.sent[-1].hold_confirmed)
        node=self.make();node.controller_future=None;node.hold_claims={'arm/position'};node.claims_at=9.
        RosBackend.publish_arm_hold(node);self.assertFalse(self.sent[-1].hold_confirmed)

class SettlingTests(unittest.TestCase):
    def sample(self,t,q=.1,revision='a'):
        s=RobotGeometryState(complete=True,attachment_state_confirmed=True,
            source_id='source',model_revision='model',attachment_revision=revision)
        s.header.stamp=Time(nanoseconds=t).to_msg();s.valid_until=Time(nanoseconds=t+300_000_000).to_msg()
        s.joints.name=['arm'];s.joints.position=[q];s.joint_position_error_bounds=[.003]
        return s

    def test_settling_overshoot_must_finish_before_commit(self):
        stable=StableGeometryReference(1_000_000_000)
        values=[-.6037,-.601,-.6001,-.6,-.6,-.6,-.6,-.6,-.6]
        results=[]
        for i,q in enumerate(values):
            t=1_100_000_000+i*100_000_000
            results.append(stable.observe(self.sample(t,q),t+100_000_000))
        self.assertFalse(any(results[:6]));self.assertTrue(results[-1])

    def test_old_repeated_expired_and_changed_scene_do_not_prove_stability(self):
        stable=StableGeometryReference(1_000_000_000)
        self.assertFalse(stable.observe(self.sample(900_000_000),1_000_000_000))
        for i in range(6):
            t=1_100_000_000+i*100_000_000
            result=stable.observe(self.sample(t),t+10_000_000)
        self.assertTrue(result)
        self.assertFalse(stable.observe(self.sample(t),t+20_000_000))
        self.assertFalse(stable.observe(self.sample(t+100_000_000,revision='b'),t+110_000_000))
        self.assertFalse(stable.observe(self.sample(t+200_000_000),t+600_000_000))
        self.assertEqual(len(stable.samples),0)

class ReadQueryTests(unittest.TestCase):
    def test_lost_read_response_retries_without_repeating_a_mutation(self):
        read=Mock(srv_name='/get_planning_scene');read.wait_for_service.return_value=True
        first,second=Future(),Future();read.call_async.side_effect=[first,second]
        node=N(get_scene=read,robot_params=object(),get_logger=lambda:Mock(),
               future=Mock(side_effect=[TaskFailure('WAIT_TIMEOUT'),'scene']))
        self.assertEqual(RosBackend.call(node,read,object()),'scene')
        read.remove_pending_request.assert_called_once_with(first)
        self.assertTrue(first.cancelled());self.assertEqual(read.call_async.call_count,2)
        write=Mock(srv_name='/apply_planning_scene');write.wait_for_service.return_value=True
        node.future=Mock(side_effect=TaskFailure('WAIT_TIMEOUT'))
        with self.assertRaisesRegex(TaskFailure,'WAIT_TIMEOUT'):
            RosBackend.call(node,write,object())
        self.assertEqual(write.call_async.call_count,1)

if __name__=='__main__':unittest.main()
