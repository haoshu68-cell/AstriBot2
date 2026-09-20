"""ROS message integration without creating another simulator or sending commands."""
import copy
from dataclasses import asdict
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
try:
    from sensor_msgs.msg import JointState, Image, CameraInfo
    from geometry_msgs.msg import TransformStamped, PoseStamped
    from moveit_msgs.msg import PlanningScene
    from astribot_transport_msgs.action import PlanManipulation
    from astribot_s1_transport.vla_ros import VlaBridge, stamp_ns
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

from astribot_s1_transport.core import Ledger, TaskFailure
from astribot_s1_transport.vla_contract import validate_config
from astribot_s1_transport.vla_policy import ReferencePolicy

CONFIG = validate_config(json.loads((Path(__file__).parents[1]/'config/vla_reference.json').read_text()))


@unittest.skipUnless(HAS_ROS, 'Source ROS and astribot_transport_msgs overlay for integration tests')
class VlaRosTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.io = SimpleNamespace(c=dict(base_frame='base', map_frame='map', tcp='tcp', observation_max_age_s=.5),
            ledger=Ledger(directory.name, 'box'), envelope=SimpleNamespace(epoch=1),
            observation=dict(calibration_epoch='calib'),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_000_000_000)),
            create_subscription=lambda *args:None, check=lambda:None, require_manipulation_hold=lambda:None,
            fresh=lambda m: m is not None and -10_000_000 <= 1_000_000_000-stamp_ns(m) <= 500_000_000,
            scene_context=lambda s:'scene', payload_context=lambda s:'payload',
            transform=lambda *args:np.eye(4), scene=lambda **kwargs:PlanningScene())
        def wait(predicate, timeout):
            if not predicate(): raise TaskFailure('NOT_READY')
        self.io.wait = wait
        self.io.joints = JointState(name=[f'astribot_arm_left_joint_{i}' for i in range(1,8)], position=[0.]*7)
        self.io.joints.header.stamp.sec = 1
        def transform(*args):
            t = TransformStamped(); t.transform.rotation.w = 1.
            return t
        self.io.tf = SimpleNamespace(can_transform=lambda *args:True, lookup_transform=transform)
        self.bridge = VlaBridge(self.io, CONFIG)
        self.addCleanup(lambda:self.bridge.session.close(dict(status='test_done')))
        rgb = Image(height=2, width=2, encoding='rgb8', step=6, data=[100]*12)
        rgb.header.frame_id='optical'; rgb.header.stamp.sec=1
        depth = Image(height=2, width=2, encoding='32FC1', step=8, data=[0]*16)
        depth.header = copy.deepcopy(rgb.header)
        info = CameraInfo(height=2, width=2, k=[100.,0.,1.,0.,100.,1.,0.,0.,1.])
        info.header=copy.deepcopy(rgb.header)
        for key, message in [('rgb',rgb), ('depth',depth), ('info',info)]:
            self.bridge.buffers['head',key].append(message)
        self.goal = PlanManipulation.Goal(operation='PICK')
        self.goal.pre_target = self.bridge.pose(dict(position=[.2,.5,1.2],quaternion=[0.,0.,0.,1.]),'base')
        self.goal.target = copy.deepcopy(self.goal.pre_target)
        self.goal.exit_targets = [copy.deepcopy(self.goal.pre_target)]

    def test_sensor_packet_and_image_layout(self):
        packet = self.bridge.observation()
        self.assertEqual(packet['cameras']['head']['depth']['scale_to_m'], 1.)
        self.assertEqual(packet['cameras']['head']['base_from_camera']['quaternion'], [0.,0.,0.,1.])
        self.bridge.buffers['head','rgb'][-1].step = 7
        with self.assertRaisesRegex(TaskFailure, 'IMAGE_LAYOUT'): self.bridge.observation()

    def test_sensor_skew_and_depth_frame_rejected(self):
        self.io.joints.header.stamp.sec = 0
        self.io.joints.header.stamp.nanosec = 890_000_000
        with self.assertRaisesRegex(TaskFailure, 'MULTIMODAL_SKEW'): self.bridge.observation()
        self.io.joints.header.stamp.sec = 1
        self.io.joints.header.stamp.nanosec = 0
        self.bridge.buffers['head','depth'][-1].header.frame_id = 'wrong'
        with self.assertRaisesRegex(TaskFailure, 'ALIGNED_DEPTH'): self.bridge.observation()

    def changing_policy(self, mutation):
        class Policy(ReferencePolicy):
            def call(policy, endpoint, request):
                response = super().call(endpoint, request)
                if endpoint == 'infer': mutation(response)
                return response
        self.bridge.session.adapter = Policy()
        self.bridge.start()

    def test_nonzero_proposal_applies_to_mtc_goal_only(self):
        self.changing_policy(lambda r:r['action']['target']['position'].__setitem__(0,.205))
        goal = self.bridge.propose(self.goal, PlanningScene())
        self.assertAlmostEqual(goal.target.pose.position.x,.205)
        self.assertEqual(self.bridge.last_decision, 'accepted_for_mtc_planning')
        self.assertEqual(self.io.ledger.object.state, 'WORLD')

    def test_shadow_does_not_change_mtc_goal(self):
        self.bridge.config = dict(CONFIG, mode='shadow')
        self.changing_policy(lambda r:r['action']['target']['position'].__setitem__(0,.205))
        goal = self.bridge.propose(self.goal, PlanningScene())
        self.assertAlmostEqual(goal.target.pose.position.x,.2)
        self.assertEqual(self.bridge.last_decision,'shadow_only')

    def test_context_mutation_blocks_planning(self):
        self.changing_policy(lambda r:setattr(self.io.ledger.object,'version',1))
        with self.assertRaisesRegex(TaskFailure,'CONTEXT_CHANGED'): self.bridge.propose(self.goal,PlanningScene())
        self.assertEqual(self.bridge.last_decision,'rejected')

    def test_joint_drift_blocks_planning(self):
        self.changing_policy(lambda r:self.io.joints.position.__setitem__(0,.1))
        with self.assertRaisesRegex(TaskFailure,'JOINTS_MOVED'): self.bridge.propose(self.goal,PlanningScene())

    def test_camera_calibration_change_blocks_planning(self):
        self.changing_policy(lambda r:self.bridge.buffers['head','info'][-1].k.__setitem__(0,110.))
        with self.assertRaisesRegex(TaskFailure,'CAMERA_CALIBRATION_CHANGED'): self.bridge.propose(self.goal,PlanningScene())

    def test_extrinsic_change_blocks_planning(self):
        original = self.io.tf.lookup_transform
        def mutate(response):
            def changed(*args):
                result = original(*args)
                result.transform.translation.x = .03
                return result
            self.io.tf.lookup_transform = changed
        self.changing_policy(mutate)
        with self.assertRaisesRegex(TaskFailure,'CAMERA_EXTRINSICS_CHANGED'): self.bridge.propose(self.goal,PlanningScene())

    def test_expired_snapshot_blocks_planning(self):
        self.changing_policy(lambda r:setattr(self.io,'get_clock', lambda:SimpleNamespace(
            now=lambda:SimpleNamespace(nanoseconds=6_000_000_000))))
        with self.assertRaisesRegex(TaskFailure,'SNAPSHOT_EXPIRED'): self.bridge.propose(self.goal,PlanningScene())


if __name__ == '__main__': unittest.main()
