"""Missing planning or policy evidence must fail before scene/motion admission."""
from types import SimpleNamespace as N
from unittest.mock import Mock
import unittest

import numpy as np
from rclpy.time import Time
from astribot_s1_transport.core import Canceled, TaskFailure
from astribot_s1_transport.ros_backend import RosBackend


class AdmissionTests(unittest.TestCase):
    def node(self):
        available=lambda:N(wait_for_server=lambda **_:True)
        node=N(odom=object(),joints=object(),scan=object(),envelope=object(),
            fresh=lambda _:True,transform=lambda *_:np.eye(4),
            c={'map_frame':'map','base_frame':'base','tcp':'tcp'},
            gz_pose=N(wait_for_service=lambda **_:True),arm=available(),
            gripper=available(),head=available(),nav=available(),mtc=available(),
            fixed_v2=True,navigation_policy={'stamp_ns':10_000_000_000},
            get_clock=lambda:N(now=lambda:Time(seconds=10.)),
            scene=Mock(return_value=N(robot_state=N(attached_collision_objects=[]))),
            ledger=N(emit=Mock()),admitted=False,vla=None)
        def wait(predicate,*_):
            if not predicate():raise TaskFailure('WAIT_TIMEOUT')
        node.wait=wait
        return node

    def assert_not_admitted(self,node):
        self.assertFalse(node.admitted)
        node.ledger.emit.assert_not_called()
        node.scene.assert_not_called()

    def test_missing_mtc_rejected_before_scene(self):
        node=self.node();node.mtc.wait_for_server=lambda **_:False
        with self.assertRaisesRegex(TaskFailure,'MTC_SERVER_UNAVAILABLE'):
            RosBackend.admit(node)
        self.assert_not_admitted(node)

    def test_absent_stale_and_future_policy_rejected(self):
        for packet in (None,{'stamp_ns':9_499_999_999},{'stamp_ns':10_000_000_001}):
            with self.subTest(packet=packet):
                node=self.node();node.navigation_policy=packet
                with self.assertRaisesRegex(TaskFailure,'NAVIGATION_POLICY_HEARTBEAT_UNAVAILABLE'):
                    RosBackend.admit(node)
                self.assert_not_admitted(node)

    def test_cancellation_preserves_its_type(self):
        node=self.node();node.navigation_policy=None
        def wait(predicate,*_):
            if not predicate():raise Canceled('user_cancel')
        node.wait=wait
        with self.assertRaises(Canceled):RosBackend.admit(node)
        self.assert_not_admitted(node)

    def test_fresh_policy_and_mtc_allow_admission(self):
        node=self.node();RosBackend.admit(node)
        self.assertTrue(node.admitted)
        node.ledger.emit.assert_called_once()


if __name__=='__main__':unittest.main()
