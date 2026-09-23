"""A changing occupancy scene cannot satisfy planning's quiet window."""
from types import SimpleNamespace
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import numpy as np
from moveit_msgs.msg import PlanningScene

from astribot_s1_transport.core import TaskFailure
from astribot_s1_transport.ros_backend import RosBackend


class SceneStabilityTest(unittest.TestCase):
    def test_hold_epoch_changed_during_revalidation_rejects_next_stage(self):
        scene = PlanningScene()
        state = SimpleNamespace(joint_state=SimpleNamespace(
            name=['astribot_head_joint1'], position=[0.]), attached_collision_objects=[])
        stage = SimpleNamespace(expected_start=state)
        with TemporaryDirectory() as directory:
            backend = SimpleNamespace(
                mtc_bundle=SimpleNamespace(complete=False, index=2, stages=[stage] * 3,
                                           check=lambda *_: stage),
                check=lambda: None, require_manipulation_hold=lambda: None,
                envelope=SimpleNamespace(epoch=1), mtc_hold_epoch=1,
                c={'map_frame': 'map', 'base_frame': 'base', 'object_id': 'payload'},
                mtc_base=np.eye(4), transform=lambda *_: np.eye(4),
                observation=None, mtc_calibration=None,
                scene=lambda **_: scene, stable_scene_snapshot=lambda: scene,
                scene_context=lambda _, include_occupancy=True: 'new' if include_occupancy else 'static',
                mtc_scene_context='old', mtc_static_scene_context='static',
                mtc_context_id='context', revalidate=object(),
                ledger=SimpleNamespace(directory=Path(directory), stage='GRASP_CONFIRM', emit=lambda *_, **__: None),
                joints=state.joint_state, payload_context=lambda _: 'payload', mtc_payload_context='payload')
            backend.check_mtc_execution_context = lambda: RosBackend.check_mtc_execution_context(backend)

            def revalidate(client, request):
                backend.envelope.epoch = 2
                return SimpleNamespace(success=True, context_id='context')

            backend.call = revalidate
            with self.assertRaisesRegex(TaskFailure, 'MTC_HOLD_EPOCH_CHANGED'):
                RosBackend.mtc_stage(backend, 'GRIPPER')

    def test_geometry_change_restarts_window_and_returns_latest_snapshot(self):
        scenes = iter(['old', 'old', 'new', 'new', 'new'])
        times = iter([0., .7, .8, 1.4, 1.9])
        observed = []
        backend = SimpleNamespace(scene=lambda **_: next(scenes), scene_context=lambda s: s)

        def wait(predicate, timeout):
            while True:
                result = predicate()
                observed.append(result)
                if result:
                    return

        backend.wait = wait
        with patch('astribot_s1_transport.ros_backend.time.monotonic', side_effect=lambda: next(times)):
            self.assertEqual(RosBackend.stable_scene_snapshot(backend), 'new')
        self.assertEqual(observed, [False, False, False, False, True])

    def test_timeout_has_explicit_reason_and_source_failure_is_preserved(self):
        for reason, expected in [('WAIT_TIMEOUT', 'MTC_SCENE_NOT_STABLE'),
                                 ('OBSERVATIONS_STALE', 'OBSERVATIONS_STALE')]:
            def wait(*_, reason=reason):
                raise TaskFailure(reason)
            with self.assertRaisesRegex(TaskFailure, expected):
                RosBackend.stable_scene_snapshot(SimpleNamespace(wait=wait))


if __name__ == '__main__':
    unittest.main()
