"""Pure offline tests for joining existing snapshots to independent simulator truth."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from prepare_box_roi_snapshot import assemble_sample, complete_truth_messages


class PrepareBoxSnapshotTest(unittest.TestCase):
    def setUp(self):
        self.meta = {'camera_id': 'head_rgbd', 'source_epoch': 'camera1',
            'calibration_revision': 42, 'capture_stamp_ns': 1_000_000_000,
            'exact_sync_stamps': dict(rgb=1_000_000_000, depth=1_000_000_000, info=1_000_000_000),
            'frame': 'optical', 'width': 10, 'height': 10,
            'K': [10, 0, 4.5, 0, 10, 4.5, 0, 0, 1], 'D': [],
            'capture_age_at_end_sec': .02,
            'camera_health_at_capture': {'valid': True, 'camera_id': 'head_rgbd',
                'source_epoch': 'camera1', 'frame_id': 'optical', 'calibration_revision': 42,
                'capture_stamp': {'sec': 1, 'nanosec': 0}},
            'base_from_camera': {'parent': 'base', 'child': 'optical', 'stamp_ns': 1_000_000_000,
                                 'translation': [.2, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1]},
            'odom_from_camera': {'parent': 'odom', 'child': 'optical', 'stamp_ns': 1_000_000_000,
                                 'translation': [.7, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1]}}
        self.context = {'session_id': 'fixture', 'task_id': 't', 'phase': 'owner_window',
            'clock_epoch': 'clock1', 'world_frame': 'world/default', 'base_frame': 'base',
            'robot_model': 'robot', 'object_model': 'box', 'object_id': 'target',
            'size_m': [2, 2, 1], 'model_from_base': np.eye(4).tolist(),
            'model_binding_evidence': 'actual_robot_description.urdf',
            'object_geometry_evidence': 'actual_box.sdf',
            'clock_epoch_evidence': 'clock_epoch.json',
            'phase_evidence': 'owner_events.jsonl', 'session_json': 'session.json',
            'snapshot_exit_code': 0}
        self.raw = ('header {\n stamp {\n sec: 1\n }\n}\n'
            'pose {\n name: "robot"\n id: 2\n position { x: 1 }\n orientation { w: 1 }\n}\n'
            'pose {\n name: "box"\n id: 3\n position { x: 1.2 z: 2.5 }\n orientation { w: 1 }\n}\n')

    def test_exact_truth_and_tf_avoid_assuming_world_equals_odom(self):
        sample, evidence = assemble_sample(self.meta, self.raw, self.context)
        self.assertAlmostEqual(sample['world_from_camera']['matrix'][0][3], 1.2)
        self.assertAlmostEqual(evidence['world_from_odom'][0][3], .5)
        self.assertEqual(sample['box_truth']['capture_stamp_ns'], 1_000_000_000)
        self.assertEqual(sample['camera']['K'], self.meta['K'])
        self.assertIsNone(sample['surface_tolerance_m'])

    def test_nearest_truth_is_not_used_for_moving_object(self):
        raw = self.raw.replace('sec: 1', 'sec: 2')
        with self.assertRaisesRegex(ValueError, 'exact'):
            assemble_sample(self.meta, raw, self.context)

    def test_omitted_protobuf_quaternion_w_means_zero(self):
        raw = self.raw.replace('orientation { w: 1 }', 'orientation { y: 1 }')
        sample, _ = assemble_sample(self.meta, raw, self.context)
        self.assertAlmostEqual(sample['box_truth']['matrix'][0][0], -1.)
        self.assertAlmostEqual(sample['box_truth']['matrix'][2][2], -1.)

    def test_missing_or_duplicate_entity_is_rejected(self):
        for raw in (self.raw.replace('name: "box"', 'name: "other"'),
                    self.raw+self.raw[self.raw.index('pose {'):]):
            with self.assertRaises(ValueError):
                assemble_sample(self.meta, raw, self.context)

    def test_duplicate_stamp_and_clock_rollback_are_rejected(self):
        for raw in (self.raw+self.raw, self.raw+self.raw.replace('sec: 1', 'sec: 0')):
            with self.assertRaises(ValueError):
                assemble_sample(self.meta, raw, self.context)

    def test_only_truncated_final_message_is_explicitly_skipped(self):
        sample, evidence = assemble_sample(self.meta, self.raw+'header {\n stamp {', self.context)
        self.assertEqual(sample['camera']['stamps_ns']['rgb'], 1_000_000_000)
        self.assertEqual(evidence['truncated_tail_messages'], 1)
        with self.assertRaises(ValueError):
            assemble_sample(self.meta, 'header {\n stamp {\n'+self.raw, self.context)

    def test_missing_quaternion_cannot_be_filled_with_identity(self):
        raw = self.raw.replace('orientation { w: 1 }', 'orientation { }')
        with self.assertRaisesRegex(ValueError, 'quaternion'):
            assemble_sample(self.meta, raw, self.context)

    def test_template_nulls_tf_time_and_failed_capture_are_rejected(self):
        for key in ('task_id', 'clock_epoch', 'model_from_base', 'model_binding_evidence'):
            ctx = copy.deepcopy(self.context);ctx[key] = None
            with self.subTest(key=key), self.assertRaises(ValueError):
                assemble_sample(self.meta, self.raw, ctx)
        meta = copy.deepcopy(self.meta);meta['base_from_camera']['stamp_ns'] += 1
        with self.assertRaises(ValueError):
            assemble_sample(meta, self.raw, self.context)

    def test_cli_joins_existing_files_and_new_sample_is_scorable(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture = root/'capture';capture.mkdir()
            (capture/'camera_info.json').write_text(json.dumps(self.meta))
            (capture/'rgb.png').write_bytes(b'synthetic opaque RGB provenance')
            np.save(capture/'depth.npy', np.full((10, 10), 2.))
            (root/'truth.pbtxt').write_text(self.raw)
            for key in ('model_binding_evidence', 'object_geometry_evidence', 'clock_epoch_evidence',
                        'phase_evidence', 'session_json'):
                (root/self.context[key]).write_text('synthetic provenance '+key)
            (root/'context.json').write_text(json.dumps(self.context))
            command = [sys.executable, str(Path(__file__).with_name('prepare_box_roi_snapshot.py')),
                       '--capture', str(capture), '--truth', str(root/'truth.pbtxt'),
                       '--context', str(root/'context.json'), '--output', str(root/'prepared')]
            run = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)
            sample = json.loads((root/'prepared/sample.json').read_text())
            self.assertEqual(sample['source_sha256']['depth']['path'], str(capture/'depth.npy'))
            self.assertIsNone(sample['surface_tolerance_m'])
            self.assertIn('UNKNOWN_THRESHOLD', run.stdout)
            again = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(again.returncode, 0)

    def test_robot_and_target_cannot_be_same_entity(self):
        self.context['object_model'] = 'robot'
        with self.assertRaises(ValueError):
            assemble_sample(self.meta, self.raw, self.context)

    def test_live_framer_waits_for_next_header_instead_of_accepting_partial_pose(self):
        partial = self.raw[:self.raw.index('pose {')]
        messages, tail = complete_truth_messages(partial)
        self.assertEqual(messages, [])
        messages, tail = complete_truth_messages(tail+self.raw[len(partial):]+'header {\n stamp {')
        self.assertEqual([ts for ts, _ in messages], [1_000_000_000])
        self.assertEqual(tail, 'header {\n stamp {')
        meta = copy.deepcopy(self.meta);meta['capture_age_at_end_sec'] = .3
        with self.assertRaises(ValueError):
            assemble_sample(meta, self.raw, self.context)
        meta['capture_age_at_end_sec'] = -.01
        with self.assertRaises(ValueError):
            assemble_sample(meta, self.raw, self.context)


if __name__ == '__main__':
    unittest.main()
