"""Offline geometry checks, including cases where valid depth hides the target."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from score_box_roi import score_sample


class BoxRoiTest(unittest.TestCase):
    def setUp(self):
        transform = np.eye(4)
        transform[2, 3] = 2.5
        self.sample = {
            'schema': 'astribot.m5.box_roi/1', 'evaluation_only': True,
            'session_id': 'synthetic', 'task_id': 'test', 'phase': 'CARRY',
            'object_id': 'box', 'camera_id': 'head', 'source_epoch': 'source1',
            'clock_epoch': 'clock1', 'calibration_revision': 1,
            'camera': {'width': 10, 'height': 10, 'frame_id': 'optical',
                       'K': [10, 0, 4.5, 0, 10, 4.5, 0, 0, 1], 'D': [],
                       'depth_convention': 'optical_z_m',
                       'stamps_ns': {'rgb': 100, 'depth': 100, 'info': 100}},
            'world_from_camera': {'matrix': np.eye(4).tolist(), 'frame_id': 'world',
                                  'child_frame_id': 'optical', 'capture_stamp_ns': 100,
                                  'clock_epoch': 'clock1', 'calibration_revision': 1},
            'box_truth': {'matrix': transform.tolist(), 'frame_id': 'world',
                          'object_id': 'box', 'capture_stamp_ns': 100,
                          'clock_epoch': 'clock1', 'size_m': [2, 2, 1],
                          'source': 'synthetic_independent_fixture'},
            'depth_policy': {'min_m': .08, 'max_m': 5.,
                             'source': 'legacy rgbd.py valid-depth predicate'},
            'surface_tolerance_m': .01,
            'surface_tolerance_source': 'synthetic fixture sensitivity, not an acceptance threshold',
        }
        self.depth = np.full((10, 10), 2.)

    def test_front_face_depth_is_axial_z_not_euclidean_range(self):
        report, maps = score_sample(self.sample, self.depth)
        self.assertEqual(report['projected_roi']['pixels'], 100)
        self.assertEqual(report['projected_roi']['valid_depth_fraction'], 1.)
        self.assertEqual(report['surface_comparison']['consistent_fraction_of_roi'], 1.)
        np.testing.assert_allclose(maps['expected_depth_m'], 2.)
        self.assertEqual(report['faces']['z-']['pixels'], 100)
        self.assertEqual(report['coverage_acceptance'], 'UNKNOWN_THRESHOLD')

    def test_missing_depth_stays_in_denominator(self):
        self.depth[:3] = np.nan
        report, _ = score_sample(self.sample, self.depth)
        self.assertEqual(report['projected_roi']['pixels'], 100)
        self.assertEqual(report['projected_roi']['valid_depth_pixels'], 70)
        self.assertAlmostEqual(report['projected_roi']['valid_depth_fraction'], .7)

    def test_occluder_valid_depth_is_not_target_surface_support(self):
        self.depth[:5] = 1.
        report, _ = score_sample(self.sample, self.depth)
        self.assertEqual(report['projected_roi']['valid_depth_fraction'], 1.)
        self.assertEqual(report['surface_comparison']['closer_fraction_of_roi'], .5)
        self.assertEqual(report['surface_comparison']['consistent_fraction_of_roi'], .5)

    def test_behind_surface_is_distinct_from_occluder(self):
        self.depth[:5] = 3.
        report, _ = score_sample(self.sample, self.depth)
        self.assertEqual(report['surface_comparison']['farther_fraction_of_roi'], .5)
        self.assertEqual(report['surface_comparison']['closer_fraction_of_roi'], 0.)

    def test_unknown_tolerance_still_reports_depth_and_residuals(self):
        self.sample['surface_tolerance_m'] = None
        self.sample['surface_tolerance_source'] = None
        self.depth[:5] = 1.
        report, _ = score_sample(self.sample, self.depth)
        self.assertEqual(report['surface_comparison']['status'], 'UNKNOWN_TOLERANCE')
        self.assertEqual(report['surface_comparison']['residual_m']['min'], -1.)
        self.assertNotIn('closer_fraction_of_roi', report['surface_comparison'])

    def test_outside_fov_is_not_zero_error_or_full_coverage(self):
        self.sample['box_truth']['matrix'][0][3] = 100
        report, _ = score_sample(self.sample, self.depth)
        self.assertEqual(report['projected_roi']['pixels'], 0)
        self.assertIsNone(report['projected_roi']['valid_depth_fraction'])
        self.assertEqual(report['geometry_status'], 'NO_PIXEL_RAY_HIT')
        self.assertEqual(report['surface_comparison']['residual_m']['samples'], 0)

    def test_box_behind_camera_has_no_projected_roi(self):
        self.sample['box_truth']['matrix'][2][3] = -2.5
        report, _ = score_sample(self.sample, self.depth)
        self.assertEqual(report['projected_roi']['pixels'], 0)

    def test_subpixel_object_is_not_claimed_to_be_outside_fov(self):
        self.sample['box_truth']['size_m'] = [.001, .001, .001]
        report, _ = score_sample(self.sample, self.depth)
        self.assertEqual(report['geometry_status'], 'NO_PIXEL_RAY_HIT')
        self.assertEqual(report['projected_roi']['pixels'], 0)

    def test_rotated_box_uses_object_axes_and_first_visible_face(self):
        self.sample['camera']['K'][2] = self.sample['camera']['K'][5] = 5.
        transform = np.eye(4)
        transform[:3, :3] = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
        transform[2, 3] = 2.5
        self.sample['box_truth']['matrix'] = transform.tolist()
        _, maps = score_sample(self.sample, self.depth)
        self.assertAlmostEqual(maps['expected_depth_m'][5, 5], 1.5)
        self.assertEqual(maps['face_id'][5, 5], 1)  # x+; camera-facing box face

    def test_world_transform_direction_is_not_inverted(self):
        self.sample['world_from_camera']['matrix'][0][3] = .7
        self.sample['box_truth']['matrix'][0][3] = .7
        _, maps = score_sample(self.sample, self.depth)
        np.testing.assert_allclose(maps['expected_depth_m'], 2.)

    def test_mixed_time_epoch_frame_or_identity_fails(self):
        mutations = [('box_truth', 'capture_stamp_ns', 101),
                     ('box_truth', 'clock_epoch', 'other'),
                     ('box_truth', 'frame_id', 'odom'),
                     ('box_truth', 'object_id', 'other'),
                     ('world_from_camera', 'child_frame_id', 'base'),
                     ('world_from_camera', 'calibration_revision', 2)]
        for section, key, value in mutations:
            sample = copy.deepcopy(self.sample)
            sample[section][key] = value
            with self.subTest(section=section, key=key), self.assertRaises(ValueError):
                score_sample(sample, self.depth)

    def test_wrong_depth_shape_distortion_and_units_are_rejected(self):
        for changes in ({'D': [.1]}, {'depth_convention': 'range_m'},
                        {'width': 20}, {'stamps_ns': {'rgb': 99, 'depth': 100, 'info': 100}}):
            sample = copy.deepcopy(self.sample)
            sample['camera'].update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                score_sample(sample, self.depth)

    def test_non_rigid_truth_is_rejected(self):
        self.sample['box_truth']['matrix'][0][0] = 2.
        with self.assertRaisesRegex(ValueError, 'rigid'):
            score_sample(self.sample, self.depth)

    def test_invalid_and_out_of_range_depth_remain_unknown(self):
        self.depth[:] = np.nan
        self.depth[0, :4] = [0, -.1, .08, 5.]
        report, _ = score_sample(self.sample, self.depth)
        self.assertEqual(report['projected_roi']['valid_depth_pixels'], 0)
        self.assertEqual(report['surface_comparison']['unknown_fraction_of_roi'], 1.)

    def test_independent_visible_mask_can_score_2d_without_box_truth(self):
        del self.sample['box_truth']
        del self.sample['world_from_camera']
        self.sample['visible_annotation'] = {
            'source': 'manual', 'annotation_id': 'a1', 'object_id': 'box',
            'capture_stamp_ns': 100, 'clock_epoch': 'clock1', 'frame_id': 'optical'}
        mask = np.zeros((10, 10), dtype=bool)
        mask[:4] = True
        self.depth[:2] = np.nan
        report, _ = score_sample(self.sample, self.depth, mask)
        self.assertEqual(report['annotated_visible_roi']['pixels'], 40)
        self.assertEqual(report['annotated_visible_roi']['valid_depth_fraction'], .5)
        self.assertEqual(report['geometry_status'], 'NO_BOX_TRUTH')
        self.assertEqual(report['surface_comparison']['status'], 'NO_BOX_TRUTH')

    def test_missing_or_misbound_annotation_cannot_supply_roi(self):
        del self.sample['box_truth']
        with self.assertRaises(ValueError):
            score_sample(self.sample, self.depth)
        self.sample['visible_annotation'] = {
            'source': 'manual', 'annotation_id': 'a1', 'object_id': 'wrong',
            'capture_stamp_ns': 100, 'clock_epoch': 'clock1', 'frame_id': 'optical'}
        with self.assertRaises(ValueError):
            score_sample(self.sample, self.depth, np.ones((10, 10), bool))

    def test_tolerance_requires_provenance_and_does_not_change_acceptance(self):
        self.sample['surface_tolerance_source'] = None
        with self.assertRaises(ValueError):
            score_sample(self.sample, self.depth)
        self.sample['surface_tolerance_source'] = 'synthetic exact test'
        self.sample['surface_tolerance_m'] = 0.
        report, _ = score_sample(self.sample, self.depth)
        self.assertEqual(report['surface_comparison']['consistent_pixels'], 100)
        self.assertEqual(report['coverage_acceptance'], 'UNKNOWN_THRESHOLD')

    def test_cli_preserves_input_hashes_pixel_maps_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            np.save(root/'depth.npy', self.depth)
            # RGB is opaque provenance for the scorer, never used as a truth mask.
            (root/'rgb.bin').write_bytes(bytes([255, 100, 0])*100)
            self.sample['files'] = {'rgb': 'rgb.bin', 'depth': 'depth.npy'}
            (root/'sample.json').write_text(json.dumps(self.sample))
            command = [sys.executable, str(Path(__file__).with_name('score_box_roi.py')),
                       '--sample', str(root/'sample.json'), '--output', str(root/'score')]
            run = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)
            report = json.loads((root/'score'/'report.json').read_text())
            self.assertEqual(set(report['input_sha256']), {'rgb', 'depth'})
            with np.load(root/'score'/'pixel_maps.npz') as maps:
                np.testing.assert_allclose(maps['expected_depth_m'], 2.)
            original = (root/'score'/'report.json').read_bytes()
            again = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(again.returncode, 0)
            self.assertEqual((root/'score'/'report.json').read_bytes(), original)


if __name__ == '__main__':
    unittest.main()
