import unittest
from pathlib import Path
import numpy as np
from astribot_s1_transport.rgbd import localize_orange_box
from astribot_s1_transport.core import TaskFailure


class RGBDTest(unittest.TestCase):
    def inputs(self):
        rgb = np.zeros((40, 40, 3), dtype=np.uint8)
        rgb[10:20, 10:20] = [230, 115, 25]
        depth = np.ones((40, 40), dtype=float)
        depth[10:20, 10:20] = np.linspace(.8, .92, 10)[:, None]
        k = [150., 0., 20., 0., 150., 20., 0., 0., 1.]
        return rgb, depth, k

    def test_projects_measured_depth_and_transform(self):
        rgb, depth, k = self.inputs()
        a = localize_orange_box(rgb, depth, k, np.eye(4), [.06, .06, .12])
        transform = np.eye(4); transform[:3, 3] = [1., 2., 3.]
        b = localize_orange_box(rgb, depth, k, transform, [.06, .06, .12])
        np.testing.assert_allclose(np.array(b['center_m'])-a['center_m'], [1, 2, 3])
        self.assertEqual(a['points'], 100)

    def test_missing_depth_rejects(self):
        rgb, depth, k = self.inputs(); depth[:] = np.nan
        with self.assertRaisesRegex(TaskFailure, 'DEPTH_INVALID'):
            localize_orange_box(rgb, depth, k, np.eye(4), [.06, .06, .12])

    def test_two_objects_are_not_silently_selected(self):
        rgb, depth, k = self.inputs(); rgb[25:35, 25:35] = [230, 115, 25]
        with self.assertRaisesRegex(TaskFailure, 'AMBIGUOUS'):
            localize_orange_box(rgb, depth, k, np.eye(4), [.06, .06, .12])

    def test_unaligned_images_reject(self):
        rgb, depth, k = self.inputs()
        with self.assertRaisesRegex(TaskFailure, 'NOT_ALIGNED'):
            localize_orange_box(rgb, depth[:20], k, np.eye(4), [.06, .06, .12])

    def test_region_rejects_background_but_not_duplicate_targets(self):
        rgb, depth, k = self.inputs()
        expected = localize_orange_box(rgb, depth, k, np.eye(4), [.06, .06, .12])
        rgb[25:35, 25:35] = [230, 115, 25]
        depth[25:35, 25:35] = np.linspace(.8, .92, 10)[:, None]
        found = localize_orange_box(rgb, depth, k, np.eye(4), [.06, .06, .12], expected['center_m'])
        np.testing.assert_allclose(found['center_m'], expected['center_m'])
        with self.assertRaisesRegex(TaskFailure, 'AMBIGUOUS'):
            localize_orange_box(rgb, depth, k, np.eye(4), [.06, .06, .12], expected['center_m'], .2)

    def test_region_does_not_replace_missing_measurement(self):
        rgb, depth, k = self.inputs(); depth[:] = np.nan
        with self.assertRaisesRegex(TaskFailure, 'NOT_VISIBLE'):
            localize_orange_box(rgb, depth, k, np.eye(4), [.06, .06, .12], [-.03, -.03, .86])

    def test_region_separates_same_color_connected_background(self):
        rgb, depth, k = self.inputs()
        expected = localize_orange_box(rgb, depth, k, np.eye(4), [.06, .06, .12])
        rgb[20:35, 10:20] = [230, 115, 25]
        depth[20:35, 10:20] = 3.
        found = localize_orange_box(rgb, depth, k, np.eye(4), [.06, .06, .12], expected['center_m'])
        np.testing.assert_allclose(found['center_m'], expected['center_m'])
        self.assertEqual(found['points'], 100)

    def test_region_keeps_invalid_target_pixels_in_quality_denominator(self):
        rgb, depth, k = self.inputs()
        expected = localize_orange_box(rgb, depth, k, np.eye(4), [.06, .06, .12])
        rgb[20:35, 10:20] = [230, 115, 25]
        depth[20:35, 10:20] = 3.
        depth[10:20, 12:15] = np.nan
        with self.assertRaisesRegex(TaskFailure, 'NOT_VISIBLE'):
            localize_orange_box(rgb, depth, k, np.eye(4), [.06, .06, .12], expected['center_m'])

    def test_region_reports_partial_valid_depth_without_inflating_quality(self):
        rgb, depth, k = self.inputs()
        expected = localize_orange_box(rgb, depth, k, np.eye(4), [.06, .06, .12])
        rgb[20:35, 10:20] = [230, 115, 25]
        depth[20:35, 10:20] = 3.
        depth[10:20, 14] = np.nan
        found = localize_orange_box(rgb, depth, k, np.eye(4), [.06, .06, .12], expected['center_m'])
        self.assertEqual(found['points'], 90)
        self.assertEqual(found['geometry_quality'], .9)

    def test_spatially_separated_targets_connected_by_background_stay_ambiguous(self):
        rgb, depth, k = self.inputs()
        rgb[25:35, 25:35] = [230, 115, 25]
        depth[25:35, 25:35] = np.linspace(.8, .92, 10)[:, None]
        rgb[19:26, 19:26] = [230, 115, 25]
        depth[20:25, 19:26] = 3.
        with self.assertRaisesRegex(TaskFailure, 'AMBIGUOUS'):
            localize_orange_box(rgb, depth, k, np.eye(4), [.06, .06, .12], [0., 0., .86], .2)

    def test_recorded_warehouse_box_touching_floor_stripe(self):
        capture = Path(__file__).resolve().parent / 'data/box_stripe_rgbd.npz'
        with np.load(capture) as frame:
            found = localize_orange_box(frame['rgb'], frame['depth'], frame['k'], frame['tf'],
                                       [.06, .06, .12], [.1, .7, 1.095])
        self.assertLess(np.linalg.norm(np.asarray(found['center_m']) - [.1, .7, 1.095]), .025)
        self.assertGreaterEqual(found['geometry_quality'], .8)


if __name__ == '__main__': unittest.main()
