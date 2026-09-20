import unittest
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


if __name__ == '__main__': unittest.main()
