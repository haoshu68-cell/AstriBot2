"""Geometric requirements for candidate slot profiles, independent of ROS/GPU."""
import math
import unittest

from generate_profiles import Profile, clearance_margin, offset_profile, screen_pair


class ProfilesTest(unittest.TestCase):
    def test_rectangle_offset_is_normal_distance_not_radial_scaling(self):
        p = Profile.polygon([(-.021, -.015), (.021, -.015), (.021, .015), (-.021, .015)])
        hole = offset_profile(p, .003)
        self.assertAlmostEqual(clearance_margin(p, hole, 0.), .003, places=12)
        self.assertEqual(tuple(round(x, 6) for x in hole.bounds()), (-.024, -.018, .024, .018))
        self.assertAlmostEqual(clearance_margin(p, hole, math.pi / 2), -.003, places=12)

    def test_triangle_miter_moves_tip_twice_the_normal_offset(self):
        p = Profile.polygon([(.024 * math.cos(math.pi / 2 + i * 2 * math.pi / 3),
                              .024 * math.sin(math.pi / 2 + i * 2 * math.pi / 3)) for i in range(3)])
        hole = offset_profile(p, .003)
        self.assertAlmostEqual(hole.bounds()[3], .030, places=12)
        self.assertAlmostEqual(clearance_margin(p, hole, 2 * math.pi / 3), .003, places=12)

    def test_d_circle_line_intersection_has_exact_normal_clearance(self):
        p = Profile('d_shape', radius=.020, clip_x=.010)
        hole = offset_profile(p, .0005)
        self.assertAlmostEqual(clearance_margin(p, hole, 0.), .0005, places=12)
        self.assertLess(clearance_margin(p, hole, math.pi), 0.)

    def test_wrong_d_part_has_positive_clearance_in_circle_slot(self):
        part = Profile('d_shape', radius=.020, clip_x=.010)
        hole = offset_profile(Profile('circle', radius=.018), .003)
        result = screen_pair(part, hole, 5.)
        self.assertEqual(result['classification'], 'POSITIVE_CLEARANCE_WITNESS')
        self.assertAlmostEqual(result['best_sampled_clearance_m'], .001, places=12)
        self.assertFalse(result['business_success'])

    def test_tangency_is_not_positive_insertion_clearance(self):
        circle = Profile('circle', radius=.018)
        rect = Profile.polygon([(-.024, -.018), (.024, -.018), (.024, .018), (-.024, .018)])
        self.assertEqual(screen_pair(circle, rect, 30.)['classification'], 'TANGENCY_ONLY_WITNESS')

    def test_no_centered_fit_is_not_global_impossibility(self):
        p = Profile('circle', radius=.018)
        hole = Profile('d_shape', radius=.023, clip_x=.013)
        result = screen_pair(p, hole, 30.)
        self.assertEqual(result['classification'], 'NO_FIT_IN_SAMPLED_CENTERED_POSES')
        self.assertFalse(result['proves_no_fit_under_arbitrary_pose'])
        # A -5 mm x translation actually fits this D slot exactly. Thus the
        # centered negative must never become a mechanical anti-misassembly claim.
        self.assertAlmostEqual(.005 + p.radius, hole.radius)
        self.assertAlmostEqual(-.005 + p.radius, hole.clip_x)

    def test_invalid_geometry_rejected(self):
        with self.assertRaises(ValueError):
            Profile.polygon([(0, 0), (0, 1), (1, 0)])  # clockwise
        with self.assertRaises(ValueError):
            offset_profile(Profile('circle', radius=.018), -0.001)
        with self.assertRaises(ValueError):
            Profile.polygon([(math.cos(i*4*math.pi/5), math.sin(i*4*math.pi/5))
                             for i in range(5)])  # positive local turns, self-intersecting


if __name__ == '__main__':
    unittest.main()
