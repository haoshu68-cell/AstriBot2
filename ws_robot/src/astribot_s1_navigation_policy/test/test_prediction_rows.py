from dataclasses import replace
import unittest

import numpy as np

from astribot_s1_navigation_policy.contracts import Covariance3, MetricBox, Stamp, Vec3, Version
from astribot_s1_navigation_policy.fusion import translate
from astribot_s1_navigation_policy.ports import Prediction, PredictionModel, TrackedObstacle, WorldSnapshot
from astribot_s1_navigation_policy.swept_geometry import obstacle_bounds
from astribot_s1_navigation_policy.world_geometry import prediction_rows


def sample_world():
    stamp = Stamp(1_000_000_000, 'ros', 1)
    box = MetricBox(Vec3(2., -1., .5), Vec3(.2, .4, 1.),
                    Covariance3((.01, 0., 0., 0., .04, 0., 0., 0., 0.)))
    steps = ((0, box), (100_000_000, replace(box, center_m=Vec3(1., 0., .5))),
             (300_000_000, replace(box, size_m=Vec3(.8, .1, 1.))))
    tracks = (
        TrackedObstacle('static', 'odom', stamp, box, (), ('scan',)),
        TrackedObstacle('model', 'odom', stamp, box, (), ('vision',),
                        PredictionModel(Vec3(.2, -.4, 0.), .03,
                                        ((100_000_000, .1), (300_000_000, .3)))),
        TrackedObstacle('explicit', 'odom', stamp, box,
                        tuple(Prediction(t, b) for t, b in steps), ('scan',)),
        TrackedObstacle('empty_model', 'odom', stamp, box, (), ('vision',),
                        PredictionModel(Vec3(0., 0., 0.), 0., ())),
    )
    return WorldSnapshot(Version('goal', 1, 1, 1), stamp, 'odom', tracks, (), (), 1)


class PredictionRowsTests(unittest.TestCase):
    def test_mixed_predictions_match_scalar_geometry_for_all_modes(self):
        world = sample_world()
        for include_current in (False, True):
            for swept in (False, True):
                with self.subTest(include_current=include_current, swept=swept):
                    expected = []
                    for owner, track in enumerate(world.tracks):
                        previous = track.geometry
                        if include_current:
                            expected.append((owner, 0, *obstacle_bounds(previous)))
                        model = track.prediction_model
                        samples = (tuple(Prediction(ns, translate(track.geometry, model.velocity,
                                                t, t*t*model.variance_m2_s2)) for ns, t in model.steps)
                                   if model is not None else track.predictions)
                        for sample in samples:
                            lo, hi = obstacle_bounds(sample.geometry, previous if swept else None)
                            expected.append((owner, sample.offset_ns, lo, hi))
                            previous = sample.geometry
                    rows = prediction_rows(world, include_current, swept)
                    np.testing.assert_array_equal(rows.owners, [x[0] for x in expected])
                    np.testing.assert_array_equal(rows.offsets_ns, [x[1] for x in expected])
                    np.testing.assert_allclose(rows.lower, [x[2] for x in expected], rtol=0, atol=1e-15)
                    np.testing.assert_allclose(rows.upper, [x[3] for x in expected], rtol=0, atol=1e-15)
                    for array in (rows.owners, rows.offsets_ns, rows.lower, rows.upper):
                        self.assertFalse(array.flags.writeable)

    def test_empty_world_and_current_only_tracks(self):
        world = sample_world()
        for tracks in ((), (world.tracks[0],), (world.tracks[3],)):
            for swept in (False, True):
                rows = prediction_rows(replace(world, tracks=tracks), swept=swept)
                self.assertEqual(rows.lower.shape, (0, 2))
                self.assertEqual(rows.upper.shape, (0, 2))
                self.assertEqual(rows.owners.dtype, np.int64)
                self.assertEqual(rows.offsets_ns.dtype, np.int64)

    def test_replaced_snapshot_is_not_reused(self):
        world = sample_world()
        before = prediction_rows(world, True)
        tracks = tuple(replace(t, geometry=replace(t.geometry, center_m=Vec3(9., 8., .5)))
                       for t in world.tracks)
        after = prediction_rows(replace(world, tracks=tracks, observation_seq=2), True)
        self.assertFalse(np.array_equal(before.lower, after.lower))


if __name__ == '__main__':
    unittest.main()
