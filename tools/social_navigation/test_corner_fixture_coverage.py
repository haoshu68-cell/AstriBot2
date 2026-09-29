#!/usr/bin/env python3
"""Offline regression: actual route coverage, including stationary and cyclic agents."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT/'corner_offline_evidence.py'
spec = importlib.util.spec_from_file_location('offline', SOURCE)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class FixtureRouteCoverage(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=ROOT)
        self.addCleanup(self.tmp.cleanup)
        self.directory = Path(self.tmp.name)
        pixels = np.full((30, 30), 255, dtype=np.uint8)
        pixels[14, 15] = 0  # Occupied world cell [15,16) x [15,16).
        Image.fromarray(pixels).save(self.directory/'map.png')
        self.map = self.directory/'map.yaml'
        self.map.write_text(yaml.safe_dump(dict(image='map.png', resolution=1.,
            origin=[0., 0., 0.], negate=0, occupied_thresh=.65, free_thresh=.196)))

    def result(self, spawn, goals, cyclic=False):
        entries = {i+1: dict(x=x, y=y) for i, (x, y) in enumerate(goals)}
        person = dict(init_pose=dict(x=spawn[0], y=spawn[1]), radius=.1,
                      goals=list(entries), cyclic_goals=cyclic)
        config = dict(agents=['person'], person=person, global_goals=entries)
        path = self.directory/'scenario.yaml'
        path.write_text(yaml.safe_dump(dict(hunav_loader=dict(ros__parameters=config))))
        return m.fixture(self.map, path)['agents'][0]

    def test_stationary_person_on_occupied_cell_is_not_clear(self):
        record = self.result((15.5, 15.5), [])
        self.assertFalse(record['reference_clear'])
        self.assertGreater(record['samples'], 0)
        self.assertLess(record['sampled_reference_disk_static_gap_m'], 0)

    def test_stationary_free_pose_has_actual_samples_and_finite_gap(self):
        record = self.result((5.5, 5.5), [])
        self.assertGreater(record['samples'], 0)
        self.assertTrue(record['reference_clear'])
        self.assertGreater(record['sampled_reference_disk_static_gap_m'], 0)

    def test_stationary_outside_map_is_not_clear(self):
        record = self.result((-1., 5.5), [])
        self.assertFalse(record['reference_clear'])
        self.assertGreater(record['samples'], 0)

    def test_cyclic_closure_targets_first_goal_not_spawn(self):
        # Three sides of a rectangle are free. Last->first goal crosses the
        # occupied cell; last->spawn is the free fourth side and would be wrong.
        record = self.result((24.5, 5.5), [(5.5, 5.5), (5.5, 24.5), (24.5, 24.5)], True)
        self.assertFalse(record['reference_clear'])
        self.assertLess(record['sampled_reference_disk_static_gap_m'], 0)

    def test_noncyclic_route_does_not_add_closing_edge(self):
        record = self.result((24.5, 5.5), [(5.5, 5.5), (5.5, 24.5), (24.5, 24.5)], False)
        self.assertTrue(record['reference_clear'])
        self.assertGreater(record['sampled_reference_disk_static_gap_m'], 0)

    def test_single_cyclic_goal_remains_same_reference_route(self):
        direct = self.result((5.5, 5.5), [(5.5, 24.5)], False)
        cyclic = self.result((5.5, 5.5), [(5.5, 24.5)], True)
        self.assertEqual(cyclic, direct)


if __name__ == '__main__':
    unittest.main()
