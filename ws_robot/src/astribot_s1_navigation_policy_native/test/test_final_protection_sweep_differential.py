"""Offline Python reference vs linked C++ continuous interval protection."""
import json
import os
from pathlib import Path
import random
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

POLICY = Path(__file__).resolve().parents[2] / 'astribot_s1_navigation_policy'
sys.path.insert(0, str(POLICY))
from astribot_s1_navigation_policy.profile import Profile
from astribot_s1_navigation_policy.protection import swept_point_collision


@pytest.mark.parametrize('polygon', [None, [[-.31, 0.], [0., -.31], [.31, 0.], [0., .31]],
    [[-.3, -.1], [-.1, -.3], [.2, -.2], [.3, .1], [.1, .3], [-.2, .2]]])
def test_continuous_sweep_matches_python_for_rotating_translating_and_stationary_commands(polygon):
    binary = os.environ.get('FINAL_PROTECTION_PROBE')
    if not binary: pytest.skip('FINAL_PROTECTION_PROBE required')
    path = POLICY / 'config/simulation.json'
    baseline = Profile.load(str(path))
    profile = SimpleNamespace(**dict(baseline._values))
    if polygon is not None: profile.footprint_xy = np.asarray(polygon, dtype=np.float32).astype(float)
    randomizer = random.Random(731)
    cases = [{'command': [0., 0., 0.], 'points': [[.28, .28]]},
             {'command': [0., 0., .4], 'points': [[.39, .12]]},
             {'command': [.2, 0., 0.], 'points': [[.6, 0.]]},
             {'command': [-.2, -.1, -.4], 'points': []}]
    for _ in range(100):
        cases.append({'command': [randomizer.uniform(-.35, .35), randomizer.uniform(-.2, .2),
                                  randomizer.uniform(-.6, .6)],
                      'points': [[randomizer.uniform(-1., 1.), randomizer.uniform(-1., 1.)]
                                 for _ in range(randomizer.randrange(1, 8))]})
    expected = [bool(swept_point_collision(case['points'], case['command'], profile)) for case in cases]
    if polygon is not None:
        for case in cases: case['polygon'] = polygon
    completed = subprocess.run([binary, str(path)], input=''.join(json.dumps(case) + '\n' for case in cases),
        text=True, capture_output=True, check=True, timeout=20)
    actual = [json.loads(line) for line in completed.stdout.splitlines()]
    assert len(actual) == len(cases)
    for i, (result, collision) in enumerate(zip(actual, expected)):
        assert result == {'collision': collision}, (i, cases[i], result, collision)
