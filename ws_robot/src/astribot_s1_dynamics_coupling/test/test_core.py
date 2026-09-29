"""Seeded finite-domain differential, independent frozen Python oracle, no bindings."""
import math
import os
from pathlib import Path
import random
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).parent / 'reference'))
os.environ['ASTRIBOT_BRIDGE_NATIVE_KERNELS'] = '0'
from astribot_s1_dynamics_coupling import arm_reach_metric as oracle


def test_same_input_core_replay():
    rng = random.Random(20260921)
    cases = []
    for _ in range(2000):
        folded = rng.uniform(0, 1)
        full = folded + rng.uniform(.001, 1)
        reach = rng.uniform(-.5, 3)
        cases.append((f'reach {reach} {folded} {full}', oracle.reach_activity(reach, folded, full)))
        x, y = rng.uniform(-2, 2), rng.uniform(-2, 2)
        cases.append((f'xy {x} {y}', oracle.horizontal_reach(x, y)))
        activity, minimum = rng.uniform(-1, 2), rng.uniform(0, 1)
        cases.append((f'scale {activity} {minimum}', oracle.scale_from_activity(activity, minimum)))
        n, r = rng.randrange(15), rng.randrange(16)
        values = [rng.uniform(-4, 4) for _ in range(n)]
        refs = [rng.uniform(-3, 3) for _ in range(r)]
        names = ['j' + str(i) for i in range(n)]
        full = rng.choice([0, 1e-6, 1.0001e-6, rng.uniform(.1, 4)])
        text = f'{full} {n} {r} ' + ' '.join(map(str, values + refs))
        cases.append(('joint ' + text, oracle.joint_deviation_activity(dict(zip(names, values)), names, refs, full)))
        cases.append(('velocity ' + text, oracle.velocity_activity(dict(zip(names, values)), names, full)))
    # Exact thresholds, zip truncation, raw angle difference across +/-pi.
    for line, expected in [('reach .42 .42 .8865', 0), ('reach .8865 .42 .8865', 1),
                           ('reach .8 .5 .5', 'error'), ('reach .8 .7 .5', 'error'),
                           ('joint 1.2 1 1 -3.141592653589793 3.141592653589793', 1),
                           ('joint 1.2 1 0 3.0', 0), ('velocity 2 0 0', 0)]:
        cases.append((line, expected))
    exceptional = [('xy', oracle.horizontal_reach, [1e200, 0.]),
                   ('xy', oracle.horizontal_reach, [1e154, 1e154])]
    for value in (float('nan'), float('inf'), -float('inf')):
        exceptional += [('reach', oracle.reach_activity, args) for args in
                        ([value,.42,.8865], [.65,value,.8865], [.65,.42,value])]
        exceptional += [('scale', oracle.scale_from_activity, args) for args in
                        ([0.,value], [1.,value], [value,.15])]
    for operation, function, args in exceptional:
        try: expected = function(*args)
        except (ValueError, OverflowError): expected = 'error'
        cases.append((operation + ' ' + ' '.join(map(str, args)), expected))
    binary = Path(os.environ.get('DYNAMICS_CORE_PROBE', '/missing/dynamics_core_probe'))
    assert binary.is_file(), 'Native core probe is not implemented/built'
    result = subprocess.run([str(binary)], input='\n'.join(v[0] for v in cases) + '\n',
                            text=True, capture_output=True, check=True, timeout=20)
    values = result.stdout.splitlines()
    assert len(values) == len(cases)
    for (line, expected), actual in zip(cases, values):
        if expected == 'error':
            assert actual == expected, line
        elif math.isnan(expected):
            assert math.isnan(float(actual)), line
        else:
            assert math.isclose(float(actual), expected, rel_tol=3e-14, abs_tol=3e-14), (line, actual, expected)
