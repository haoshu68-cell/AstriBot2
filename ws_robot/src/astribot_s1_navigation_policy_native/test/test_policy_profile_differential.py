"""Full resolved-value and rejection differential through a standalone C++ probe."""
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import pytest

POLICY = Path(__file__).resolve().parents[2] / 'astribot_s1_navigation_policy'
sys.path.insert(0, str(POLICY))
from astribot_s1_navigation_policy.profile import Profile

BASE = json.loads((POLICY / 'config/simulation.json').read_text())
RESOLVED = dict(Profile(BASE)._values)


def cases():
    yield 'defaults', BASE
    for name, value in RESOLVED.items():
        if isinstance(value, (float, int)) and not isinstance(value, bool):
            for invalid in (-1, 0, True, '1', None, math.nan, math.inf):
                candidate = copy.deepcopy(RESOLVED)
                candidate[name] = invalid
                yield f'{name}={invalid}', candidate
    for section in ('planning_takeover',):
        for name in RESOLVED[section]:
            for invalid in (-1, 0, True, None):
                candidate = copy.deepcopy(RESOLVED)
                candidate[section][name] = invalid
                yield f'{section}.{name}={invalid}', candidate
    for changes in (
        {'constraint_lease_s': .500001}, {'constraint_lease_s': .5},
        {'path_risk_timeout_s': 10.}, {'input_command_timeout_s': 10.},
        {'command_timeout_s': 10.}, {'narrow_speed_m_s': 1.},
        {'narrow_angular_speed_rad_s': 10.}, {'narrow_centering_speed_m_s': .051},
        {'narrow_centering_tolerance_m': .3}, {'prediction_step_s': 100.},
        {'blocked_confirm_s': 100.}, {'scan_min_valid_fraction': 1.00001},
        {'velocity_confirmation_s': 100.}, {'environment': 'unsupported'},
        {'hardware_validated': 1}, {'schema_version': True},
    ):
        yield str(changes), dict(RESOLVED, **changes)
    for value in ('evidence.json', True, False, 0, 1, [], {}, {'source': 'measured'}):
        candidate = copy.deepcopy(RESOLVED)
        candidate.update(environment='hardware', hardware_validated=True,
                         hardware_evidence={name: value for name in
                            ('transport_envelope', 'payload', 'braking', 'latency',
                             'sensor_coverage')})
        yield f'hardware_evidence={value}', candidate


CASES = list(cases())


def compare(path, simulated=True):
    binary = os.environ.get('POLICY_PROFILE_PROBE')
    if not binary:
        pytest.skip('POLICY_PROFILE_PROBE must select the standalone C++ test binary')
    try:
        reference = Profile.load(path)
        reference.require_environment(simulated)
    except Exception:
        reference = None
    result = subprocess.run([binary, str(path), str(simulated).lower()],
                            capture_output=True, text=True, timeout=3)
    assert (result.returncode == 0) == (reference is not None), result.stderr
    if reference is not None:
        assert json.loads(result.stdout) == dict(reference._values)


@pytest.mark.parametrize('name,data', CASES, ids=[c[0] for c in CASES])
def test_profile_fields_and_budgets(name, data, tmp_path):
    path = tmp_path / 'profile.json'
    path.write_text(json.dumps(data))
    compare(path)


@pytest.mark.parametrize('name', ['simulation', 'social', 'h2_simulation', 'hardware.template'])
def test_repository_profiles(name):
    compare(POLICY / 'config' / f'{name}.json')


@pytest.mark.parametrize('mode', ['valid', 'nested', 'missing', 'null'])
def test_base_profile(mode, tmp_path):
    base = copy.deepcopy(BASE)
    if mode == 'nested':
        base['base_profile'] = 'other.json'
    (tmp_path / 'base.json').write_text(json.dumps(base))
    data = {'base_profile': 'missing.json' if mode == 'missing' else
            None if mode == 'null' else 'base.json', 'payload_mass_kg': 4.0}
    path = tmp_path / 'override.json'
    path.write_text(json.dumps(data))
    compare(path)


@pytest.mark.parametrize('mode', ['valid', 'hardware', 'schema', 'negative', 'boolean', 'bad_range',
                                 'denominator_underflow'])
def test_stop_reference(mode, tmp_path):
    reference = json.loads((POLICY / 'config/measured_stop_reference_20260915.json').read_text())
    if mode == 'hardware': reference['hardware_validated'] = True
    if mode == 'schema': reference['schema_version'] = 2
    if mode == 'negative': reference['polynomial']['engineering_scale'] = -1
    if mode == 'boolean': reference['polynomial']['engineering_scale'] = True
    if mode == 'bad_range': reference['actual_speed_range_m_s'] = [1., .1]
    if mode == 'denominator_underflow':
        reference['polynomial']['engineering_scale'] = 1e-200
        reference['polynomial']['nominal_quadratic_s2_per_m'] = 1e-200
    (tmp_path / 'stop.json').write_text(json.dumps(reference))
    data = dict(BASE, stop_reference_file='stop.json')
    path = tmp_path / 'profile.json'
    path.write_text(json.dumps(data))
    compare(path)


@pytest.mark.parametrize('target', ['profile', 'base', 'stop'])
@pytest.mark.parametrize('suffix', [' {}', ' INVALID'])
def test_json_must_consume_entire_file(target, suffix, tmp_path):
    data = dict(BASE)
    base = dict(BASE)
    stop = json.loads((POLICY / 'config/measured_stop_reference_20260915.json').read_text())
    if target == 'base': data = {'base_profile': 'base.json'}
    if target == 'stop': data['stop_reference_file'] = 'stop.json'
    for name, value in [('profile', data), ('base', base), ('stop', stop)]:
        (tmp_path / f'{name}.json').write_text(json.dumps(value) + (suffix if target == name else ''))
    compare(tmp_path / 'profile.json')


def test_null_sources_rejected_when_applying_stop_reference(tmp_path):
    stop = (POLICY / 'config/measured_stop_reference_20260915.json').read_text()
    (tmp_path / 'stop.json').write_text(stop)
    path = tmp_path / 'profile.json'
    path.write_text(json.dumps(dict(BASE, sources=None, stop_reference_file='stop.json')))
    compare(path)
