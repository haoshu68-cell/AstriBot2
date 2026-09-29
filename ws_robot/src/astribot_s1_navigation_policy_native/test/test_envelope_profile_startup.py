"""The direct node must make the same startup admission decisions as Profile.

Run with ENVELOPE_CPP_BINARY pointing to the candidate (no bindings needed).
These cases catch skipped safety budgets and rejected valid hardware evidence.
"""
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

import pytest

POLICY = Path(__file__).resolve().parents[2] / 'astribot_s1_navigation_policy'
sys.path.insert(0, str(POLICY))
from astribot_s1_navigation_policy.profile import Profile


def cases():
    return [
        ('valid_sim', {}, True, True),
        ('clock_mismatch', {}, False, False),
        ('missing_hardware_evidence', {'environment': 'hardware'}, False, False),
        ('negative_sensor_timeout', {'sensor_timeout_s': -1}, True, False),
        ('excess_lease', {'constraint_lease_s': .51}, True, False),
        ('stop_budget', {'reaction_time_s': .1}, True, False),
        ('prediction_step', {'prediction_step_s': 9.0}, True, False),
        ('boolean_dimension', {'half_length_m': True}, True, False),
        ('fraction_above_one', {'scan_min_valid_fraction': 1.01}, True, False),
        ('hardware_evidence_flags', {
            'environment': 'hardware', 'hardware_validated': True,
            'hardware_evidence': {k: True for k in
                ('transport_envelope', 'payload', 'braking', 'latency', 'sensor_coverage')}
        }, False, True),
        ('hardware_evidence_paths', {
            'environment': 'hardware', 'hardware_validated': True,
            'hardware_evidence': {k: 'record.json' for k in
                ('transport_envelope', 'payload', 'braking', 'latency', 'sensor_coverage')}
        }, False, True),
    ]


@pytest.mark.parametrize('name,updates,simulated,accepted', cases(), ids=[c[0] for c in cases()])
def test_profile_admission(tmp_path, name, updates, simulated, accepted):
    binary = os.environ.get('ENVELOPE_CPP_BINARY')
    if not binary:
        pytest.skip('ENVELOPE_CPP_BINARY is required for direct node startup tests')
    data = json.loads((POLICY / 'config/simulation.json').read_text())
    data.update(copy.deepcopy(updates))
    path = tmp_path / f'{name}.json'
    path.write_text(json.dumps(data))
    try:
        Profile.load(path).require_environment(simulated)
        python_accepts = True
    except (ValueError, TypeError, KeyError, AttributeError):
        python_accepts = False
    assert python_accepts is accepted
    env = dict(os.environ, ROS_DOMAIN_ID='108', ROS_LOCALHOST_ONLY='1')
    log_path = tmp_path / 'node.log'
    with log_path.open('w') as log:
        proc = subprocess.Popen([binary, '--ros-args', '-p', f'profile:={path}',
                                 '-p', f'use_sim_time:={str(simulated).lower()}'],
                                stdout=log, stderr=subprocess.STDOUT, env=env)
        remained_alive = False
        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            remained_alive = True
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=3)
    output = log_path.read_text()
    cpp_accepts = remained_alive and 'started in legacy mode' in output
    assert cpp_accepts == accepted, output
    if accepted:
        assert proc.returncode == 0, output
