#!/usr/bin/env python3
"""Rebuild and compare isolated sweep/risk cores; never install or start ROS."""
import argparse
import os
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--ubsan', action='store_true')
args = parser.parse_args()
root = Path(__file__).resolve().parents[4]
package = root / 'ws_robot/src/astribot_s1_navigation_policy_native'
geometry = root / 'ws_robot/src/astribot_s1_robot_geometry'
work = Path('/tmp/codex_policy_risk_20260921/reproduce')
work.mkdir(parents=True, exist_ok=True)
flags = ['-O1', '-g', '-fsanitize=undefined', '-fno-sanitize-recover=all',
         '-fno-omit-frame-pointer'] if args.ubsan else ['-O2']
env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
for component in ('sweep', 'risk'):
    binary = work / (component + ('_ubsan' if args.ubsan else '_probe'))
    sources = ['src/navigation_math.cpp', 'src/policy_sweep.cpp']
    if component == 'risk':
        sources += ['src/policy_contracts.cpp', 'src/policy_fusion.cpp', 'src/policy_risk.cpp']
    sources.append('test/policy_' + component + '_probe.cpp')
    subprocess.run(['c++', '-std=c++17', *flags, '-ffp-contract=off',
                    '-I' + str(package / 'include'), '-I' + str(geometry / 'include'),
                    *(str(package / source) for source in sources), '-o', str(binary)], check=True)
    env['POLICY_' + component.upper() + '_PROBE'] = str(binary)
    subprocess.run(['python3', '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
                    str(package / 'test/test_policy_' + component + '.py')], env=env, check=True)
subprocess.run(['python3', str(package / 'test/verify_policy_risk_extra.py')], env=env, check=True)
