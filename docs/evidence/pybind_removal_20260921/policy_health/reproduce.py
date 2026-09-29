#!/usr/bin/env python3
"""Rebuild/run only the standalone health core; never starts ROS or installs."""
import argparse
import os
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--ubsan', action='store_true')
args = parser.parse_args()
repository = Path(__file__).resolve().parents[4]
package = repository / 'ws_robot/src/astribot_s1_navigation_policy_native'
work = Path('/tmp/codex_policy_health_20260921/reproduce')
work.mkdir(parents=True, exist_ok=True)
binary = work / ('policy_health_ubsan' if args.ubsan else 'policy_health_probe')
flags = ['-O1', '-g', '-fsanitize=undefined', '-fno-sanitize-recover=all',
         '-fno-omit-frame-pointer'] if args.ubsan else ['-O2']
command = ['c++', '-std=c++17', *flags, '-Wall', '-Wextra', '-Wpedantic', '-ffp-contract=off',
           '-I' + str(package / 'include')]
command += [str(package / source) for source in (
    'src/policy_contracts.cpp', 'src/policy_health.cpp', 'src/navigation_math.cpp',
    'test/policy_health_probe.cpp')]
subprocess.run(command + ['-o', str(binary)], check=True)
environment = dict(os.environ, POLICY_HEALTH_PROBE=str(binary), PYTHONDONTWRITEBYTECODE='1')
subprocess.run(['python3', '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
                str(package / 'test/test_policy_health.py')], env=environment, check=True)
