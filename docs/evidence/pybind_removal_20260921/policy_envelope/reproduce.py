#!/usr/bin/env python3
"""Private compile and oracle replay only; no ROS launch, install, or Git mutation."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

parser=argparse.ArgumentParser()
parser.add_argument('--sanitizer',action='store_true')
parser.add_argument('--build-dir',type=Path,default=Path('/tmp/codex_policy_envelope_20260921'))
args=parser.parse_args()
repo=Path(__file__).resolve().parents[4]
package=repo/'ws_robot/src/astribot_s1_navigation_policy_native'
args.build_dir.mkdir(parents=True,exist_ok=True)
name='ubsan' if args.sanitizer else 'release'
binary=args.build_dir/('policy_envelope_probe_'+name)
includes=[package/'include',repo/'ws_robot/src/astribot_s1_robot_geometry/include',
          repo/'ws_robot/install/astribot_navigation_msgs/include/astribot_navigation_msgs']
includes += [Path('/opt/ros/humble/include')/name for name in
             ['builtin_interfaces','geometry_msgs','std_msgs','rosidl_runtime_cpp','rosidl_runtime_c','rcutils','rosidl_typesupport_interface']]
command=['g++','-std=c++17','-g','-ffp-contract=off','-Wall','-Wextra','-Wpedantic']
command += ['-O1','-fsanitize=undefined,float-cast-overflow','-fno-sanitize-recover=all'] if args.sanitizer else ['-O2']
command += ['-I'+str(p) for p in includes]
command += [str(package/p) for p in ['src/policy_contracts.cpp','src/policy_profile.cpp',
                                    'src/policy_envelope.cpp','test/policy_envelope_probe.cpp']]
command += ['-lcrypto','-o',str(binary)]
(args.build_dir/('compile_'+name+'.json')).write_text(json.dumps(command,indent=2)+'\n')
with (args.build_dir/('build_'+name+'.txt')).open('w') as out:
    subprocess.run(command,check=True,stdout=out,stderr=subprocess.STDOUT)
env=dict(os.environ,POLICY_ENVELOPE_PROBE=str(binary),PYTHONDONTWRITEBYTECODE='1',
         POLICY_ENVELOPE_FAILURE=str(args.build_dir/'failure.json'),
         POLICY_ENVELOPE_STATS=str(args.build_dir/('stats_'+name+'.json')),UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
with (args.build_dir/('test_'+name+'.txt')).open('w') as out:
    result=subprocess.run([sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
        str(package/'test/test_policy_envelope.py')],env=env,stdout=out,stderr=subprocess.STDOUT)
print((args.build_dir/('test_'+name+'.txt')).read_text(),end='')
sys.exit(result.returncode)
