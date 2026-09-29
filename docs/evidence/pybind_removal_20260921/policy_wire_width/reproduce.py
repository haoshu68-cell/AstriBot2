#!/usr/bin/env python3
"""Private native rebuild and frozen-oracle tests; no ROS or installation operations."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

parser=argparse.ArgumentParser()
parser.add_argument('--sanitizer',action='store_true')
parser.add_argument('--build-dir',type=Path,default=Path('/tmp/codex_policy_wire_width_20260921'))
args=parser.parse_args()
repo=Path(__file__).resolve().parents[4]
pkg=repo/'ws_robot/src/astribot_s1_navigation_policy_native'
build=args.build_dir/('ubsan' if args.sanitizer else 'release');build.mkdir(parents=True,exist_ok=True)
flags=['g++','-std=c++17','-O1' if args.sanitizer else '-O2','-g','-ffp-contract=off','-Wall','-Wextra','-Wpedantic',
       '-I'+str(pkg/'include'),'-I'+str(repo/'ws_robot/src/astribot_s1_robot_geometry/include')]
if args.sanitizer:flags+=['-fsanitize=undefined,float-cast-overflow','-fno-sanitize-recover=all']
source_names=['policy_contracts','navigation_math','policy_fusion','policy_health','policy_sweep','policy_risk','policy_observer_core']
commands=[]
with (build/'build.txt').open('w') as log:
    cmd=flags+['-fsyntax-only',str(Path(__file__).with_name('api_compat.cpp'))]
    commands.append(cmd);subprocess.run(cmd,check=True,stdout=log,stderr=subprocess.STDOUT)
    for name in source_names:
        cmd=flags+['-c',str(pkg/'src'/(name+'.cpp')),'-o',str(build/(name+'.o'))]
        commands.append(cmd);subprocess.run(cmd,check=True,stdout=log,stderr=subprocess.STDOUT)
    for name,objects in [('policy_fusion',['policy_contracts','policy_fusion']),
                         ('policy_health',['policy_contracts','navigation_math','policy_health']),
                         ('policy_risk',['policy_contracts','navigation_math','policy_fusion','policy_sweep','policy_risk']),
                         ('policy_observer_core',['policy_contracts','navigation_math','policy_observer_core'])]:
        cmd=flags+[str(pkg/'test'/(name+'_probe.cpp'))]+[str(build/(n+'.o')) for n in objects]+['-lcrypto','-o',str(build/(name+'_probe'))]
        commands.append(cmd);subprocess.run(cmd,check=True,stdout=log,stderr=subprocess.STDOUT)
(build/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
for name in ('policy_fusion','policy_health','policy_risk','policy_observer_core'):
    env[name.upper()+'_PROBE']=str(build/(name+'_probe'))
with (build/'tests.txt').open('w') as log:
    result=subprocess.run([sys.executable,'-m','pytest','-q','-p','no:cacheprovider']+
        [str(pkg/'test'/('test_'+name+'.py')) for name in ('policy_fusion','policy_health','policy_risk','policy_observer_core')],
        env=env,stdout=log,stderr=subprocess.STDOUT)
print((build/'tests.txt').read_text(),end='')
sys.exit(result.returncode)
