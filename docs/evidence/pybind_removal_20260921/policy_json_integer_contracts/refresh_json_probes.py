#!/usr/bin/env python3
"""Refresh only probes after the independently owned JSON header is frozen."""
import json,os,subprocess,sys
from pathlib import Path
repo=Path(__file__).resolve().parents[4];pkg=repo/'ws_robot/src/astribot_s1_navigation_policy_native'
build=Path('/tmp/codex_policy_json_integer_contracts_20260921')
variant=sys.argv[1];root=build/variant
commands=json.loads((root/'commands.json').read_text())
selected=[cmd for cmd in commands if any('/test/'+name+'_probe.cpp' in item for item in cmd for name in ('policy_fusion','policy_health'))]
assert len(selected)==2
with (root/'final_probe_build.txt').open('w') as log:
    for cmd in selected:subprocess.run(cmd,check=True,stdout=log,stderr=subprocess.STDOUT)
(root/'final_probe_commands.json').write_text(json.dumps(selected,indent=2)+'\n')
names=('policy_fusion','policy_health','policy_risk','policy_observer_core')
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
for name in names:env[name.upper()+'_PROBE']=str(root/(name+'_probe'))
with (root/'tests.txt').open('w') as log:
    result=subprocess.run([sys.executable,'-m','pytest','-q','-p','no:cacheprovider']+[str(pkg/'test'/('test_'+name+'.py')) for name in names],env=env,stdout=log,stderr=subprocess.STDOUT)
print((root/'tests.txt').read_text(),end='');sys.exit(result.returncode)
