"""Dry-run only: no ROS graph or simulator is started."""
import json
from pathlib import Path
import subprocess
import sys

SCRIPT=Path(__file__).resolve().parents[1]/'sim_stack_supervisor.py'
BASE=[sys.executable,str(SCRIPT),'--dry-run','--instance','payload_test','--ros-domain-id','94',
      '--navigation-policy','p4','--navigation-geometry-mode','fixed_v2',
      '--corridor-file',str(SCRIPT.parent/'social_navigation/cases/corridor_13.json')]

def test_payload_ledger_identity_and_journal_are_forwarded(tmp_path):
    run=tmp_path/'session'
    value=subprocess.run(BASE+['--payload-source-id','gazebo_empty_v1','--log-dir',str(run)],capture_output=True,text=True)
    assert value.returncode==0,value.stderr
    nav=json.loads(value.stdout)['navigation']
    assert 'enable_payload_ledger:=true' in nav
    assert 'payload_session_id:=payload_test' in nav
    assert 'payload_source_id:=gazebo_empty_v1' in nav
    assert 'payload_journal_path:='+str(run/'payload_ledger.jsonl') in nav

def test_payload_ledger_is_not_implicitly_enabled():
    value=subprocess.run(BASE,capture_output=True,text=True)
    assert value.returncode==0,value.stderr
    assert 'enable_payload_ledger:=true' not in json.loads(value.stdout)['navigation']

def test_payload_source_requires_fixed_v2_isolated_identity():
    for options in [ ['--navigation-geometry-mode','legacy'], ['--instance','', '--ros-domain-id','25'],
                     ['--payload-source-id','bad source'] ]:
        value=subprocess.run(BASE+['--payload-source-id','gazebo_empty_v1']+options,capture_output=True,text=True)
        assert value.returncode!=0
