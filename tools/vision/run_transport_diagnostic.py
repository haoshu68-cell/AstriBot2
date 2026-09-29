#!/usr/bin/env python3
"""Bounded, owned, planning-only transport service regression (no execution)."""
import argparse, json, os, signal, subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if os.environ.get('ROS_DOMAIN_ID')!='89':raise RuntimeError('owned domain 89 required')
if a.output.exists():raise RuntimeError('preserve prior evidence')
for proc in Path('/proc').iterdir():
 if not proc.name.isdigit():continue
 try:
  cmd=proc.joinpath('cmdline').read_bytes().split(b'\0')
  if not cmd or Path(cmd[0].decode()).name not in ('move_group','transport_skill_planner'):continue
  env=dict(x.split('=',1) for x in proc.joinpath('environ').read_bytes().decode().split('\0') if '=' in x)
  if env.get('ROS_DOMAIN_ID')=='89':raise RuntimeError('planning server already running: '+proc.name)
 except (OSError,UnicodeDecodeError):continue
a.output.mkdir(parents=True)
with (a.output/'skills.log').open('w') as log:
 child=subprocess.Popen(['ros2','launch',str(ROOT/'ws_robot/src/astribot_s1_transport/launch/transport_skills.launch.py'),'use_lidar:=false','allow_trajectory_execution:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 rc=None
 try:
  with (a.output/'probe.log').open('w') as probe:
   rc=subprocess.run(['python3',str(ROOT/'tools/vision/diagnose_transport_skill.py'),'--output',str(a.output)],stdout=probe,stderr=subprocess.STDOUT,timeout=150).returncode
 finally:
  if child.poll() is None:child.send_signal(signal.SIGINT)
  try:child.wait(timeout=12)
  except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=8)
 (a.output/'exit.json').write_text(json.dumps(dict(probe_returncode=rc,launch_returncode=child.returncode)))
raise SystemExit(rc if rc is not None else 1)
