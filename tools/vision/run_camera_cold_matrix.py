#!/usr/bin/env python3
"""Two owned camera-only cold starts with capture and verified cleanup."""
import argparse,json,os,signal,subprocess,time,shlex
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--manifest',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise RuntimeError('preserve evidence')
a.output.mkdir(parents=True);rows=[]
for i in range(2):
 out=a.output/str(i);out.mkdir();session=out/'session'
 args=['python3',str(ROOT/'tools/vision/camera_reference_session.py'),'--run-root',str(out),'--instance','task_chain_20260921','--domain','89','--wrist-rate','10','--manifest',str(a.manifest.resolve())]
 for prefix in ('runs/task_chain_20260921/ros_ws/install/local_setup.bash','runs/task_chain_20260921/moveit_sensor_plugin/install/local_setup.bash','runs/task_chain_20260921/moveit_shutdown_fix/install/local_setup.bash','tools/setup_mtc_humble.sh','runs/task_chain_20260922/gz_control_fix/install/local_setup.bash'):args+=['--overlay',str(ROOT/prefix)]
 with (out/'supervisor.log').open('w') as log:
  child=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  code=None
  try:
   deadline=time.monotonic()+30
   while not (session/'session.json').exists() and child.poll() is None and time.monotonic()<deadline:time.sleep(.1)
   if not (session/'session.json').exists():raise RuntimeError('supervisor failed to launch')
   command='source '+shlex.quote(str((session/'query_env.sh').resolve()))+'\n'+shlex.join(['python3',str(ROOT/'tools/vision/capture_camera_reference.py'),'--seconds','30','--output',str(out/'capture')])
   with (out/'capture.log').open('w') as cap:code=subprocess.run(['bash','-c',command],stdout=cap,stderr=subprocess.STDOUT,timeout=40).returncode
  finally:
   if child.poll() is None:child.send_signal(signal.SIGTERM)
   child.wait(timeout=25)
  state=json.loads((session/'session.json').read_text());clean=json.loads((session/'owned_cleanup.json').read_text())
  row=dict(repetition=i,capture_returncode=code,supervisor_returncode=child.returncode,cleanup=clean,session=str(session.resolve()))
  row['passed']=code==0 and child.returncode==0 and not clean['remaining']
  rows.append(row);(a.output/'summary.json').write_text(json.dumps(rows,indent=2));print(row,flush=True)
  if not row['passed']:raise RuntimeError('cold start matrix failed; preserve and diagnose')
