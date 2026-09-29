#!/usr/bin/env python3
"""Own bounded planning-only MoveIt probes in an already-owned simulation domain.

Source the session environment before calling. Never starts robot execution.
"""
import argparse,json,os,signal,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--repetitions',type=int,default=3);a=p.parse_args()
 if not 1<=a.repetitions<=3:raise ValueError('bounded probe: 1..3 repetitions')
 if os.environ.get('ROS_DOMAIN_ID')!='89':raise RuntimeError('this diagnostic is reserved for owned task_chain domain 89')
 if a.output.exists():raise RuntimeError('preserve previous evidence; use a fresh output directory')
 # Do not create a duplicate planning server in the owned simulation domain.
 for proc in Path('/proc').iterdir():
  if not proc.name.isdigit():continue
  try:
   cmd=proc.joinpath('cmdline').read_bytes().split(b'\0')
   if not cmd or Path(cmd[0].decode()).name!='move_group':continue
   env=dict(x.split('=',1) for x in proc.joinpath('environ').read_bytes().decode().split('\0') if '=' in x)
   if env.get('ROS_DOMAIN_ID')=='89':raise RuntimeError('existing move_group in domain 89: '+proc.name)
  except (OSError,UnicodeDecodeError):continue
 a.output.mkdir(parents=True)
 rows=[]
 for index in range(a.repetitions):
  out=a.output/str(index);out.mkdir()
  with (out/'move_group.log').open('w') as log:
   child=subprocess.Popen(['ros2','launch',str(ROOT/'ws_robot/src/astribot_s1_moveit_config/launch/move_group.launch.py'),'use_lidar:=false','allow_trajectory_execution:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
   loaded=[];rc=None
   try:
    with (out/'probe.log').open('w') as probe_log:rc=subprocess.run(['python3',str(ROOT/'tools/vision/diagnose_ready_right.py'),'--output',str(out)],stdout=probe_log,stderr=subprocess.STDOUT,timeout=100).returncode
    for proc in Path('/proc').iterdir():
     if not proc.name.isdigit():continue
     try:
      stat=proc.joinpath('stat').read_text().split(') ',1)[1].split()
      if int(stat[2])!=child.pid:continue
      for line in proc.joinpath('maps').read_text().splitlines():
       if any(s in line for s in ('trajectory_execution_manager.so','moveit_ros_move_group','pointcloud_octomap_updater')):loaded.append(line.split()[-1])
     except (OSError,ValueError):continue
   finally:
    if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
    try:child.wait(timeout=8)
    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=8)
   (out/'loaded_libraries.json').write_text(json.dumps(sorted(set(loaded)),indent=2))
  text=(out/'move_group.log').read_text()
  row=dict(repetition=index,probe_returncode=rc,launch_returncode=child.returncode,
           clean_child_exit='process has finished cleanly' in text and 'process has died' not in text,
           patched_library_loaded=any('moveit_shutdown_fix' in path and 'trajectory_execution_manager' in path for path in loaded))
  rows.append(row);print(json.dumps(row),flush=True)
  (a.output/'lifecycle_summary.json').write_text(json.dumps(rows,indent=2))
 return 0 if all(r['probe_returncode']==0 and r['clean_child_exit'] and r['patched_library_loaded'] for r in rows) else 1

if __name__=='__main__':raise SystemExit(main())
