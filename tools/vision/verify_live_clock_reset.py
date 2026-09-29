#!/usr/bin/env python3
"""Pause/reset/resume only the recorded owned warehouse; observe real C++ health."""
import argparse, json, os, subprocess, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from astribot_perception_msgs.msg import CameraHealth
from rosgraph_msgs.msg import Clock
p=argparse.ArgumentParser();p.add_argument('--session',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise RuntimeError('preserve previous evidence')
s=json.loads(a.session.read_text());pid=s['launch_pid'];stat=Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1].split();assert stat[19]==s['launch_start_ticks']
env=dict(x.split('=',1) for x in Path(f'/proc/{pid}/environ').read_bytes().decode().split('\0') if '=' in x)
for key in ('ROS_DOMAIN_ID','IGN_PARTITION','ASTRIBOT_SIM_INSTANCE'):assert env[key]==s['env'][key]==os.environ[key]
assert env['ASTRIBOT_SIM_INSTANCE']=='task_chain_20260921' and 'enable_effort_drive:=false' in s['command']
rclpy.init(args=['--ros-args','-p','use_sim_time:=true']);n=Node('owned_clock_reset_validation');health={};events=[];clock=[];subs=[];phase='initial';start=time.monotonic()
def on_health(camera,msg):
 row=dict(camera=camera,phase=phase,elapsed=time.monotonic()-start,valid=msg.valid,epoch=msg.source_epoch,state=msg.state,reason=msg.reason_code,stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,capture=msg.capture_stamp.sec+msg.capture_stamp.nanosec*1e-9)
 health[camera]=row;events.append(row)
for cam in ('head_rgbd','torso_rgbd'):
 subs.append(n.create_subscription(CameraHealth,'/perception/camera_health/'+cam,lambda msg,c=cam:on_health(c,msg),10))
subs.append(n.create_subscription(Clock,'/clock',lambda msg:clock.append(msg.clock.sec+msg.clock.nanosec*1e-9),10))
def wait_until(predicate,seconds):
 deadline=time.monotonic()+seconds
 while time.monotonic()<deadline:
  rclpy.spin_once(n,timeout_sec=.01)
  if predicate():return True
 return False
def control(request):
 r=subprocess.run(['ign','service','-s','/world/default/control','--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean','--timeout','3000','--req',request],capture_output=True,text=True,timeout=5)
 if r.returncode or 'data: true' not in r.stdout:raise RuntimeError(r.stdout+r.stderr)
report={'scope':'owned camera-only Gazebo clock fault; health gate observed, no arm/nav/VLA/inference execution','session':str(a.session.resolve()),'cases':[]}
try:
 assert wait_until(lambda:len(health)==2 and all(h['valid'] for h in health.values()) and len(clock)>2,20),'baseline not ready'
 old={c:h['epoch'] for c,h in health.items()};old_clock=clock[-1]
 phase='paused';control('pause: true');assert wait_until(lambda:all(not h['valid'] and h['phase']==phase for h in health.values()),2),'health stayed valid under stopped clock'
 report['cases'].append({'name':'pause_revokes_health','passed':True,'observed':dict(health)})
 phase='reset';control('pause: true, reset: {time_only: true}');control('pause: false')
 assert wait_until(lambda:len(health)==2 and all(h['valid'] and h['epoch']!=old[c] for c,h in health.items()),15),'fresh epochs did not recover'
 assert any(b<a for a,b in zip(clock,clock[1:])),'ROS clock did not rewind'
 report['cases'].append({'name':'reset_changes_epoch_and_recovers','passed':True,'old_epochs':old,'observed':dict(health),'old_clock':old_clock,'new_clock':clock[-1]})
 report['passed']=True
except Exception as error:
 report.update(passed=False,error=str(error));raise
finally:
 try:control('pause: false')
 finally:
  report['events']=events;a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
