#!/usr/bin/env python3
"""Activate/deactivate two C++ wrist pipelines for bounded simulation validation."""
import argparse,subprocess,signal,os,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=int,default=60);p.add_argument('--rate',type=float,default=5.);a=p.parse_args()
if os.environ.get('ROS_DOMAIN_ID')!='89' or not 1<=a.seconds<=1800:raise ValueError('owned domain 89 and 1..1800 seconds required')
if a.output.exists():raise ValueError('preserve existing evidence; choose a new output directory')
a.output.mkdir(parents=True);children=[];logs=[]
try:
 for side in ['left','right']:
  camera=side+'_wrist_rgbd';log=(a.output/(camera+'.log')).open('w');logs.append(log)
  cmd=['ros2','launch',str(ROOT/'ws_robot/src/astribot_s1_perception_components/launch/rgbd_vision_pipeline.launch.py'),f'camera_id:={camera}',f'expected_rate_hz:={a.rate}','enable_cloud:=true','enable_pose:=false','enable_detection_gate:=false',f'frame_id:=astribot_s1/astribot_arm_{side}_link_7/{camera}_sensor',f'color_topic:=/camera/raw/{camera}/image',f'depth_topic:=/camera/raw/{camera}/depth_image',f'camera_info_topic:=/camera/raw/{camera}/camera_info','calibration_revision:=2026092101']
  children.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
 with (a.output/'capture.log').open('w') as log:r=subprocess.run(['python3',str(ROOT/'tools/vision/capture_camera_reference.py'),'--output',str(a.output/'capture'),'--seconds',str(a.seconds)],stdout=log,stderr=subprocess.STDOUT,timeout=a.seconds+15)
finally:
 for child in children:
  if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
 for child in children:
  try:child.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=8)
 for log in logs:log.close()
raise SystemExit(r.returncode)
