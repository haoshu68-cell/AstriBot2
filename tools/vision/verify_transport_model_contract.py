#!/usr/bin/env python3
"""Evaluate launch substitutions only; no robot processes or movement."""
import importlib.util
from pathlib import Path
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from launch.utilities import perform_substitutions,normalize_to_list_of_substitutions
ROOT=Path(__file__).resolve().parents[2]
def load(path,name):
 s=importlib.util.spec_from_file_location(name,ROOT/path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
transport=load('ws_robot/src/astribot_s1_transport/launch/transport_skills.launch.py','transport_launch')
move=load('ws_robot/src/astribot_s1_moveit_config/launch/move_group.launch.py','move_launch')
for mounts,lidar in [('reference','true'),('reference','false'),('original','true')]:
 c=LaunchContext();c.launch_configurations['use_lidar']=lidar
 if mounts=='original':c.launch_configurations['camera_mounts_profile']=''
 description=transport.generate_launch_description()
 for action in description.entities:
  if isinstance(action,DeclareLaunchArgument):action.execute(c)
 include=next(a for a in description.entities if isinstance(a,IncludeLaunchDescription))
 for k,v in include.launch_arguments:c.launch_configurations[k]=perform_substitutions(c,normalize_to_list_of_substitutions(v))
 for action in move.generate_launch_description().entities:
  if isinstance(action,DeclareLaunchArgument):action.execute(c)
 for action in move._prepare_robot_descriptions(c):action.execute(c)
 count=0
 for action in description.entities:
  if isinstance(action,Node) and action.node_executable in ('transport_skill_planner','mtc_planner'):
   params={}
   for entry in evaluate_parameters(c,action._Node__parameters):
    if isinstance(entry,dict):params.update(entry)
   assert params['robot_description']==c.launch_configurations['resolved_robot_description']
   assert params['robot_description_semantic']==c.launch_configurations['resolved_robot_semantic']
   count+=1
 assert count==2
 print('PASS three planners share model bytes:',mounts,'lidar='+lidar)
