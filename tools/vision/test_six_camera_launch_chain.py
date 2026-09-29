#!/usr/bin/env python3
"""Resolve the real launch chain without spawning ROS or moving the robot."""
import importlib.util,json,subprocess,sys,unittest
from pathlib import Path
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions,perform_substitutions
ROOT=Path(__file__).resolve().parents[2]
FIELDS={'use_wrist_cameras':'true','use_stereo_cameras':'true','camera_calibration_dir':'/fixture/calib','torso_camera_profile':'/fixture/torso.yaml'}
class SixCameras(unittest.TestCase):
 def test_invalid_fixed_geometry_policy_is_rejected_before_spawn(self):
  p=subprocess.run([sys.executable,str(ROOT/'tools/sim_stack_supervisor.py'),'--dry-run',
    '--navigation-geometry-mode','fixed_v2','--navigation-policy','off'],capture_output=True,text=True)
  self.assertEqual(p.returncode,2);self.assertIn('fixed_v2 requires',p.stderr);self.assertFalse(p.stdout.strip())
 def test_supervisor_forwards_explicit_sensor_configuration(self):
  command=[sys.executable,str(ROOT/'tools/sim_stack_supervisor.py'),'--dry-run','--instance','six_camera_test','--ros-domain-id','89']
  for name,value in FIELDS.items():command+=['--'+name.replace('_','-'),value]
  p=subprocess.run(command,capture_output=True,text=True);self.assertEqual(p.returncode,0,p.stderr);args=json.loads(p.stdout)['simulation']
  for name,value in FIELDS.items():self.assertIn(name+':='+value,args)
 def test_bringup_layers_forward_profile_without_replacement(self):
  for path in ['ws_robot/src/astribot_s1_navigation/launch/nav2_full_bringup.launch.py','ws_robot/src/astribot_s1_perception/launch/perception_slam_bringup.launch.py']:
   spec=importlib.util.spec_from_file_location('launch_under_test',ROOT/path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);ld=m.generate_launch_description();ctx=LaunchContext();ctx.launch_configurations.update(FIELDS)
   for action in ld.entities:
    if isinstance(action,DeclareLaunchArgument):action.execute(ctx)
   options=[]
   for action in ld.entities:
    if isinstance(action,IncludeLaunchDescription):options.append(dict(action.launch_arguments))
   selected=[args for args in options if all(name in args for name in FIELDS)]
   self.assertEqual(len(selected),1,path)
   for name,value in FIELDS.items():self.assertEqual(perform_substitutions(ctx,normalize_to_list_of_substitutions(selected[0][name])),value)
if __name__=='__main__':unittest.main()
