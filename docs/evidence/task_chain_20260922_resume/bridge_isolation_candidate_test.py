#!/usr/bin/env python3
"""Inspect real launch arguments; no simulator or control process is spawned."""
import importlib.util
from pathlib import Path
import unittest
from launch import LaunchContext
from launch.actions import OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions,perform_substitutions
ROOT=Path(__file__).resolve().parents[2]
class CameraBridgeIsolation(unittest.TestCase):
 def test_each_camera_isolated_from_clock_and_other_cameras(self):
  spec=importlib.util.spec_from_file_location('warehouse',ROOT/'ws_robot/src/astribot_s1_gazebo_bringup/launch/warehouse_sim.launch.py')
  module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
  ctx=LaunchContext();ctx.launch_configurations['robot_name']='astribot_s1'
  actions=[a for a in module.generate_launch_description().entities if isinstance(a,OpaqueFunction) and a._OpaqueFunction__function is module._camera_bridge_node]
  self.assertEqual(len(actions),7)
  args=[[perform_substitutions(ctx,normalize_to_list_of_substitutions(x)) for x in a._OpaqueFunction__kwargs['arguments']] for a in actions]
  core=[x for x in args if any('/clock@' in v for v in x)]
  self.assertEqual(len(core),1);self.assertEqual(len(core[0]),6)
  cameras=['head_rgbd','torso_rgbd','left_wrist_rgbd','right_wrist_rgbd','head_stereo_left','head_stereo_right']
  for camera in cameras:
   groups=[x for x in args if any('/'+camera+'/' in v for v in x)]
   self.assertEqual(len(groups),1);self.assertEqual(len(groups[0]),2 if 'stereo' in camera else 3)
   self.assertTrue(all('/'+camera+'/' in v for v in groups[0]))
if __name__=='__main__':unittest.main()
