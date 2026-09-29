#!/usr/bin/env python3
"""Empty physical fixtures need the geometry observer, not a human simulator."""
import importlib.util
from pathlib import Path
import sys,tempfile,unittest,xml.etree.ElementTree as ET
import yaml
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ws_robot/src/astribot_s1_social_navigation'))
from astribot_s1_social_navigation.scenario import prepare_world
SCENES=ROOT/'ws_robot/src/astribot_s1_gazebo_bringup/config/social'
class EmptySceneDependencies(unittest.TestCase):
 def test_observed_empty_scene_requires_hunav_without_actor_assets(self):
  with tempfile.TemporaryDirectory() as folder:
   root=Path(folder);(root/'lib').mkdir();(root/'lib/libastribot_social_scene.so').touch()
   (root/'base.sdf').write_text('<sdf version="1.7"><world name="warehouse"/></sdf>')
   data=yaml.safe_load((SCENES/'h2_empty.yaml').read_text())
   data['hunav_loader']['ros__parameters']['enable_empty_observations']=True
   scene=root/'observed.yaml';scene.write_text(yaml.safe_dump(data))
   with self.assertRaises(FileNotFoundError):prepare_world(root/'base.sdf',scene,root/'out.sdf',None,root)
   (root/'lib/libHuNavSystemPluginIGN.so').touch()
   output=prepare_world(root/'base.sdf',scene,root/'out.sdf',root,root)
   world=ET.parse(output).getroot().find('world')
   self.assertIn('HuNavSystemPluginIGN',[x.attrib['name'] for x in world.findall('plugin')])
   self.assertEqual(world.findall('actor'),[])
   self.assertEqual(world.findall('model'),[])
 def test_empty_observation_flag_requires_boolean(self):
  from astribot_s1_social_navigation.scenario import load_scenario
  with tempfile.TemporaryDirectory() as folder:
   data=yaml.safe_load((SCENES/'h2_empty.yaml').read_text())
   data['hunav_loader']['ros__parameters']['enable_empty_observations']='false'
   scene=Path(folder)/'invalid.yaml';scene.write_text(yaml.safe_dump(data))
   with self.assertRaises(ValueError):load_scenario(scene)
 def test_populated_scene_still_requires_hunav(self):
  with tempfile.TemporaryDirectory() as folder:
   root=Path(folder);(root/'lib').mkdir();(root/'lib/libastribot_social_scene.so').touch();(root/'base.sdf').write_text('<sdf version="1.7"><world name="warehouse"/></sdf>')
   with self.assertRaises(FileNotFoundError):prepare_world(root/'base.sdf',SCENES/'h2_cross_left.yaml',root/'out.sdf',None,root)
 def test_populated_scene_retains_actor_and_human_plugin(self):
  with tempfile.TemporaryDirectory() as folder:
   root=Path(folder);(root/'lib').mkdir();(root/'lib/libastribot_social_scene.so').touch();(root/'lib/libHuNavSystemPluginIGN.so').touch()
   mesh=root/'share/hunav_gazebo_fortress_wrapper/worlds/models/walk.dae';mesh.parent.mkdir(parents=True);mesh.touch()
   (root/'base.sdf').write_text('<sdf version="1.7"><world name="warehouse"/></sdf>')
   output=prepare_world(root/'base.sdf',SCENES/'h2_cross_left.yaml',root/'out.sdf',root,root)
   world=ET.parse(output).getroot().find('world');self.assertTrue(world.findall('actor'))
   self.assertIn('HuNavSystemPluginIGN',[x.attrib['name'] for x in world.findall('plugin')]);self.assertTrue(world.findall('model'))
 def test_empty_scene_keeps_physical_observer_without_hunav(self):
  with tempfile.TemporaryDirectory() as folder:
   root=Path(folder);(root/'lib').mkdir();(root/'lib/libastribot_social_scene.so').touch();(root/'base.sdf').write_text('<sdf version="1.7"><world name="warehouse"/></sdf>')
   output=prepare_world(root/'base.sdf',SCENES/'h2_empty.yaml',root/'out.sdf',None,root)
   world=ET.parse(output).getroot().find('world');names=[x.attrib['name'] for x in world.findall('plugin')]
   self.assertIn('astribot::SocialScene',names);self.assertNotIn('HuNavSystemPluginIGN',names);self.assertEqual(world.findall('actor'),[])
 def test_empty_scene_still_requires_physical_observer(self):
  with tempfile.TemporaryDirectory() as folder:
   root=Path(folder);(root/'base.sdf').write_text('<sdf version="1.7"><world name="warehouse"/></sdf>')
   with self.assertRaises(FileNotFoundError):prepare_world(root/'base.sdf',SCENES/'h2_empty.yaml',root/'out.sdf',None,root)
 def test_empty_launch_never_resolves_hunav_packages(self):
  from launch import LaunchContext
  from launch.substitutions import TextSubstitution
  path=ROOT/'ws_robot/src/astribot_s1_gazebo_bringup/launch/warehouse_sim.launch.py';spec=importlib.util.spec_from_file_location('fixture_launch',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
  ctx=LaunchContext();ctx.launch_configurations['social_scenario']=str(SCENES/'h2_empty.yaml')
  def prefix(name):
   if name.startswith('hunav'):raise AssertionError('Empty fixture requested '+name)
   return '/fixture/'+name
  with patch('ament_index_python.packages.get_package_prefix',side_effect=prefix),patch('ament_index_python.packages.get_package_share_directory',side_effect=prefix),patch('astribot_s1_social_navigation.scenario.prepare_world',return_value='/fixture/world.sdf'),patch('astribot_s1_social_navigation.scenario.prepare_gui_config',return_value='/fixture/gui.config'):
   actions=module._prepare_social(ctx,TextSubstitution(text='/fixture/base.sdf'))
   self.assertTrue(actions)
 def test_observed_empty_launch_retains_real_observer_and_episode_source(self):
  from launch import LaunchContext
  from launch.actions import IncludeLaunchDescription
  from launch.substitutions import TextSubstitution
  path=ROOT/'ws_robot/src/astribot_s1_gazebo_bringup/launch/warehouse_sim.launch.py'
  spec=importlib.util.spec_from_file_location('observed_fixture_launch',path)
  module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
  ctx=LaunchContext();ctx.launch_configurations['social_scenario']=str(SCENES/'h2_empty_observed.yaml')
  with patch('ament_index_python.packages.get_package_prefix',side_effect=lambda name:'/fixture/'+name) as prefix,patch('ament_index_python.packages.get_package_share_directory',side_effect=lambda name:'/fixture/'+name),patch('astribot_s1_social_navigation.scenario.prepare_world',return_value='/fixture/world.sdf') as world,patch('astribot_s1_social_navigation.scenario.prepare_gui_config',return_value='/fixture/gui.config'):
   actions=module._prepare_social(ctx,TextSubstitution(text='/fixture/base.sdf'))
   prefix.assert_any_call('hunav_gazebo_fortress_wrapper')
   self.assertEqual(world.call_args.args[3],'/fixture/hunav_gazebo_fortress_wrapper')
   self.assertEqual(sum(isinstance(action,IncludeLaunchDescription) for action in actions),2)
if __name__=='__main__':unittest.main()
