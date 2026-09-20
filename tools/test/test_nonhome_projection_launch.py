"""Verify the selected cloud before the slice node can truncate observations."""
import importlib.util
from pathlib import Path
import unittest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions

class HeightInputTests(unittest.TestCase):
    def test_fixed_geometry_uses_unclipped_cloud_and_legacy_keeps_existing_topic(self):
        path=Path(__file__).resolve().parents[2]/'ws_robot/src/astribot_s1_perception/launch/sim_perception.launch.py'
        spec=importlib.util.spec_from_file_location('nonhome_projection_launch',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        arg=next(a for a in module.generate_launch_description().entities
                 if isinstance(a,DeclareLaunchArgument) and a.name=='slam_cloud_topic')
        for mode,expected in [('legacy','/map_scan_filtered'),('fixed_v2','/map_scan')]:
            with self.subTest(mode=mode):
                context=LaunchContext();context.launch_configurations['navigation_geometry_mode']=mode
                self.assertEqual(perform_substitutions(context,arg.default_value),expected)

if __name__=='__main__':unittest.main()
