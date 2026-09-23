"""Resolve the manipulation launch model arguments without starting ROS nodes."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION = ROOT/'ws_robot/src/astribot_s1_description'
PRESET = DESCRIPTION/'config/simulation_navigation_full/launch_preset.yaml'
SOURCE = ROOT/'ws_robot/src/astribot_s1_transport/launch/transport_skills.launch.py'


class TransportCameraProfileTest(unittest.TestCase):
    def resolve(self, overrides=None):
        spec = importlib.util.spec_from_file_location('transport_launch_test', SOURCE)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with patch.object(module, 'get_package_share_directory', side_effect=lambda p: str(ROOT/'ws_robot/src'/p)):
            description = module.generate_launch_description()
        self.description = description
        context = LaunchContext()
        context.launch_configurations.update(overrides or {})
        for item in description.entities:
            if isinstance(item, DeclareLaunchArgument):
                item.execute(context)
        self.context = context
        includes = [item for item in description.entities if isinstance(item, IncludeLaunchDescription)]
        self.assertEqual(len(includes), 1)
        return {name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
                for name, value in includes[0].launch_arguments}

    def test_defaults_match_single_navigation_preset(self):
        actual = self.resolve()
        parameters = yaml.safe_load(PRESET.read_text())['parameters']
        for name in ('camera_profile', 'torso_camera_profile', 'camera_calibration_dir', 'camera_mounts_profile'):
            self.assertEqual(Path(actual[name]).resolve(), (PRESET.parent/parameters[name]).resolve(), name)
        for name in ('use_camera', 'use_wrist_cameras', 'use_stereo_cameras'):
            self.assertEqual(actual[name], str(parameters[name]).lower(), name)

    def test_explicit_profile_and_sensor_overrides_are_forwarded(self):
        overrides = {'camera_profile': '/fixture/head.yaml', 'torso_camera_profile': '/fixture/torso.yaml',
                     'camera_calibration_dir': '/fixture/calibration', 'camera_mounts_profile': '/fixture/mounts.yaml',
                     'use_camera': 'false', 'use_wrist_cameras': 'false', 'use_stereo_cameras': 'false'}
        actual = self.resolve(overrides)
        for name, value in overrides.items():
            self.assertEqual(actual[name], value, name)

    def test_planning_only_disables_execution_capability(self):
        actual = self.resolve({'allow_trajectory_execution': 'false'})
        self.assertEqual(actual['allow_trajectory_execution'], 'false')
        self.assertEqual(actual['extra_capabilities'], '')

    def test_both_planners_consume_same_resolved_model(self):
        self.resolve()
        urdf, srdf = '<robot name="shared_model"/>', '<robot name="shared_semantic"/>'
        self.context.launch_configurations.update(resolved_robot_description=urdf, resolved_robot_semantic=srdf)
        checked = 0
        for action in self.description.entities:
            if isinstance(action, Node) and action.node_executable in ('mtc_planner', 'transport_skill_planner'):
                # Exercise actual launch_ros parameter normalization; no process is executed.
                parameters = evaluate_parameters(self.context, action._Node__parameters)
                combined = {key: value for item in parameters if isinstance(item, dict) for key, value in item.items()}
                self.assertEqual(combined['robot_description'], urdf)
                self.assertEqual(combined['robot_description_semantic'], srdf)
                checked += 1
        self.assertEqual(checked, 2)


if __name__ == '__main__':
    unittest.main()
