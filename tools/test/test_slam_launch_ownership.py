"""Check the real launch graph without starting sensors or acquiring hardware."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions


ROOT = Path(__file__).resolve().parents[2]


def description(package, filename):
    path = ROOT / 'ws_robot/src' / package / 'launch' / filename
    spec = importlib.util.spec_from_file_location(filename, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def context(**values):
    result = LaunchContext()
    result.launch_configurations.update(values)
    return result


class SlamOwnershipTests(unittest.TestCase):
    def test_full_launch_static_context_uses_same_map_source_resolution(self):
        graph = description('astribot_s1_navigation', 'nav2_full_bringup.launch.py')
        setup = next(a for a in graph.entities if isinstance(a, OpaqueFunction) and
                     a._OpaqueFunction__function.__name__ == '_static_map_context')
        ctx = context(env='sim', slam_backend='static_map', launch_navigation='true',
                      map_source_file='/fixture/config.yaml', map_yaml_path='')
        with patch('astribot_s1_perception.map_source_config.resolve', return_value=(
                '/fixture/config.yaml', {'map_yaml_path': '/fixture/map.yaml'}, 'real_file', 'ground_truth')) as resolve:
            for action in setup.execute(ctx):
                action.execute(ctx)
            self.assertEqual(ctx.launch_configurations['simulation_static_map_yaml'], '/fixture/map.yaml')
            self.assertEqual(resolve.call_args.args[0], '/fixture/config.yaml')
        ctx.launch_configurations['env'] = 'hardware'
        for action in setup.execute(ctx):
            action.execute(ctx)
        self.assertEqual(ctx.launch_configurations['simulation_static_map_yaml'], '')

    def test_split_supervisor_explicitly_owns_cloud_producer_before_navigation(self):
        for mode in ('baseline', 'mapping', 'explore'):
            with self.subTest(mode=mode):
                result = subprocess.run([sys.executable, str(ROOT / 'tools/sim_stack_supervisor.py'),
                                         '--dry-run', '--mode', mode, '--instance', 'ownership_test',
                                         '--ros-domain-id', '71'], check=True, text=True, capture_output=True)
                commands = json.loads(result.stdout)
                self.assertIn('launch_navigation:=false', commands['simulation'])
                self.assertIn('launch_slam:=true', commands['simulation'])
                self.assertTrue(any(s.startswith('operator_runtime_params_file:=')
                                    for s in commands['navigation']))
                static_context = [s for s in commands['navigation']
                                  if s.startswith('simulation_static_map_yaml:=')]
                self.assertEqual(bool(static_context), mode == 'baseline')
                mapping_session = commands.get('mapping_session')
                self.assertEqual(bool(mapping_session), mode == 'mapping')
                if mapping_session:
                    self.assertEqual(mapping_session[:4], ['ros2', 'run', 'astribot_s1_exploration', 'mapping_session_node'])
                    self.assertIn('use_sim_time:=true', mapping_session)
                    self.assertIn('require_navigation_zones:=true', mapping_session)

    def test_localization_does_not_create_a_new_mapping_context(self):
        with tempfile.TemporaryDirectory() as folder:
            (Path(folder) / 'alidarState.txt').write_text('fixture for dry-run argument validation')
            result = subprocess.run([sys.executable, str(ROOT / 'tools/sim_stack_supervisor.py'),
                                     '--dry-run', '--mode', 'localize', '--map', folder,
                                     '--instance', 'ownership_test', '--ros-domain-id', '71'],
                                    check=True, text=True, capture_output=True)
            commands = json.loads(result.stdout)
            self.assertIsNone(commands['mapping_session'])
            self.assertIn('mode:=localization', commands['simulation'])

    def test_measured_registration_is_forwarded_and_external_slam_owner_is_explicit(self):
        pose = '.25,.15,.1292,0,0,0,1'
        result = subprocess.run([sys.executable, str(ROOT / 'tools/sim_stack_supervisor.py'),
                                 '--dry-run', '--mode', 'baseline', '--instance', 'ownership_test',
                                 '--ros-domain-id', '71', '--launch-slam', 'false',
                                 '--initial-chassis-pose', pose],
                                check=True, text=True, capture_output=True)
        commands = json.loads(result.stdout)
        self.assertIn('launch_slam:=false', commands['simulation'])
        self.assertIn('initial_chassis_pose:=' + pose, commands['simulation'])
        launch = description('astribot_s1_navigation', 'nav2_full_bringup.launch.py')
        perception = next(a for a in launch.entities if isinstance(a, IncludeLaunchDescription)
                          and 'initial_chassis_pose' in dict(a.launch_arguments))
        forwarded = dict(perception.launch_arguments)['initial_chassis_pose']
        self.assertEqual(perform_substitutions(context(initial_chassis_pose=pose),
                         normalize_to_list_of_substitutions(forwarded)),pose)

    def test_default_owner_distinguishes_fixed_map_and_managed_mapping(self):
        launch = description('astribot_s1_navigation', 'nav2_full_bringup.launch.py')
        argument = next(a for a in launch.entities
                        if isinstance(a, DeclareLaunchArgument) and a.name == 'launch_slam')
        for env, backend, mode, expected in (
            ('sim', 'static_map', 'mapping', 'true'),
            ('sim', 'voxel', 'mapping', 'false'),
            ('hardware', 'voxel', 'mapping', 'false'),
            ('sim', 'voxel', 'localization', 'true'),
            ('hardware', 'voxel', 'localization', 'true'),
        ):
            with self.subTest(env=env, backend=backend, mode=mode):
                ctx = context(env=env, slam_backend=backend, mode=mode)
                self.assertEqual(perform_substitutions(ctx, argument.default_value), expected)

    def test_cloud_producer_respects_explicit_owner_and_map_tf_ownership(self):
        launch = description('astribot_s1_perception', 'perception_slam_bringup.launch.py')
        for env in ('sim', 'hardware'):
            for backend in ('static_map', 'voxel'):
                for enabled in ('true', 'false'):
                    if env == 'hardware' and backend == 'static_map':
                        continue  # Rejected independently by launch validation.
                    with self.subTest(env=env, backend=backend, enabled=enabled):
                        ctx = context(env=env, slam_backend=backend, launch_slam=enabled)
                        producers = []
                        for action in launch.entities:
                            if not isinstance(action, IncludeLaunchDescription):
                                continue
                            arguments = dict(action.launch_arguments)
                            if 'point_notime' in arguments and action.condition.evaluate(ctx):
                                producers.append(action)
                        self.assertEqual(len(producers), int(enabled == 'true'))
                        if env == 'sim' and producers:
                            arguments = dict(producers[0].launch_arguments)
                            for key in ('publish_grid', 'publish_map_odom'):
                                self.assertEqual(perform_substitutions(
                                    ctx, normalize_to_list_of_substitutions(arguments[key])),
                                                 str(backend == 'voxel'))


if __name__ == '__main__':
    unittest.main()
