import unittest
from launch import LaunchContext
from launch.actions import GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from launch_ros.utilities import evaluate_parameters
from tools.test.test_slam_launch_ownership import description


class CameraProfileContractTests(unittest.TestCase):
    @staticmethod
    def flatten(entities):
        for action in entities:
            yield action
            if isinstance(action, GroupAction):
                yield from CameraProfileContractTests.flatten(action.get_sub_entities())

    def test_rendering_and_postprocess_share_selected_camera_profiles(self):
        graph = description('astribot_s1_gazebo_bringup', 'warehouse_sim.launch.py')
        context = LaunchContext()
        context.launch_configurations.update(use_sim_time='true', camera_calibration_dir='/fallback',
                                            camera_profile='/selected/head.yaml',
                                            torso_camera_profile='/selected/torso.yaml')
        nodes = [a for a in self.flatten(graph.entities) if hasattr(a, 'node_executable') and
                 perform_substitutions(context, normalize_to_list_of_substitutions(a.node_executable)) == 'camera_calibration_postprocess']
        parameters = [evaluate_parameters(context, a._Node__parameters)[0] for a in nodes]
        self.assertEqual(len(parameters), 2)
        self.assertEqual({p['profile'] for p in parameters}, {'/selected/head.yaml', '/selected/torso.yaml'})
        self.assertEqual({p['output_frame'] for p in parameters},
                         {'head_rgbd_camera_optical_frame', 'torso_rgbd_camera_optical_frame'})

    def test_camera_pipeline_inherits_bounded_local_transport_with_lan_opt_out(self):
        graph = description('astribot_s1_gazebo_bringup', 'warehouse_sim.launch.py')
        context = LaunchContext()
        groups = [a for a in graph.entities if isinstance(a, GroupAction) and any(
            hasattr(n, 'node_executable') and perform_substitutions(
                context, normalize_to_list_of_substitutions(n.node_executable)) == 'camera_calibration_postprocess'
            for n in a.get_sub_entities())]
        self.assertEqual(len(groups), 1)
        setup = next(a for a in groups[0].get_sub_entities() if isinstance(a, OpaqueFunction))
        for local, explicit, expected in [('true', '', 3), ('false', '', 0),
                                           ('false', '/explicit/profile.xml', 3)]:
            context.launch_configurations.update(localhost_only=local, camera_bridge_dds_profile=explicit)
            actions = setup.execute(context)
            self.assertEqual(len(actions), expected)
            self.assertTrue(all(isinstance(a, SetEnvironmentVariable) for a in actions))


if __name__ == '__main__':
    unittest.main()
