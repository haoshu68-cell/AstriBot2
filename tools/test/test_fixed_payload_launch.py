"""Actual launch parameter resolution without starting a ROS graph."""
import importlib.util
from pathlib import Path
import unittest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


class FixedPayloadLaunchTest(unittest.TestCase):
    def test_coordinator_receives_the_same_explicit_payload_identity(self):
        path=Path(__file__).resolve().parents[2]/'ws_robot/src/astribot_s1_navigation/launch/navigation.launch.py'
        spec=importlib.util.spec_from_file_location('fixed_payload_launch',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        description=module.generate_launch_description()
        context=LaunchContext()
        expected={'payload_environment':'simulation','payload_session_id':'owned_test','payload_source_id':'explicit_physical_inventory'}
        context.launch_configurations.update(expected)
        for item in description.entities:
            if isinstance(item,DeclareLaunchArgument):item.execute(context)
        nodes=[item for item in description.entities if isinstance(item,Node) and item.node_executable=='fixed_envelope_cpp']
        self.assertEqual(len(nodes),1)
        parameters=evaluate_parameters(context,nodes[0]._Node__parameters)
        actual={key:value for values in parameters if isinstance(values,dict) for key,value in values.items()}
        for key,value in expected.items():self.assertEqual(actual[key],value)


if __name__=='__main__':unittest.main()
