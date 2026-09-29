#!/usr/bin/env python3
"""Exercise real launch scoping with process actions replaced by observers."""
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction

ROOT=Path(__file__).resolve().parents[2]

class WristTransport(unittest.TestCase):
    def check_transport(self, explicit):
        path=ROOT/'ws_robot/src/astribot_s1_perception_components/launch/wrist_camera_session.launch.py'
        spec=importlib.util.spec_from_file_location('wrist_launch',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        observed={}
        with tempfile.NamedTemporaryFile(suffix='.xml') as profile:
            def capture(label, additional=None):
                def observe(context):
                    env=dict(context.environment);env.update(additional or {})
                    observed[label]={key:env.get(key) for key in ('FASTRTPS_DEFAULT_PROFILES_FILE','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION')}
                    return []
                return OpaqueFunction(function=observe)
            module.Node=lambda **kwargs:capture('session',kwargs.get('additional_env'))
            module.IncludeLaunchDescription=lambda *args,**kwargs:capture('pipeline')
            from launch.actions import SetLaunchConfiguration
            ld=LaunchDescription([SetLaunchConfiguration('dds_profile',profile.name if explicit else ''),capture('before'),*module.generate_launch_description().entities,capture('after')])
            service=LaunchService();service.include_launch_description(ld)
            self.assertEqual(service.run(),0)
            expected=dict(observed['before'])
            if explicit:expected.update(FASTRTPS_DEFAULT_PROFILES_FILE=profile.name,ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
            self.assertEqual(observed['session'],expected)
            self.assertEqual(observed['pipeline'],expected)
            self.assertEqual(observed['after'],observed['before'])
    def test_explicit_profile_reaches_both_nodes_without_leaking(self):self.check_transport(True)
    def test_empty_profile_preserves_inherited_network(self):self.check_transport(False)

if __name__=='__main__':unittest.main()
