"""Gazebo follower configuration must agree with the kinematic URDF contract."""
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import xacro


@pytest.mark.parametrize('enabled', [True, False])
def test_simulation_mimics_are_registered_without_independent_commands(enabled):
    model = Path(__file__).resolve().parents[1] / 'urdf/astribot_s1.xacro'
    root = ET.fromstring(xacro.process_file(str(model), mappings={
        'robot_name': 'astribot_s1', 'use_gripper': str(enabled).lower(), 'use_camera': 'false',
        'use_lidar': 'false'}).toxml())
    hardware = {j.attrib['name']: j for j in root.find('ros2_control').findall('joint')}
    followers = [j for j in root.findall('joint')
                 if 'gripper_' in j.attrib['name'] and j.find('mimic') is not None]
    assert len(followers) == (10 if enabled else 0)
    for joint in followers:
        mimic = joint.find('mimic'); interface = hardware[joint.attrib['name']]
        params = {p.attrib['name']: p.text for p in interface.findall('param')}
        assert params['mimic'] == mimic.attrib['joint']
        assert float(params['multiplier']) == float(mimic.attrib['multiplier'])
        assert float(mimic.get('offset', '0')) == 0
        assert not interface.findall('command_interface')
        assert {s.attrib['name'] for s in interface.findall('state_interface')} == {'position', 'velocity'}
        master = hardware[params['mimic']]
        assert [c.attrib['name'] for c in master.findall('command_interface')] == ['position']
    if not enabled:
        assert not any('gripper_' in name for name in hardware)
