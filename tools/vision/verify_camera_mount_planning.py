#!/usr/bin/env python3
"""Offline launch/semantic checks; optionally export models for the C++ FCL probe."""
import argparse
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

ROOT = Path(__file__).resolve().parents[2]
PATH = ROOT/'ws_robot/src/astribot_s1_moveit_config/launch/move_group.launch.py'


def resolve(module, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    description = module.generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    for action in module._prepare_robot_descriptions(context):
        action.execute(context)
    return context.launch_configurations


def pairs(xml):
    return {frozenset((x.attrib['link1'], x.attrib['link2']))
            for x in ET.fromstring(xml).findall('disable_collisions')}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path)
    parser.add_argument('--use-lidar', choices=['true', 'false'], default='true')
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('mount_planning', PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original_prefix = module.get_package_prefix
    def absent_package(_name):
        raise module.PackageNotFoundError('deliberately missing test plugin')
    try:
        for lookup in (absent_package, lambda _name: '/nonexistent/astribot_test_sensor_plugin'):
            module.get_package_prefix = lookup
            try:
                module._prepare_robot_descriptions(None)
            except RuntimeError as error:
                assert 'Missing' in str(error)
            else:
                raise AssertionError('missing camera obstacle plugin must block startup')
    finally:
        module.get_package_prefix = original_prefix
    print('PASS missing package and missing plugin library block startup')
    original = (ROOT/'ws_robot/src/astribot_s1_moveit_config/config/astribot_s1.srdf').read_text()
    sim = resolve(module, use_lidar=args.use_lidar)
    assert sim['camera_mounts_profile'].endswith('camera_mounts_reference_sim.yaml')
    expected = {frozenset((c+'_camera_link', 'astribot_head_link_2'))
                for c in ('head_rgbd', 'head_stereo_left', 'head_stereo_right')}
    expected.add(frozenset(('torso_rgbd_camera_link', 'astribot_torso_link_4')))
    expected.update(frozenset((s+'_wrist_rgbd_camera_link', 'astribot_gripper_'+s+'_base'))
                    for s in ('left', 'right'))
    assert pairs(sim['resolved_robot_semantic'])-pairs(original) == expected
    assert not pairs(original)-pairs(sim['resolved_robot_semantic'])
    print('PASS reference adds exactly six rigid mount contacts, preserves every existing pair')
    for overrides in ({'use_sim_time': 'false'}, {'camera_mounts_profile': ''}):
        resolved = resolve(module, **overrides)
        assert resolved['camera_mounts_profile'] == ''
        assert resolved['resolved_robot_semantic'] == original
        print('PASS original/hardware semantic bytes unchanged:', overrides)
    # A candidate attachment separated by a revolute joint must never receive
    # the fixed-mount exception, even if its names match the reference profile.
    urdf = ET.fromstring(sim['resolved_robot_description'])
    joint = next(j for j in urdf.findall('joint')
                 if j.find('child').attrib['link'] == 'torso_rgbd_camera_link')
    joint.set('type', 'revolute')
    changed = module._fixed_camera_mount_semantic(ET.tostring(urdf, encoding='unicode'), original)
    assert frozenset(('torso_rgbd_camera_link', 'astribot_torso_link_4')) not in pairs(changed)
    print('PASS movable attachment receives no fixed-contact exemption')
    for flag, absent in (
        ('use_lidar', ('livox_mid360_left', 'livox_mid360_right')),
        ('use_wrist_cameras', ('left_wrist_rgbd_camera_link', 'right_wrist_rgbd_camera_link')),
        ('use_stereo_cameras', ('head_stereo_left_camera_link', 'head_stereo_right_camera_link')),
        ('use_camera', ('head_rgbd_camera_link', 'torso_rgbd_camera_link')),
    ):
        resolved = resolve(module, **{flag: 'false'})
        links = {e.get('name') for e in ET.fromstring(resolved['resolved_robot_description']).findall('link')}
        assert not links.intersection(absent), (flag, absent)
        print('PASS planner attachment selection:', flag)
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output/'reference.urdf').write_text(sim['resolved_robot_description'])
        (args.output/'reference.srdf').write_text(sim['resolved_robot_semantic'])


if __name__ == '__main__':
    main()
