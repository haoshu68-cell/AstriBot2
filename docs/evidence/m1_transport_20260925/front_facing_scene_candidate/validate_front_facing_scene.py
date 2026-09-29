#!/usr/bin/env python3
"""Offline fixture geometry/limited IK checks. Never initializes or commands ROS."""
import ast
import hashlib
import json
import math
from pathlib import Path
import textwrap
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml


EVIDENCE = Path(__file__).resolve().parent
MANIFEST = json.loads((EVIDENCE / 'manifest.json').read_text())
CANDIDATE = Path(MANIFEST['candidate_directory'])
SOURCE = Path(MANIFEST['source_scene'])
config = json.loads((CANDIDATE / 'scenario.json').read_text())
initial = yaml.safe_load((CANDIDATE / 'sim_initial_transport_ready.yaml').read_text())['initial_positions']
model_path = SOURCE / 'fixtures/robot_state_publisher.urdf'
model = ET.parse(model_path).getroot()
parents = {joint.find('child').get('link'): joint for joint in model.findall('joint')}
report = {'scope': 'offline coordinate/kinematic fixture only; no collision, planning, ROS, Gazebo or hardware acceptance',
          'source_urdf': str(model_path), 'source_urdf_sha256': hashlib.sha256(model_path.read_bytes()).hexdigest(),
          'candidate': str(CANDIDATE), 'passed': False}


def chain_to(link):
    result = []
    while link != config['base_frame']:
        joint = parents[link]
        origin = joint.find('origin')
        transform = np.eye(4)
        transform[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        transform[:3, :3] = Rotation.from_euler('xyz', [float(v) for v in origin.get('rpy', '0 0 0').split()]).as_matrix()
        axis = joint.find('axis')
        result.append((joint.get('name'), joint.get('type'), transform,
                       None if axis is None else np.array([float(v) for v in axis.get('xyz').split()])))
        link = joint.find('parent').get('link')
    return list(reversed(result))


def fk(chain, values):
    transform = np.eye(4)
    for name, kind, origin, axis in chain:
        transform = transform @ origin
        if kind != 'fixed':
            motion = np.eye(4)
            motion[:3, :3] = Rotation.from_rotvec(axis * values[name]).as_matrix()
            transform = transform @ motion
    return transform


try:
    assert config['idle_position_hold'] is True and config['idle_position_kp'] == 3.0
    assert [initial['astribot_head_joint_1'], initial['astribot_head_joint_2']] == config['head_pick_joints']
    for name in ('astribot_torso_joint_1', 'astribot_torso_joint_2', 'astribot_torso_joint_3', 'astribot_torso_joint_4'):
        assert initial[name] == 0.0
    for path in CANDIDATE.glob('*.py'):
        ast.parse(path.read_text())
    for name, source in MANIFEST['source_files'].items():
        assert hashlib.sha256(Path(source['path']).read_bytes()).hexdigest() == source['sha256']
    scene_ast = ast.parse((CANDIDATE / 'prepare_scene.py').read_text())
    items = next(ast.literal_eval(node.value) for node in ast.walk(scene_ast)
                 if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'items' for t in node.targets))
    physical = {name: (np.array(xyz), np.array(size)) for name, xyz, size in items}
    report['stations'] = []
    for kind, dock in [('pick', config['spawn_world_xyyaw']), ('place', config['nav_goal_world'])]:
        center, size = physical[config['object_id'] + '_' + kind + '_station']
        front = np.array([math.cos(dock[2]), math.sin(dock[2])])
        left = np.array([-front[1], front[0]])
        offset = center[:2] - dock[:2]
        outward = np.array(config['station_access_normals_world'][kind + '_station'])[:2]
        assert abs(front @ offset - .55) < 1e-12 and abs(left @ offset) < 1e-12
        assert abs(front @ outward + 1.) < 1e-12
        assert np.allclose(center[:2], config[kind + '_xyz'][:2], atol=1e-12)
        assert abs(center[2] + size[2] / 2 + config['size_xyz'][2] / 2 - config[kind + '_xyz'][2]) < 1e-12
        report['stations'].append({'kind': kind, 'dock_world_xyyaw': dock, 'station_center_world': center.tolist(),
                                   'front_target_m': float(front @ offset), 'lateral_target_m': float(left @ offset),
                                   'front_face_distance_m': float(front @ offset - size[1] / 2),
                                   'nominal_chassis_gap_m': float(front @ offset - size[1] / 2 - .31),
                                   'access_normal_world': outward.tolist()})
    # Execute the candidate's own map-goal math for identity and a spawn-centred,
    # rotated map; the runtime will obtain this registration from same-stamp TF.
    source = (CANDIDATE / 'prepare_transfer_goal.py').read_text()
    begin = source.index('    world_nav=np.eye(4);')
    end = source.index('    goal.navigation_target=', begin)
    goal_math = textwrap.dedent(source[begin:end])
    registration = np.eye(4)
    registration[:3, :3] = Rotation.from_euler('z', -math.pi / 2).as_matrix()
    registration[:2, 3] = [-.15, .10]
    report['map_registrations'] = []
    for transform in [np.eye(4), registration]:
        scope = {'np': np, 'Rotation': Rotation, 'config': config, 'map_from_world': transform}
        exec(compile(goal_math, str(CANDIDATE / 'prepare_transfer_goal.py'), 'exec'), scope)
        nav, world_nav = scope['nav'], scope['world_nav']
        assert np.allclose(np.linalg.inv(transform) @ nav, world_nav, atol=1e-12)
        report['map_registrations'].append({'map_from_world': transform.tolist(), 'map_navigation_pose': nav.tolist(),
                                          'world_roundtrip_max_error': float(np.max(np.abs(np.linalg.inv(transform) @ nav - world_nav)))})
    preflight = json.loads((SOURCE / 'transfer_goal.preflight.json').read_text())
    robot = next(p for p in preflight['world_truth']['pose'] if p.get('name') == 'astribot_s1')
    base_height = robot['position']['z']
    report['height_reference'] = {'world_base_z_m': base_height, 'source': 'scene39 same-stamp world pose; new scene must remeasure'}
    arm_names = [f'astribot_arm_left_joint_{i}' for i in range(1, 8)]
    joints = {joint.get('name'): joint for joint in model.findall('joint')}
    lower = np.array([float(joints[n].find('limit').get('lower')) for n in arm_names])
    upper = np.array([float(joints[n].find('limit').get('upper')) for n in arm_names])
    seed = np.array([initial[n] for n in arm_names])
    arm_chain = chain_to(config['tcp'])
    orientation = Rotation.from_euler('z', -math.pi / 2).as_matrix() @ Rotation.from_quat(config['orientation_xyzw']).as_matrix()
    report['kinematics'] = []
    for name, distance, lift, margin in [('rejected_070_grasp', .70, 0., 0.),
                                         ('candidate_pregrasp', .55, config['approach_m'], .1),
                                         ('candidate_grasp', .55, 0., .1),
                                         ('candidate_lift', .55, config['pick_lift_m'], .1)]:
        target = np.array([distance, 0., config['pick_xyz'][2] + config['grasp_offset_m'] + lift - base_height])

        def residual(q):
            transform = fk(arm_chain, dict(initial, **dict(zip(arm_names, q))))
            return np.r_[transform[:3, 3] - target, Rotation.from_matrix(orientation.T @ transform[:3, :3]).as_rotvec()]

        solution = least_squares(residual, seed, bounds=(lower + margin, upper - margin), max_nfev=500,
                                 xtol=1e-12, gtol=1e-12, ftol=1e-12)
        error = residual(solution.x)
        row = {'name': name, 'target_in_base': target.tolist(), 'position_error_m': float(np.linalg.norm(error[:3])),
               'orientation_error_rad': float(np.linalg.norm(error[3:])), 'joint_values': dict(zip(arm_names, solution.x.tolist())),
               'required_limit_margin_rad': margin, 'minimum_actual_limit_margin_rad': float(min(np.min(solution.x - lower), np.min(upper - solution.x))),
               'scope': 'single-seed numerical IK only; no collision, path or measured execution proof'}
        report['kinematics'].append(row)
        if name.startswith('candidate'):
            assert row['position_error_m'] < 1e-5 and row['orientation_error_rad'] < 1e-4
            assert row['minimum_actual_limit_margin_rad'] >= .1 - 1e-12
    head = fk(chain_to('head_rgbd_camera_optical_frame'), initial)
    point = np.array([.55, 0., config['pick_xyz'][2] - base_height])
    optical = head[:3, :3].T @ (point - head[:3, 3])
    assert optical[2] > 0.
    report['head'] = {'head_pick_joints': config['head_pick_joints'], 'camera_origin_base': head[:3, 3].tolist(),
                      'box_center_optical': optical.tolist(), 'axis_error_rad': float(math.atan2(np.linalg.norm(optical[:2]), optical[2])),
                      'nominal_center_pixel': [float(374.5375 * optical[0] / optical[2] + 318.252),
                                               float(374.451 * optical[1] / optical[2] + 179.303)],
                      'scope': 'unchanged URDF optical geometry; image visibility/occlusion not tested'}
    report['passed'] = True
finally:
    (EVIDENCE / 'validation.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
print(json.dumps({'passed': report['passed'], 'output': str(EVIDENCE / 'validation.json')}, indent=2))
