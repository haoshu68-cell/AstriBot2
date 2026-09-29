#!/usr/bin/env python3
"""Analytic sphere-box check of the actual scene41 collision pair; no ROS."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
FIXTURE = Path('/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/front_facing_transfer_candidate_20260925/front_transfer_scene41_world54/fixtures')
report = json.loads((FIXTURE / 'result.json').read_text())
model = ET.parse(FIXTURE / 'robot_state_publisher.urdf').getroot()
parents = {joint.find('child').get('link'): joint for joint in model.findall('joint')}
measured = report['initial_ready']['measurement']['joints']
values = dict(zip(measured['name'], measured['position']))


def origin_transform(origin):
    transform = np.eye(4)
    if origin is not None:
        transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    return transform


def fk(link, joints):
    chain = []
    while link != 'astribot_torso_base':
        joint = parents[link]
        chain.append(joint)
        link = joint.find('parent').get('link')
    result = np.eye(4)
    for joint in reversed(chain):
        result = result @ origin_transform(joint.find('origin'))
        if joint.get('type') != 'fixed':
            motion = np.eye(4)
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            value = joints[joint.get('name')]
            if joint.get('type') == 'prismatic':
                motion[:3, 3] = axis * value
            else:
                motion[:3, :3] = Rotation.from_rotvec(axis * value).as_matrix()
            result = result @ motion
    return result


torso = model.find("link[@name='astribot_torso_link_4']/collision")
head = model.find("link[@name='astribot_head_link_2']/collision")
half_size = np.fromstring(torso.find('geometry/box').get('size'), sep=' ') / 2
radius = float(head.find('geometry/sphere').get('radius'))
box_pose = fk('astribot_torso_link_4', values) @ origin_transform(torso.find('origin'))
head_origin = origin_transform(head.find('origin'))[:, 3]
validity = report['initial_ready']['full_scene_state_validity']
assert validity['valid'] is False and len(validity['contacts']) == 1
contact = validity['contacts'][0]
assert {contact['contact_body_1'], contact['contact_body_2']} == {'astribot_torso_link_4', 'astribot_head_link_2'}
result = {'scope': 'exact URDF sphere-box pair only; full-scene runtime revalidation required',
          'source_result': str(FIXTURE / 'result.json'), 'observed_contact': contact, 'rows': [],
          'source_sha256': {name: hashlib.sha256((FIXTURE / name).read_bytes()).hexdigest()
                            for name in ('result.json', 'robot_state_publisher.urdf')}}
for pitch in (.95, .65):
    joints = dict(values, astribot_head_joint_1=0., astribot_head_joint_2=pitch)
    sphere = fk('astribot_head_link_2', joints) @ head_origin
    relative = np.linalg.inv(box_pose) @ sphere
    clearance = float(np.linalg.norm(np.maximum(np.abs(relative[:3]) - half_size, 0.)) - radius)
    result['rows'].append({'head_yaw_rad': 0., 'head_pitch_rad': pitch, 'signed_clearance_m': clearance})
assert abs(result['rows'][0]['signed_clearance_m'] + contact['depth']) < 1e-8
assert result['rows'][1]['signed_clearance_m'] > 0.
result['passed'] = True
(HERE / 'validation.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({'passed': True, 'rows': result['rows']}))
