"""Versioned, ROS-independent policy boundary. All outputs are proposals, never commands."""
import copy
import hashlib
import json
import math
from urllib.parse import urlparse

from .core import TaskFailure

VERSION = 'astribot.vla/1'
ACTION_TYPES = ('mtc_targets', 'ee_delta_chunk', 'joint_position_chunk')
UNITS = dict(position='m', rotation='rad', quaternion='xyzw', time='s', gripper='width_m')
IDENTITY = ('schema', 'episode_id', 'request_id', 'sequence', 'context_id')


def fail(reason):
    raise TaskFailure('VLA_' + reason)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def numbers(value, length):
    return (isinstance(value, list) and len(value) == length and
            all(type(x) in (int, float) and math.isfinite(x) for x in value))


def validate_config(config):
    c = copy.deepcopy(config)
    if c.get('mode') not in ('shadow', 'mtc') or c.get('adapter') not in ('reference', 'http'):
        fail('CONFIG_MODE_OR_ADAPTER')
    for key, default, upper in [('timeout_s', 3., 10.), ('max_snapshot_age_s', 4., 12.),
                              ('max_target_delta_m', .015, .02), ('max_rotation_delta_rad', .1, .2)]:
        c.setdefault(key, default)
        if type(c[key]) not in (float, int) or not math.isfinite(c[key]) or not 0 < c[key] <= upper:
            fail('CONFIG_' + key.upper())
    cameras = c.get('cameras')
    if not isinstance(cameras, list) or not 1 <= len(cameras) <= 4:
        fail('CONFIG_CAMERAS')
    ids = set()
    for camera in cameras:
        if not isinstance(camera, dict) or not all(isinstance(camera.get(k), str) and camera[k]
                for k in ('id', 'rgb_topic', 'info_topic')) or camera['id'] in ids:
            fail('CONFIG_CAMERA')
        ids.add(camera['id'])
        if 'depth_topic' in camera and not isinstance(camera['depth_topic'], str):
            fail('CONFIG_DEPTH_TOPIC')
    if c['adapter'] == 'http':
        if not isinstance(c.get('endpoint'), str):
            fail('CONFIG_ENDPOINT')
        try:
            url = urlparse(c['endpoint'])
            if (url.scheme not in ('http', 'https') or not url.hostname or url.username or url.password or
                    url.query or url.fragment or (url.port is not None and not 0 < url.port <= 65535)):
                fail('CONFIG_ENDPOINT')
        except ValueError:
            fail('CONFIG_ENDPOINT')
    return c


def validate_capabilities(capabilities, mode):
    if not isinstance(capabilities, dict) or capabilities.get('schema') != VERSION:
        fail('CAPABILITIES_VERSION')
    if capabilities.get('robot_model') != 'astribot_s1' or capabilities.get('units') != UNITS:
        fail('CAPABILITIES_EMBODIMENT_OR_UNITS')
    for key in ('policy_id', 'model_revision', 'normalization_id'):
        if not isinstance(capabilities.get(key), str) or not capabilities[key]:
            fail('CAPABILITIES_' + key.upper())
    types = capabilities.get('action_types')
    if not isinstance(types, list) or not types or any(t not in ACTION_TYPES for t in types):
        fail('CAPABILITIES_ACTION_TYPES')
    if mode == 'mtc' and 'mtc_targets' not in types:
        fail('EXECUTABLE_ACTION_NOT_SUPPORTED')


def validate_proposal(request, response, capabilities, config):
    if not isinstance(response, dict) or any(type(response.get(k)) is not type(request[k]) or
                                           response.get(k) != request[k] for k in IDENTITY):
        fail('RESPONSE_CONTEXT')
    if response.get('units') != UNITS:
        fail('RESPONSE_UNITS')
    action = response.get('action')
    if not isinstance(action, dict) or action.get('type') not in capabilities['action_types']:
        fail('ACTION_TYPE')
    if action.get('frame_id') != request['frame_id'] or action.get('group') != 'arm_left':
        fail('ACTION_FRAME_OR_GROUP')
    kind = action['type']
    if kind != 'mtc_targets' and config['mode'] != 'shadow':
        fail('ACTION_REQUIRES_SHADOW')
    if kind == 'mtc_targets':
        if set(action) != {'type', 'frame_id', 'group', 'pre_target', 'target', 'exit_targets'}:
            fail('TARGET_FIELDS')
        default = request['nominal_action']
        exits = action.get('exit_targets')
        if not isinstance(exits, list) or len(exits) != len(default['exit_targets']):
            fail('EXIT_TARGET_COUNT')
        for actual, expected in zip([action['pre_target'], action['target']] + exits,
                                    [default['pre_target'], default['target']] + default['exit_targets']):
            if (not isinstance(actual, dict) or set(actual) != {'position', 'quaternion'} or
                    not numbers(actual['position'], 3) or not numbers(actual['quaternion'], 4)):
                fail('TARGET_FORMAT')
            q = actual['quaternion']
            if abs(sum(x*x for x in q) - 1.) > 1e-5:
                fail('QUATERNION_NOT_NORMALIZED')
            if math.dist(actual['position'], expected['position']) > config['max_target_delta_m']:
                fail('TARGET_OUTSIDE_TASK_REGION')
            cosine = min(1., abs(sum(a*b for a,b in zip(q, expected['quaternion']))))
            if 2*math.acos(cosine) > config['max_rotation_delta_rad']:
                fail('ROTATION_OUTSIDE_TASK_REGION')
    else:
        # Wire contracts for policy benchmarking. There is deliberately no controller
        # consumer for these chunks until robot-specific mapping and validation exist.
        expected_fields = {'type', 'frame_id', 'group', 'samples'}
        if kind == 'joint_position_chunk':
            expected_fields.add('joint_names')
            names = action.get('joint_names')
            expected_names = request['observation']['arm_joint_names']
            if names != expected_names or len(names) != 7:
                fail('JOINT_ORDER')
        if set(action) != expected_fields:
            fail('CHUNK_FIELDS')
        samples = action.get('samples')
        if not isinstance(samples, list) or not 1 <= len(samples) <= 32:
            fail('CHUNK_LENGTH')
        last = 0.
        for sample in samples:
            fields = ({'time_s', 'position_delta_m', 'rotation_vector_rad', 'gripper_width_m'}
                      if kind == 'ee_delta_chunk' else {'time_s', 'positions_rad', 'gripper_width_m'})
            if not isinstance(sample, dict) or set(sample) != fields:
                fail('CHUNK_SAMPLE_FIELDS')
            t, width = sample['time_s'], sample['gripper_width_m']
            if (type(t) not in (int, float) or not math.isfinite(t) or not last < t <= 2. or
                    type(width) not in (int, float) or not math.isfinite(width) or not 0 <= width <= .1):
                fail('CHUNK_TIME_OR_GRIPPER')
            last = t
            if kind == 'ee_delta_chunk':
                if (not numbers(sample['position_delta_m'], 3) or not numbers(sample['rotation_vector_rad'], 3) or
                        math.dist(sample['position_delta_m'], [0.]*3) > .02 or
                        math.dist(sample['rotation_vector_rad'], [0.]*3) > .2):
                    fail('DELTA_LIMIT')
            elif not numbers(sample['positions_rad'], 7):
                fail('JOINT_FORMAT')
    return copy.deepcopy(action)
