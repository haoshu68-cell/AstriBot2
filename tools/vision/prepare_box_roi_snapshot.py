#!/usr/bin/env python3
"""Offline join of an existing sim_pose_capture snapshot and exact-stamp truth."""
import argparse
import hashlib
import json
from pathlib import Path
import re

import numpy as np
from scipy.spatial.transform import Rotation

from score_box_roi import rigid_matrix, score_sample


def checked_pose(pose):
    xyz = np.asarray(pose['translation'], dtype=float)
    quaternion = np.asarray(pose['quaternion_xyzw'], dtype=float)
    if (xyz.shape != (3,) or quaternion.shape != (4,)
            or not np.isfinite(np.r_[xyz, quaternion]).all()
            or not np.isclose(np.linalg.norm(quaternion), 1., atol=1e-6, rtol=0)):
        raise ValueError('Invalid position or quaternion; no normalization of malformed truth')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    result[:3, 3] = xyz
    return result


def complete_truth_messages(raw, final=False):
    """Incremental text framing; the next header terminates a live message."""
    starts = [m.start() for m in re.finditer(r'(?m)^header\s*\{', raw)]
    messages = []
    if not starts:
        return messages, raw
    for index, start in enumerate(starts):
        block = raw[start:starts[index+1] if index+1 < len(starts) else len(raw)]
        if index == len(starts)-1 and not final:
            return messages, block
        if block.count('{') != block.count('}'):
            if index == len(starts)-1:
                return messages, block
            raise ValueError('Incomplete interior truth message')
        header = block.split('\npose', 1)[0]
        time_block = re.search(r'\bstamp\s*\{([^}]*)\}', header)
        if time_block is None:
            raise ValueError('Truth header has no source stamp')
        fields = dict(re.findall(r'\b(sec|nsec):\s*(\d+)', time_block.group(1)))
        ts = int(fields.get('sec', 0))*10**9 + int(fields.get('nsec', 0))
        messages.append((ts, block))
    return messages, ''


def truth_at_stamp(raw, stamp, names):
    messages, tail = complete_truth_messages(raw, final=True)
    matches, stamps = [], []
    for index, (ts, block) in enumerate(messages):
        if stamps and ts <= stamps[-1]:
            raise ValueError('Truth clock duplicate/regression; split and bind clock epochs explicitly')
        stamps.append(ts)
        if ts == stamp:
            matches.append((index, block))
    if len(matches) != 1:
        before = max((ts for ts in stamps if ts < stamp), default=None)
        after = min((ts for ts in stamps if ts > stamp), default=None)
        raise ValueError(f'No unique exact truth stamp {stamp}; before={before}, after={after}')
    index, selected = matches[0]
    found = {name: [] for name in names}
    for match in re.finditer(r'(?ms)^pose\s*\{.*?^\}', selected):
        block = match.group(0)
        name_match = re.search(r'\bname:\s*("(?:[^"\\]|\\.)*")', block)
        if name_match is None:
            continue
        name = json.loads(name_match.group(1))
        if name not in found:
            continue
        values = {}
        for key, components in [('position', 'xyz'), ('orientation', 'xyzw')]:
            part = re.search(r'\b'+key+r'\s*\{([^}]*)\}', block)
            if part is None:
                raise ValueError(f'Missing complete {key} block for {name}')
            fields = dict(re.findall(r'\b([xyzw]):\s*([^\s}]+)', part.group(1)))
            # Protobuf text omits numeric zeros, including quaternion w=0.
            values[key] = [float(fields.get(c, 0)) for c in components]
        pose = {'translation': values['position'], 'quaternion_xyzw': values['orientation']}
        entity_id = re.search(r'\bid:\s*(\d+)', block)
        pose['entity_id'] = int(entity_id.group(1)) if entity_id else 0
        checked_pose(pose)
        found[name].append(pose)
    if any(len(poses) != 1 for poses in found.values()):
        raise ValueError('Missing or duplicate exact-name model in selected truth message')
    return {name: poses[0] for name, poses in found.items()}, {
        'selected_message_index': index, 'selected_stamp_ns': stamp,
        'complete_truth_messages': len(stamps), 'truncated_tail_messages': int(bool(tail)),
        'selected_truth_text': selected}


def assemble_sample(metadata, raw_truth, context):
    required = ('session_id', 'task_id', 'phase', 'clock_epoch', 'world_frame', 'base_frame',
                'robot_model', 'object_model', 'object_id', 'model_binding_evidence',
                'object_geometry_evidence', 'clock_epoch_evidence', 'phase_evidence', 'session_json')
    if any(not isinstance(context[key], str) or not context[key] for key in required):
        raise ValueError('Fill all real-session context fields; unknown values must not be fabricated')
    if context['snapshot_exit_code'] != 0:
        raise ValueError('Original snapshot command must have succeeded')
    if context['robot_model'] == context['object_model']:
        raise ValueError('Robot and target must be distinct model entities')
    model_base = rigid_matrix(context['model_from_base'])
    stamp, frame = metadata['capture_stamp_ns'], metadata['frame']
    if metadata['exact_sync_stamps'] != dict(rgb=stamp, depth=stamp, info=stamp):
        raise ValueError('Snapshot is not exact RGB/depth/info synchronized')
    age = metadata['capture_age_at_end_sec']
    if age is None or not np.isfinite(age) or not 0 <= age <= .25:
        raise ValueError('Snapshot failed existing nonfuture 250ms freshness requirement')
    health = metadata['camera_health_at_capture']
    health_stamp = health['capture_stamp']['sec']*10**9 + health['capture_stamp']['nanosec']
    if (not health['valid'] or health_stamp != stamp or health['frame_id'] != frame
            or health['camera_id'] != metadata['camera_id']
            or health['source_epoch'] != metadata['source_epoch']
            or health['calibration_revision'] != metadata['calibration_revision']):
        raise ValueError('Snapshot health identity/calibration/capture time mismatch')
    base = metadata['base_from_camera']
    odom = metadata['odom_from_camera']
    if any(tf['stamp_ns'] != stamp or tf['child'] != frame for tf in (base, odom)):
        raise ValueError('TF was not evaluated at the exact image stamp/frame')
    if base['parent'] != context['base_frame'] or odom['parent'] != 'odom':
        raise ValueError('Unexpected TF reference frame')
    poses, evidence = truth_at_stamp(raw_truth, stamp, (context['robot_model'], context['object_model']))
    world_model = checked_pose(poses[context['robot_model']])
    world_camera = world_model @ model_base @ checked_pose(base)
    world_odom = world_camera @ np.linalg.inv(checked_pose(odom))
    sample = {key: context[key] for key in ('session_id', 'task_id', 'phase', 'clock_epoch', 'object_id')}
    sample.update(schema='astribot.m5.box_roi/1', evaluation_only=True,
        camera_id=metadata['camera_id'], source_epoch=metadata['source_epoch'],
        calibration_revision=metadata['calibration_revision'],
        camera={'width': metadata['width'], 'height': metadata['height'], 'frame_id': frame,
                'K': metadata['K'], 'D': metadata['D'], 'depth_convention': 'optical_z_m',
                'stamps_ns': metadata['exact_sync_stamps']},
        world_from_camera={'matrix': world_camera.tolist(), 'frame_id': context['world_frame'],
            'child_frame_id': frame, 'capture_stamp_ns': stamp,
            'clock_epoch': context['clock_epoch'], 'calibration_revision': metadata['calibration_revision']},
        box_truth={'matrix': checked_pose(poses[context['object_model']]).tolist(),
            'frame_id': context['world_frame'], 'object_id': context['object_id'],
            'capture_stamp_ns': stamp, 'clock_epoch': context['clock_epoch'],
            'size_m': context['size_m'], 'source': 'gazebo_model_pose'},
        depth_policy={'min_m': .08, 'max_m': 5.,
            'source': 'astribot_s1_transport/rgbd.py project_component validity predicate; not a coverage threshold'},
        surface_tolerance_m=None, surface_tolerance_source=None,
        context_provenance=context)
    evidence.update(actual_model_poses=poses, world_from_odom=world_odom.tolist(),
        tf_semantics='tf2 evaluation at image stamp, may interpolate original TF messages',
        coordinate_semantics='world-model times explicit model-base times capture-time base-camera; no world=odom assumption',
        clock_epoch_semantics='operator-owned session/clock evidence; camera source_epoch is not a clock epoch',
        motion_acceptance='NOT_EVALUATED')
    return sample, evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True, help='Existing sim_pose_capture directory')
    parser.add_argument('--truth', type=Path, required=True, help='Independent Pose_V text recording')
    parser.add_argument('--context', type=Path, required=True, help='Filled operator context JSON')
    parser.add_argument('--output', type=Path, required=True, help='New output directory')
    args = parser.parse_args()
    metadata_path = args.capture/'camera_info.json'
    metadata = json.loads(metadata_path.read_text())
    context = json.loads(args.context.read_text())
    sample, evidence = assemble_sample(metadata, args.truth.read_text(), context)
    paths = {'camera_info': metadata_path, 'truth_stream': args.truth, 'operator_context': args.context,
             'rgb': args.capture/'rgb.png', 'depth': args.capture/'depth.npy'}
    for key in ('model_binding_evidence', 'object_geometry_evidence', 'clock_epoch_evidence',
                'phase_evidence', 'session_json'):
        paths[key] = args.context.parent/context[key]
    hashes = {name: {'path': str(path.resolve()), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
              for name, path in paths.items()}
    sample['files'] = {name: str(paths[name].resolve()) for name in ('rgb', 'depth')}
    sample['source_sha256'] = hashes
    # Validate the completed input with the same scorer; do not imply coverage PASS.
    report, _ = score_sample(sample, np.load(paths['depth'], allow_pickle=False))
    args.output.mkdir(parents=True, exist_ok=False)
    selected = evidence.pop('selected_truth_text')
    (args.output/'sample.json').write_text(json.dumps(sample, indent=2, allow_nan=False)+'\n')
    (args.output/'assembly.json').write_text(json.dumps(evidence, indent=2, allow_nan=False)+'\n')
    (args.output/'truth_at_capture.pbtxt').write_text(selected)
    print(json.dumps({'capture_stamp_ns': sample['camera']['stamps_ns']['depth'],
                      'geometry_status': report['geometry_status'],
                      'coverage_acceptance': report['coverage_acceptance']}))


if __name__ == '__main__':
    main()
