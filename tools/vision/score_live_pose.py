#!/usr/bin/env python3
"""Independently score live Pose Actions; truth never enters the Action client.

--run-live records Gazebo transport in a separate read-only process while the
RGB-D-only client runs. Offline scoring requires capture-time TF and bracketing
truth samples. This fixture is static; any measured target/robot motion refuses
scoring instead of silently treating a later transform as capture-time truth.
"""
import argparse
import bisect
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

from sim_pose_truth import named_pose


TRANSLATION_LIMIT_M = .020
ROTATION_LIMIT_DEG = 10.0
BASE_FRAME = 'astribot_torso_base'


def matrix(pose):
    xyz = np.asarray(pose['translation'], dtype=float)
    quat = np.asarray(pose['quaternion_xyzw'], dtype=float)
    if xyz.shape != (3,) or quat.shape != (4,) or not np.isfinite(np.r_[xyz, quat]).all():
        raise ValueError('Nonfinite or malformed transform')
    if abs(np.linalg.norm(quat) - 1) > .001:
        raise ValueError('Invalid quaternion norm')
    value = np.eye(4)
    value[:3, :3] = Rotation.from_quat(quat).as_matrix()
    value[:3, 3] = xyz
    return value


def error_between(first, second):
    return (float(np.linalg.norm(first[:3, 3] - second[:3, 3])),
            float(np.degrees(Rotation.from_matrix(first[:3, :3].T @ second[:3, :3]).magnitude())))


def parse_truth(raw):
    samples = []
    starts = [m.start() for m in re.finditer(r'(?m)^header \{', raw)]
    for index, start in enumerate(starts):
        block = raw[start:starts[index + 1] if index + 1 < len(starts) else len(raw)]
        # Terminating our recorder may truncate its final message. Discard that
        # incomplete transport message; do not fill it with a previous sample.
        if block.count('{') != block.count('}'):
            if index == len(starts) - 1:
                continue
            raise ValueError('Incomplete interior Gazebo message')
        head = block.split('\npose {', 1)[0]
        sec, nsec = re.search(r'\bsec: (\d+)', head), re.search(r'\bnsec: (\d+)', head)
        ts = (int(sec.group(1)) if sec else 0) * 10**9 + (int(nsec.group(1)) if nsec else 0)
        samples.append({'stamp_ns': ts, 'world_from_object': named_pose(block, 'grasp_pose_fixture'),
                        'world_from_robot': named_pose(block, 'astribot_s1')})
    if len(samples) < 2 or any(b['stamp_ns'] <= a['stamp_ns'] for a, b in zip(samples, samples[1:])):
        raise ValueError('Truth stream requires multiple strictly increasing transport stamps')
    return samples


def score_report(report, samples):
    if report.get('truth_used_as_input') is not False:
        raise ValueError('Report does not establish truth-free estimator input')
    if report.get('pose_source') != '/perception/estimate_object_pose':
        raise ValueError('Missing live Pose Action provenance')
    trials = report.get('trials', [])
    if not trials or len({t['capture_stamp_ns'] for t in trials}) != len(trials):
        raise ValueError('Missing or duplicate live capture stamps')
    first_object, first_robot = (matrix(samples[0][key]) for key in ('world_from_object', 'world_from_robot'))
    motion = {'object_translation_m': 0., 'object_rotation_deg': 0.,
              'robot_translation_m': 0., 'robot_rotation_deg': 0.}
    for sample in samples:
        for key, reference, label in (('world_from_object', first_object, 'object'),
                                       ('world_from_robot', first_robot, 'robot')):
            translation, rotation = error_between(reference, matrix(sample[key]))
            motion[label + '_translation_m'] = max(motion[label + '_translation_m'], translation)
            motion[label + '_rotation_deg'] = max(motion[label + '_rotation_deg'], rotation)
    # The scene is a static validation fixture. A moving target/robot needs
    # synchronized/interpolated simulator truth, which this scorer won't infer.
    if any(motion[key] > (1e-6 if key.endswith('_m') else 1e-4) for key in motion):
        raise ValueError(f'Observed motion invalidates static-fixture scoring: {motion}')
    stamps = [sample['stamp_ns'] for sample in samples]
    rows = []
    for trial in trials:
        ts, frame = trial['capture_stamp_ns'], trial['frame']
        tf = trial.get('capture_tf')
        if not tf or tf.get('requested_stamp_ns') != ts or tf.get('latest_tf_fallback') is not False:
            raise ValueError('Missing exact capture-time TF; old reports cannot be scored')
        if tf.get('exact_sync_stamps') != dict(rgb=ts, depth=ts, info=ts):
            raise ValueError('RGB/depth/CameraInfo stamps are not identical')
        for key, child in (('odom_from_camera', frame), ('odom_from_robot_base', BASE_FRAME)):
            transform = tf[key]
            if transform['parent'] != 'odom' or transform['child'] != child or transform['stamp_ns'] != ts:
                raise ValueError('TF frame or capture stamp mismatch')
        pos = bisect.bisect_right(stamps, ts)
        if pos == 0 or pos == len(stamps):
            raise ValueError('Capture is not bracketed by independent truth')
        before, after = samples[pos - 1], samples[pos]
        gap = (after['stamp_ns'] - before['stamp_ns']) / 1e9
        if gap > .25:
            raise ValueError('Truth bracket exceeds 250 ms')
        # Gazebo odometry is world anchored. Verify the anchor independently
        # against the measured root model, then use actual base->camera TF.
        odom_base, odom_camera = matrix(tf['odom_from_robot_base']), matrix(tf['odom_from_camera'])
        world_robot = matrix(before['world_from_robot'])
        anchor_t, anchor_r = error_between(world_robot, odom_base)
        if anchor_t > .002 or anchor_r > .2:
            raise ValueError('Independent world/odom anchor check exceeds 2 mm / 0.2 deg')
        world_camera = world_robot @ np.linalg.inv(odom_base) @ odom_camera
        camera_object = np.linalg.inv(world_camera) @ matrix(before['world_from_object'])
        result = trial.get('result', {})
        row = {'capture_stamp_ns': ts, 'frame': frame, 'task_id': trial.get('task_id'),
               'scored': False, 'passed': False, 'truth_before': before, 'truth_after': after,
               'truth_bracket_sec': gap, 'camera_from_object_truth': camera_object.tolist(),
               'world_odom_anchor_translation_m': anchor_t, 'world_odom_anchor_rotation_deg': anchor_r}
        if not trial.get('accepted') or trial.get('status') != 4 or result.get('success') is not True:
            row['reason'] = 'Action did not succeed'
            rows.append(row)
            continue
        obs = result['observation']
        header = obs['header']
        observed_ts = header['stamp']['sec'] * 10**9 + header['stamp']['nanosec']
        if observed_ts != ts or header['frame_id'] != frame:
            raise ValueError('Action result frame or capture stamp mismatch')
        if (obs['source_camera_id'] != trial['camera_id'] or obs['source_epoch'] != trial['source_epoch'] or
                obs['calibration_revision'] != trial['calibration_revision'] or
                obs['source_model'] != report['model_id'] or not obs['model_revision'] or
                not obs['position_valid'] or not obs['orientation_valid']):
            raise ValueError('Action result provenance or pose validity mismatch')
        pose = obs['pose']['pose']
        predicted = matrix({'translation': [pose['position'][key] for key in 'xyz'],
                            'quaternion_xyzw': [pose['orientation'][key] for key in 'xyzw']})
        translation, rotation = error_between(predicted, camera_object)
        row.update(scored=True, translation_error_m=translation, rotation_error_deg=rotation,
                   passed=translation <= TRANSLATION_LIMIT_M and rotation <= ROTATION_LIMIT_DEG,
                   model_revision=obs['model_revision'], source_epoch=obs['source_epoch'])
        rows.append(row)
    valid_rows = [row for row in rows if row['scored']]
    return {'schema': 'astribot.live_pose_independent_score/1', 'evaluation_valid': True,
            'truth_used_as_estimator_input': False, 'scenario': 'static clear fixture, live rendered RGB-D',
            'truth_source': 'independent /world/default/pose/info Gazebo transport',
            'camera_pose_source': 'ROS /tf and /tf_static at each exact image timestamp',
            'translation_definition': 'Euclidean translation distance in capture camera frame',
            'rotation_definition': 'SO(3) geodesic angle, no symmetry relaxation',
            'thresholds': {'translation_m': TRANSLATION_LIMIT_M, 'rotation_deg': ROTATION_LIMIT_DEG},
            'observed_truth_motion': motion, 'truth_sample_count': len(samples),
            'trial_count': len(rows), 'scored_count': len(valid_rows),
            'passed_count': sum(row['passed'] for row in rows),
            'max_translation_error_m': max((row['translation_error_m'] for row in valid_rows), default=None),
            'max_rotation_error_deg': max((row['rotation_error_deg'] for row in valid_rows), default=None),
            'trials': rows}


def record_live(args):
    if args.report.exists() or args.truth_stream.exists():
        raise ValueError('Use new report/truth paths; existing evidence is never overwritten')
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.truth_stream.parent.mkdir(parents=True, exist_ok=True)
    raw_path = args.truth_stream.with_suffix('')
    if raw_path.exists():
        raise ValueError('Raw truth evidence already exists')
    command = ['ign', 'topic', '-e', '-t', '/world/default/pose/info']
    with raw_path.open('wb') as raw, args.truth_stream.with_suffix('.stderr').open('wb') as stderr:
        recorder = subprocess.Popen(command, stdout=raw, stderr=stderr)
        try:
            deadline = time.monotonic() + 10
            while raw_path.stat().st_size < 40000:
                if recorder.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError('No actual Gazebo pose transport samples')
                time.sleep(.05)
            run = subprocess.run([sys.executable, str(Path(__file__).with_name('validate_inference_actions.py')),
                                  '--kind', 'pose', '--repeat', str(args.repeat), '--output', str(args.report)],
                                 timeout=args.repeat * 25 + 20, check=False)
            if run.returncode:
                raise RuntimeError(f'Live RGB-D Action client exited {run.returncode}')
            if recorder.poll() is not None:
                raise RuntimeError('Independent truth recorder stopped before Action capture finished')
        finally:
            # Own read-only child only; no simulator or shared ROS process is stopped.
            recorder.terminate()
            try:
                recorder.wait(timeout=3)
            except subprocess.TimeoutExpired:
                recorder.kill()
                recorder.wait(timeout=3)
            with gzip.open(args.truth_stream, 'wb') as compressed:
                compressed.write(raw_path.read_bytes())
    return {'command': command, 'pid': recorder.pid,
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
            'ign_partition': os.environ.get('IGN_PARTITION'),
            'raw_path': str(raw_path), 'raw_sha256': hashlib.sha256(raw_path.read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--truth-stream', type=Path, required=True, help='gzip of raw ign pose/info text')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--run-live', action='store_true')
    parser.add_argument('--repeat', type=int, default=5)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('Score output already exists; use a new output path')
    evidence = {}
    try:
        if args.run_live:
            evidence = record_live(args)
        raw = gzip.open(args.truth_stream, 'rt').read()
        result = score_report(json.loads(args.report.read_text()), parse_truth(raw))
        result.update(report_file=str(args.report), truth_stream_file=str(args.truth_stream),
                      report_sha256=hashlib.sha256(args.report.read_bytes()).hexdigest(),
                      truth_stream_sha256=hashlib.sha256(args.truth_stream.read_bytes()).hexdigest(),
                      recorder=evidence)
    except Exception as error:
        result = {'evaluation_valid': False, 'reason': str(error), 'recorder': evidence}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
    print(json.dumps({key: value for key, value in result.items() if key != 'trials'}), flush=True)
    return 0 if result.get('evaluation_valid') and result['passed_count'] == result['trial_count'] else 2


if __name__ == '__main__':
    sys.exit(main())
