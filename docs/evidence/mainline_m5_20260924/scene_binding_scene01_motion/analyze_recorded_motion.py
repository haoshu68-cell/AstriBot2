#!/usr/bin/env python3
"""Offline evidence extraction; no ROS, control or inference."""
import csv
import hashlib
import json
from pathlib import Path
import re

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

source = Path('/home/yjh/WorkSpace/astribot_validation/M1_scene_binding_20260924/scene_binding_scene01')
output = Path(__file__).resolve().parent
result = json.loads((source/'first_stage/result.json').read_text())
journal = json.loads((source/'own_lease_journal.json').read_text())
video = json.loads((source/'first_stage.json').read_text())
snapshot = json.loads((source/'head_snapshot/camera_info.json').read_text())
jtc = result['jtc_records']
times = np.array([row['ros_s'] for row in jtc])
stamps = np.array([row['message']['header']['stamp']['sec']+
                   row['message']['header']['stamp']['nanosec']*1e-9 for row in jtc])
assert np.all(times == stamps)
names = jtc[0]['message']['joint_names']
assert all(row['message']['joint_names'] == names for row in jtc)
actual = np.array([row['message']['feedback']['positions'] for row in jtc])
velocity = np.array([row['message']['feedback']['velocities'] for row in jtc])
reference = np.array([row['message']['reference']['positions'] for row in jtc])
errors = np.array([row['message']['error']['positions'] for row in jtc])
transitions, previous = [], None
for row in result['feedback_records']:
    value = row['feedback']
    key = (value['phase'], value['reason'])
    if key != previous:
        transitions.append({'observer_ros_s': row['ros_s'], 'receive_steady_s': row['wall'],
                            'phase': key[0], 'reason': key[1]})
    previous = key
executing = next(row for row in transitions if row['reason'] == 'EXECUTING_FIRST_MTC_STAGE')
guard = next(row for row in transitions if row['reason'] == 'EXECUTION_GUARD_UNHEALTHY')
children = {}
for event in ('child_submission', 'child_response', 'child_terminal'):
    children[event] = [row['details'] for row in journal if row['event'] == event]
controllers = {row['controller'] for row in children['child_submission']}
assert len(controllers) == 6 and all(len(rows) == 6 for rows in children.values())
response_ids = {row['controller']: row['uuid'] for row in children['child_response']}
assert all(row['uuid'] == response_ids[row['controller']] for row in children['child_terminal'])
assert all(row['success'] is False and row['result_code'] == 5 for row in children['child_terminal'])
events = [{'journal_index': i, 'event': row['event'], 'lease_issued_at_ns': row['issued_at'],
           'reason': row['reason'], 'details': row.get('details')} for i, row in enumerate(journal)]
(output/'recorded_events.json').write_text(json.dumps({'task_id': result['task_id'],
    'parent_request': result['parent_request'], 'feedback_transitions': transitions,
    'journal': events, 'journal_time_warning': 'issued_at is a lease field, not an event timestamp; use explicit scene_evidence ros_ns/steady_ns or observer feedback timestamps where available.'}, indent=2)+'\n')
with (output/'left_jtc.csv').open('w', newline='') as stream:
    writer = csv.writer(stream)
    writer.writerow(['source_ros_s', 'receive_steady_s'] +
                    [f'{kind}:{name}' for kind in ('reference_rad', 'actual_rad', 'velocity_rad_s', 'error_rad') for name in names])
    for i, row in enumerate(jtc):
        writer.writerow([stamps[i], row['wall'], *reference[i], *actual[i], *velocity[i], *errors[i]])
states = video['stage_frames']
wait_video = next(s for s in states if s['stage'] == 'WAITING_FOR_EXECUTION_GUARD')
execute_video = next(s for s in states if s['stage'] == 'EXECUTING_FIRST_MTC_STAGE')
guard_video = next(s for s in states if s['stage'] == 'EXECUTION_GUARD_UNHEALTHY')
assert wait_video['frame'] == execute_video['frame'] == guard_video['frame'] == 118
cap = cv2.VideoCapture(str(source/'first_stage.mp4'))
assert cap.isOpened()
count, fps = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), cap.get(cv2.CAP_PROP_FPS)
assert count == video['frames'] == 127 and fps == video['encoding_fps'] == 10
excerpt = cv2.VideoWriter(str(output/'before_after_excerpt.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), fps, (1280, 720))
assert excerpt.isOpened()
for index in range(count):
    ok, frame = cap.read()
    assert ok, index
    if index in (110, 117, 118, 119, 126):
        assert cv2.imwrite(str(output/f'frame_{index:03d}.png'), frame)
    if index >= 110:
        excerpt.write(frame)
cap.release()
excerpt.release()
fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, constrained_layout=True)
window = (stamps >= 38.15) & (stamps <= 39.45)
axes[0].plot(stamps[window], reference[window, 0], label='Left joint 1 reference', linewidth=1.6)
axes[0].plot(stamps[window], actual[window, 0], label='Left joint 1 actual', linewidth=1.6)
axes[0].set_ylabel('Position (rad)')
axes[0].legend(loc='lower right')
axes[1].plot(stamps[window], velocity[window, 0], label='Left joint 1 actual velocity', color='#237755')
axes[1].set_ylabel('Velocity (rad/s)')
axes[1].set_xlabel('JTC source / observer ROS time (s; equal in this record)')
for axis in axes:
    axis.axvline(executing['observer_ros_s'], color='#777777', linestyle='--', label='Execution feedback')
    axis.axvline(guard['observer_ros_s'], color='#b13b31', linestyle='--', label='Guard unhealthy feedback')
    axis.grid(alpha=.25)
axes[1].legend(loc='upper right', fontsize=8)
fig.suptitle('Recorded left-arm motion and stop convergence\nFeedback markers are observations, not the original guard event')
fig.savefig(output/'left_joint_timing.png', dpi=160)
plt.close(fig)
log = (source/'stack/session.log').read_text()
stale = []
for number, line in enumerate(log.splitlines(), 1):
    match = re.search(r'CAMERA_STALE camera=(\S+) ros_age=(\S+) wall_age=(\S+) limit=(\S+) capture_ns=(\d+) now_ns=(\d+)', line)
    if match:
        camera, ros_age, steady_age, limit, capture, now = match.groups()
        stale.append(dict(line=number, camera=camera, ros_age_s=float(ros_age), steady_age_s=float(steady_age),
                          limit_s=float(limit), capture_stamp_ns=int(capture), observer_ros_ns=int(now)))
files = sorted(str(path.relative_to(source)) for path in source.rglob('*') if path.is_file())
passive_names = {'events.jsonl', 'manifest.json', 'stream_timing.json', 'health_events.json', 'metadata.yaml'}
passive_matches = [name for name in files if Path(name).name in passive_names or name.endswith(('.db3', '.mcap'))]
camera = dict(motion_window_camera_health='NOT_MEASURED', motion_window_projection_health='NOT_MEASURED',
    motion_window_raw_receive_gaps='NOT_MEASURED', motion_window_capture_age='NOT_MEASURED',
    delivered_file_count=len(files), delivered_passive_capture_candidates=passive_matches,
    inventory=files, snapshot_capture_stamp_ns=snapshot['capture_stamp_ns'],
    snapshot_capture_age_s=snapshot['capture_age_at_end_sec'],
    snapshot_before_execution_feedback_s=executing['observer_ros_s']-snapshot['capture_stamp_ns']/1e9,
    snapshot_camera_health=snapshot['camera_health_at_capture'], snapshot_stream_counts=snapshot['stream_counts'],
    snapshot_stream_sim_hz=snapshot['stream_sim_hz'], snapshot_wall_observation_seconds=snapshot['wall_observation_seconds'],
    snapshot_scope='Pre-action head-only snapshot; aggregate counts are not per-frame gap/age observations.',
    camera_stale_log_rows=stale, stale_scope='All logged capture stamps precede motion; warning rows are not raw-image gap counts. No warning during motion is not evidence of valid health.',
    video_age_scope='Recorder stores neither image header stamps nor per-frame receive times. Head inset is cached; its age cannot be reconstructed.',
    guard_original_reason='NOT_RECORDED', root_cause_scope='No control diagnosis or inference that RESOURCE_CLOCK_RESET proves a time rollback.')
(output/'camera_evidence_gaps.json').write_text(json.dumps(camera, indent=2)+'\n')
report = dict(scope='Short actual left-arm motion and recorded stop convergence; not completed PREGRASP/Hold/PICK.',
    task_id=result['task_id'], children=children, parent_terminal=result['parent_terminal'],
    measured_joint_names=names, jtc_samples=len(jtc), jtc_source_ros_range_s=[float(stamps[0]), float(stamps[-1])],
    reference_change_first_s=float(stamps[np.flatnonzero(np.max(np.abs(reference-reference[0]), axis=1)>1e-6)[0]]),
    actual_change_first_s=float(stamps[np.flatnonzero(np.max(np.abs(actual-actual[0]), axis=1)>1e-6)[0]]),
    onset_detection_scope='1e-6 rad identifies change above numeric jitter for this report; not a control or safety threshold.',
    actual_joint_range_rad=np.ptp(actual, axis=0).tolist(), actual_max_speed_rad_s=float(np.abs(velocity).max()),
    max_tracking_error_rad=float(np.abs(errors).max()), last_max_speed_rad_s=float(np.abs(velocity[-1]).max()),
    last_max_tracking_error_rad=float(np.abs(errors[-1]).max()),
    execution_feedback_ros_s=executing['observer_ros_s'], guard_feedback_ros_s=guard['observer_ros_s'],
    feedback_window_sim_s=guard['observer_ros_s']-executing['observer_ros_s'],
    feedback_window_steady_s=guard['receive_steady_s']-executing['receive_steady_s'],
    measured_envelope_messages=len(result['envelopes']), measured_ack_messages=len(result['acks']),
    hold_confirmed=any(row['feedback']['hold_confirmed'] for row in result['feedback_records']),
    video_path=str(source/'first_stage.mp4'), video_frames=count, video_fps=fps,
    video_playback_seconds=count/fps, video_capture_wall_seconds=video['wall_duration_s'],
    video_stage_frames=states, unchanged_encoded_frame_index=118,
    no_new_encoded_frame_wait_to_guard_wall_s=guard_video['wall_time']-wait_video['wall_time'],
    no_new_encoded_frame_execute_to_guard_wall_s=guard_video['wall_time']-execute_video['wall_time'],
    visual_review='Original decoded frames117 and118 show a small left-arm/gripper pose change. Frame118 already carries GUARD_UNHEALTHY. No continuous executing-labelled frames were encoded.',
    alignment_limit='Ordering is consistent, but image acquisition stamps are absent. Later frames may contain queued images acquired during execution; do not assert no upstream images or exact image-to-JTC time alignment.',
    excerpt=dict(path=str(output/'before_after_excerpt.mp4'), original_frames_inclusive=[110,126],
                 frames=17, playback_seconds=1.7, processing='Same decoded pixels re-encoded at original10fps, no interpolation. Before/after excerpt, not continuous motion acceptance.'),
    guard_original_reason='NOT_RECORDED', clock_rollback_proven=False,
    cleanup_scope='Owner identity audit confirms12unique processes exited; MoveGroup unloading exit-11 prevents clean-shutdown acceptance.',
    evidence_acceptance=dict(actual_left_arm_motion='OBSERVED', event_order='CONSISTENT',
        exact_video_jtc_alignment='NOT_MEASURED', continuous_motion_video_coverage='NOT_VERIFIED',
        camera_motion_age='NOT_MEASURED', guard_root_cause='UNKNOWN', complete_first_stage='NOT_PASSED',
        six_envelope_acks='NOT_OBSERVED', full_pick='NOT_EVALUATED'))
(output/'motion_video_report.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps({'actual_change_first_s': report['actual_change_first_s'],
                  'joint1_range_rad': report['actual_joint_range_rad'][0],
                  'video_frames': count, 'missing_motion_camera_age': True}))
