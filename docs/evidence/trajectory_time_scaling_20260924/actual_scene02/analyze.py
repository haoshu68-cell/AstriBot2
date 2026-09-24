#!/usr/bin/env python3
"""Read existing CDR/JSON only; no rclpy.init(), nodes, ROS graph or simulation."""
import bisect
import collections
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from control_msgs.action import FollowJointTrajectory
from rclpy.serialization import deserialize_message
from trajectory_msgs.msg import JointTrajectory

SOURCE = Path('/home/yjh/WorkSpace/astribot_validation/M1_execution_response_20260924/execution_response_scene02')
OUT = Path(__file__).resolve().parent
read = lambda name: json.loads((SOURCE / name).read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
seconds = lambda stamp: stamp['sec'] + stamp['nanosec'] * 1e-9
duration = lambda stamp: stamp.sec + stamp.nanosec * 1e-9
result = read('first_stage/result.json')
journal = read('own_lease_journal.json')
guard = read('guard_raw.json')
capture = next(x for x in journal if x['event'] == 'trajectory_evidence')['details']
trajectories = {}
cdr_summaries = []
for file in capture['files']:
    path = Path(file['path'])
    msg = deserialize_message(path.read_bytes(), JointTrajectory if file['role'] == 'stage' else FollowJointTrajectory.Goal)
    traj = msg if file['role'] == 'stage' else msg.trajectory
    trajectories[file['controller'] or 'stage'] = traj
    cdr_summaries.append({
        **file, 'sha256': sha(path), 'actual_bytes': path.stat().st_size,
        'joint_names': list(traj.joint_names), 'points': len(traj.points),
        'start_s': duration(traj.points[0].time_from_start),
        'duration_s': duration(traj.points[-1].time_from_start),
        'node_max_abs_velocity_rad_s': max((abs(v) for p in traj.points for v in p.velocities), default=0),
        'node_max_abs_acceleration_rad_s2': max((abs(v) for p in traj.points for v in p.accelerations), default=0),
    })
stage = trajectories['stage']
times = [duration(p.time_from_start) for p in stage.points]
assert all(b > a for a, b in zip(times, times[1:]))
assert trajectories['arm_left_controller'] == stage
records = result['jtc_records']
eligible = [x for x in records if x['ros_s'] >= capture['ros_ns'] / 1e9 and 0 <= seconds(x['message']['desired']['time_from_start']) <= times[-1]]
origins = collections.Counter(round(seconds(x['message']['header']['stamp']) - seconds(x['message']['desired']['time_from_start']), 9) for x in eligible)
origin, _ = origins.most_common(1)[0]
active = [x for x in eligible if abs(seconds(x['message']['header']['stamp']) - seconds(x['message']['desired']['time_from_start']) - origin) < 1e-8]
assert all(x['message']['joint_names'] == list(stage.joint_names) for x in active)
replay_max = dict(position=0., velocity=0., acceleration=0.)
rows = []
for x in active:
    m = x['message']
    desired, actual = m['desired'], m['actual']
    t = seconds(desired['time_from_start'])
    i = min(bisect.bisect_right(times, t) - 1, len(times) - 2)
    p0, p1 = stage.points[i:i+2]
    dt = times[i+1] - times[i]
    u = (t - times[i]) / dt
    q0, q1 = np.array(p0.positions), np.array(p1.positions)
    v0, v1 = np.array(p0.velocities), np.array(p1.velocities)
    a0, a1 = np.array(p0.accelerations), np.array(p1.accelerations)
    dq = q1 - q0
    # Quintic Hermite polynomial in normalized segment time; p/v/a are CDR values.
    b = [q0, v0*dt, a0*dt*dt/2,
         10*dq-(6*v0+4*v1)*dt-(1.5*a0-.5*a1)*dt*dt,
         -15*dq+(8*v0+7*v1)*dt+(1.5*a0-a1)*dt*dt,
         6*dq-(3*v0+3*v1)*dt-(.5*a0-.5*a1)*dt*dt]
    q = sum(b[k]*u**k for k in range(6))
    v = sum(k*b[k]*u**(k-1) for k in range(1, 6))/dt
    a = sum(k*(k-1)*b[k]*u**(k-2) for k in range(2, 6))/dt**2
    for label, observed, replayed in [('position', desired['positions'], q), ('velocity', desired['velocities'], v), ('acceleration', desired['accelerations'], a)]:
        replay_max[label] = max(replay_max[label], float(np.max(np.abs(np.array(observed)-replayed))))
    for j, name in enumerate(stage.joint_names):
        rows.append({'source_ros_s': seconds(m['header']['stamp']), 'receive_ros_s': x['ros_s'], 'receive_monotonic_s': x['wall'],
                     'trajectory_time_s': t, 'segment': i, 'joint': name,
                     'desired_position': desired['positions'][j], 'actual_position': actual['positions'][j],
                     'position_error': desired['positions'][j]-actual['positions'][j],
                     'message_position_error': m['error']['positions'][j],
                     'desired_velocity': desired['velocities'][j], 'actual_velocity': actual['velocities'][j],
                     'desired_acceleration': desired['accelerations'][j],
                     'replayed_position': float(q[j]), 'replayed_velocity': float(v[j]), 'replayed_acceleration': float(a[j])})
with (OUT/'jtc_active_samples.csv').open('w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
guard_active = [x for x in guard if x['data']['active'] and x['data']['context_id'] == result['context_id']]
guard_peak = max(guard_active, key=lambda x: x['data']['maximum_joint_error_rad'])
peak_row = max(rows, key=lambda x: abs(x['position_error']))
terminal = [(i, x['details']) for i, x in enumerate(journal) if x['event'] == 'child_terminal']
responses = {x['details']['controller']: x['details']['uuid'] for x in journal if x['event'] == 'child_response'}
hold_feedback = [x for x in result['feedback_records'] if x['feedback']['hold_confirmed']]
manifest = read('manipulation_runtime_manifest.json')
identity = [x for x in manifest if Path(x['identity']['exe']).name in ['mtc_planner', 'trajectory_executor']]
for item in identity:
    item['current_file_sha256_matches_recorded'] = sha(Path(item['identity']['exe'])) == item['executable_sha256']
hold_status = []
for i, item in enumerate(read('executor_status_raw.json')):
    data = json.loads(item['data']['data'])
    if data.get('lease_id') == journal[0]['lease_id'] and data.get('hold_confirmed'):
        hold_status.append({'index': i, **item, 'decoded': data})
cleanup_audit = read('post_cleanup_identity_audit.json')
iptp_lines = [line for line in (SOURCE/'transport_skills.log').read_text().splitlines() if 'baseline (IPTP' in line]
endpoint = result['first_stage_evidence']['jtc_state']
endpoint_message = endpoint['message']
source_files = ['first_stage/result.json', 'own_lease_journal.json', 'guard_raw.json', 'mtc_parameters_pre_action.json', 'manipulation_runtime_manifest.json', 'executor_status_raw.json', 'runtime_manifest.json', 'gz_control_runtime_manifest.json', 'transport_skills.log', 'result.json', 'post_cleanup_identity_audit.json']
report = {
    'evidence_layer': 'offline independent analysis of one completed owned simulation',
    'source_directory': str(SOURCE), 'task_id': result['task_id'], 'context_id': result['context_id'],
    'lease_id': journal[0]['lease_id'], 'source_sha256': {name: sha(SOURCE/name) for name in source_files},
    'runtime_identity': identity, 'parameters': result['mtc_parameters'], 'pre_action_parameters': read('mtc_parameters_pre_action.json'),
    'cdr': cdr_summaries, 'stage_equals_actual_left_goal': trajectories['arm_left_controller'] == stage,
    'capture_ros_s': capture['ros_ns']/1e9, 'jtc_inferred_start_ros_s': origin,
    'same_run_pre_scaling_log': {'first_stage': iptp_lines[0], 'note': 'Rounded IPTP log says 426 points, 3.048 s, 0.801 rad/s, 4.089 rad/s^2; compatible with the CDR after t*2.5, v/2.5, a/6.25. The full unscaled instance was not captured.'},
    'jtc': {
        'topic': '/arm_left_controller/controller_state', 'samples': len(active), 'joint_rows': len(rows),
        'source_ros_window': [seconds(active[0]['message']['header']['stamp']), seconds(active[-1]['message']['header']['stamp'])],
        'trajectory_time_window': [seconds(active[0]['message']['desired']['time_from_start']), seconds(active[-1]['message']['desired']['time_from_start'])],
        'source_gap_s_max': max(seconds(b['message']['header']['stamp'])-seconds(a['message']['header']['stamp']) for a,b in zip(active,active[1:])),
        'receive_gap_s_max': max(b['wall']-a['wall'] for a,b in zip(active,active[1:])),
        'max_sample_abs_position_error_rad': abs(peak_row['position_error']), 'peak_error_sample': peak_row,
        'max_message_error_disagreement_rad': max(abs(x['position_error']-x['message_position_error']) for x in rows),
        'max_sample_abs_desired_velocity_rad_s': max(abs(x['desired_velocity']) for x in rows),
        'max_sample_abs_desired_acceleration_rad_s2': max(abs(x['desired_acceleration']) for x in rows),
        'max_sample_abs_actual_velocity_rad_s': max(abs(x['actual_velocity']) for x in rows),
        'quintic_replay_max_difference': replay_max,
        'replay_method': 'quintic Hermite interpolation of CDR p/v/a at each recorded desired.time_from_start; no controller process invoked',
    },
    'guard': {'active_samples': len(guard_active), 'states': [{'healthy': h, 'reason': reason, 'count': n} for (h,reason),n in collections.Counter((x['data']['healthy'],x['data']['reason']) for x in guard_active).items()],
              'source_ros_window': [seconds(guard_active[0]['data']['stamp']), seconds(guard_active[-1]['data']['stamp'])],
              'max_sample_joint_error_rad': guard_peak['data']['maximum_joint_error_rad'], 'peak_sample': guard_peak,
              'max_sample_base_translation_m': max(x['data']['base_translation_m'] for x in guard_active),
              'max_sample_base_rotation_rad': max(x['data']['base_rotation_rad'] for x in guard_active)},
    'six_child_successes': len(terminal)==6 and all(x['success'] and x['result_code']==4 and responses[x['controller']]==x['uuid'] for _,x in terminal),
    'child_terminal_journal_entries': terminal,
    'hold_observed': {'journal_indices': [i for i,x in enumerate(journal) if x['event']=='hold_confirmed'], 'feedback_samples': len(hold_feedback), 'first_feedback': hold_feedback[0], 'last_feedback': hold_feedback[-1], 'status_samples': len(hold_status), 'first_status': hold_status[0], 'last_status': hold_status[-1]},
    'endpoint': {'source_ros_s': seconds(endpoint_message['header']['stamp']), 'actual_vs_cdr_endpoint_max_error_rad': max(abs(x-y) for x,y in zip(endpoint_message['actual']['positions'],stage.points[-1].positions)), 'desired_vs_cdr_endpoint_max_error_rad': max(abs(x-y) for x,y in zip(endpoint_message['desired']['positions'],stage.points[-1].positions)), 'actual_max_abs_velocity_rad_s': max(map(abs,endpoint_message['actual']['velocities']))},
    'protocol': {'overall_passed': result['passed'], 'error': result['error'], 'acks_recorded': len(result['acks']), 'cleanup_complete': result['cleanup_complete'], 'hold_handle_unresolved': result['hold_handle_unresolved'], 'resource_disposition': result['resource_disposition'], 'final_journal_events': [{k:x[k] for k in ['event','phase','reason','lease_id']} for x in journal[-2:]], 'six_consumer_acks': 'not verified', 'cancel_release': 'not verified'},
    'outer_process_cleanup': {'cleanup_complete': read('result.json')['cleanup_complete'], 'identity_audit_records': len(cleanup_audit), 'all_recorded_owned_processes_stopped': all(not x['same_process_alive'] for x in cleanup_audit), 'not_business_resource_release': True},
    'limits': ['Sampled maxima are not continuous-time upper bounds.', 'Only left-arm JTC state was recorded in first_stage/result.json; other controllers have Goal CDR and terminal evidence.', 'This 426-point path differs from the old 717-point path: no exact old/new trajectory A/B or unique causal attribution.', 'The live run used the recorded scale=2.5 candidate; no full unscaled instance of this exact path was captured to directly measure the ratio.', 'Motion success and Hold observation do not imply complete protocol PASS; ACK collection failed and lease subsequently expired into quarantine.', 'Journal issued_at/valid_until are lease fields, not per-event timestamps: no exact quarantine time is inferred from them.', 'No new ROS nodes, simulation, rebuild or GPU computation. Original tracking issue start/checkpoint remain 12:15/13:15.'],
}
(OUT/'analysis.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
print(json.dumps({k:report[k] for k in ['parameters','jtc','guard','six_child_successes','endpoint','protocol']}, indent=2))
