"""Offline evidence checks shared by new episodes and historical replay."""
import math
from bisect import bisect_right

SOCIAL_STATUS_MAX_AGE_S = 1.0


def assess_evidence(case, records, summary, policy, navigation_status=None):
    checks = dict(summary.get('checks', {}))
    groups = {kind: [r for r in records if r.get('kind') == kind]
              for kind in ('geometry', 'command', 'motion', 'social', 'execution')}
    # A newly created use_sim_time node can receive data before its first clock.
    # Retain those raw records but exclude only the leading clock-zero prefix.
    for kind, rows in groups.items():
        first = next((i for i, r in enumerate(rows) if r.get('ros_s', 0) > 0), len(rows))
        groups[kind] = rows[first:]
    moving = [r for r in groups['motion'] if r.get('v', 0) > .01 or abs(r.get('w', 0)) > .02]
    checks['movement_observed'] = len(moving) >= 3 if case.get('expect_motion', True) else True
    for kind in ('geometry', 'command', 'motion'):
        rows = groups[kind]
        stamps = [r.get('ros_s') for r in rows]
        finite = all(isinstance(t, (int, float)) and math.isfinite(t) for t in stamps)
        checks[kind + '_samples'] = len(rows) >= 3 and finite
        if finite and len(stamps) >= 3:
            checks[kind + '_continuity'] = stamps[-1] > stamps[0] and all(
                0 <= b-a and (kind == 'command' or b-a <= case.get('max_sample_gap_s', 0.75))
                for a, b in zip(stamps, stamps[1:]))
        else:
            checks[kind + '_continuity'] = False
    for kind in ('command', 'motion'):
        checks[kind + '_finite'] = bool(groups[kind]) and all(
            isinstance(r.get(key), (int, float)) and math.isfinite(r[key])
            for r in groups[kind] for key in ('v', 'w'))
    for kind, key, scale in (('geometry','stamp_ns',1e-9),('motion','source_s',1.)):
        rows = groups[kind]
        # Older recordings did not include odometry source_s. Their replay is
        # explicitly historical; new schema-1 evidence must contain it.
        required = kind == 'geometry' or case.get('schema_version') == 1
        if required or any(key in r for r in rows):
            stamps = [r.get(key) for r in rows]
            finite = bool(stamps) and all(isinstance(t,(int,float)) and math.isfinite(t) for t in stamps)
            checks[kind+'_source_finite'] = finite
            checks[kind+'_source_progress'] = finite and stamps[-1]>stamps[0] and all(
                b>=a for a,b in zip(stamps,stamps[1:]))
            checks[kind+'_source_age'] = finite and all(-.05<=r['ros_s']-t*scale<=.3
                                                        for r,t in zip(rows,stamps))
    if groups['geometry']:
        checks['geometry'] = all(r.get('geometry_valid') is True and
            r.get('conservative_overlap_steps', 1) == 0 for r in groups['geometry'])
    def covers_motion(rows, maximum_age):
        stamps = [r['ros_s'] for r in rows]
        if not stamps or not moving or stamps != sorted(stamps):
            return False
        for row in moving:
            index = bisect_right(stamps, row['ros_s']) - 1
            if index < 0 or row['ros_s'] - stamps[index] > maximum_age:
                return False
        return True

    checks['command_covers_motion'] = covers_motion(groups['command'], .75)
    checks['geometry_covers_motion'] = covers_motion(groups['geometry'], .75)
    if policy != 'off':
        active = [r for r in groups['social'] if r.get('active')]
        checks['policy_observed'] = len(active) >= 3
        # Between goals the adapter reports active=false; this is still valid
        # telemetry, unlike a missing stream. Authority is checked separately.
        checks['policy_telemetry_covers_motion'] = covers_motion(groups['social'], SOCIAL_STATUS_MAX_AGE_S)

    # The social policy may deliberately select SLOW/YIELD while both speed
    # candidates are still geometrically admissible. Preserve that decision
    # as exercised conflict evidence; requiring an inadmissible candidate
    # would reject the goal-occupied and early-warning cases.
    if case.get('expect_conflict') and not summary.get('_force_conflict_false'):
        reasons = {'PREDICTED_CONFLICT', 'YIELD', 'STATIONARY_CONFLICT',
                   'SOCIAL_BLOCKED_TIMEOUT', 'GOAL_OCCUPIED'}
        conflict_rows = [r for r in groups['social'] if r.get('active') and (
            any(not c.get('admissible', True) for c in r.get('candidates', [])) or
            (r.get('goal_occupied') and r.get('reason') in reasons) or
            r.get('reason') in reasons)]
        checks['conflict_exercised'] = bool(conflict_rows)
        if conflict_rows:
            first_conflict = conflict_rows[0].get('ros_s', float('inf'))
            checks['recovery_exercised'] = any(
                r.get('state') == 'CRUISE' and r.get('ros_s', 0.) > first_conflict
                for r in groups['social'])

    if case.get('cancel'):
        checks.pop('arrivals', None)
        checks.pop('recovery_exercised', None)
        status = navigation_status or {}
        checks['cancel_terminal'] = (status.get('state') == 'stopped' and
            status.get('cancel_action_status') == 5 and not status.get('cancel_error'))
        cancel_events = [e for e in summary.get('events', []) if e.get('command') == 'cancel_navigation']
        checks['cancel_triggered'] = len(cancel_events) == 1
        executing = [r for r in groups['execution'] if r.get('state') == 'EXECUTING' and
                     cancel_events and r['ros_s'] <= cancel_events[0]['ros_s']]
        task = executing[-1].get('task_id') if executing else None
        canceled_tasks = [r for r in groups['execution'] if r.get('state') == 'CANCELED' and
                          r.get('task_id') == task and r.get('action_status') == 5]
        checks['cancel_execution_terminal'] = bool(task) and bool(cancel_events) and any(
            r['ros_s'] >= cancel_events[0]['ros_s'] and r['sequence'] > executing[-1]['sequence']
            for r in canceled_tasks)
        checks['no_task_reactivation_after_cancel'] = bool(cancel_events) and not any(
            r.get('state') in ('ACCEPTED', 'EXECUTING') and r['ros_s'] > cancel_events[0]['ros_s']
            for r in groups['execution'])
        checks['motion_before_cancel'] = bool(cancel_events) and any(
            r['v'] > 0.02 and r['ros_s'] < cancel_events[0]['ros_s'] for r in groups['motion'])
        stationary_before = case['cancel'].get('stationary_before_s', 0.)
        if stationary_before:
            end = cancel_events[0]['ros_s'] if cancel_events else -math.inf
            begin = end-stationary_before
            window = [r for r in groups['motion'] if begin-.3 <= r.get('source_s', -math.inf) <= end]
            first = max((i for i,r in enumerate(window) if r['source_s'] <= begin), default=-1)
            window = window[first:] if first >= 0 else []
            checks['stationary_before_cancel'] = bool(window) and end-window[-1]['source_s'] <= .3 and all(
                r['v'] < .01 and abs(r['w']) < .02 for r in window) and all(
                0 <= b['source_s']-a['source_s'] <= .3 for a,b in zip(window,window[1:]))
            waiting = [r for r in groups['social'] if begin-SOCIAL_STATUS_MAX_AGE_S <= r['ros_s'] <= end]
            first = max((i for i,r in enumerate(waiting) if r['ros_s'] <= begin), default=-1)
            waiting = waiting[first:] if first >= 0 else []
            checks['wait_covers_stationary_cancel'] = bool(waiting) and end-waiting[-1]['ros_s'] <= SOCIAL_STATUS_MAX_AGE_S and all(
                r.get('state')=='WAIT' and any(not c['admissible'] for c in r.get('candidates', []))
                for r in waiting) and all(0 <= b['ros_s']-a['ros_s'] <= SOCIAL_STATUS_MAX_AGE_S for a,b in zip(waiting,waiting[1:]))
        if cancel_events:
            start = cancel_events[0]['ros_s'] + case['cancel'].get('settle_s', 1.5)
            for kind in ('command', 'motion'):
                rows = [r for r in groups[kind] if r['ros_s'] >= start]
                checks[kind + '_stopped_after_cancel'] = len(rows) >= 10 and (
                    rows[-1]['ros_s'] - rows[0]['ros_s'] >= 1.0 and
                    all(r['v'] < 0.01 and abs(r['w']) < 0.02 for r in rows))
        else:
            checks['command_stopped_after_cancel'] = False
            checks['motion_stopped_after_cancel'] = False
    else:
        checks['navigation_process_completed'] = summary.get('navigation_returncode') == 0

    # Once a dynamic-obstacle stop has settled, no body motion is allowed
    # until the explicit resume event. This catches a command leak that a
    # single sampled HOLD status can miss.
    if case.get('expect_goal_occupied') and summary.get('pause_stop_evidence'):
        evidence = summary['pause_stop_evidence']
        settled = evidence.get('settled_source_s')
        resume = next((e.get('ros_s') for e in summary.get('events', [])
                       if e.get('command') == 'start' and
                       e.get('ros_s', -1) > evidence.get('pause_source_s', float('inf'))), None)
        motion_rows = groups['motion']
        occupied = [r for r in motion_rows if settled is not None and resume is not None and
                    settled + .2 <= r.get('ros_s', -1) < resume]
        checks['no_motion_while_occupied'] = bool(occupied) and all(
            r.get('v', 0.) < .01 and abs(r.get('w', 0.)) < .02 for r in occupied)

    missing = [name for name, passed in checks.items() if passed is not True]
    invalid_fixture = {'episode_ack', 'human_moved', 'conflict_exercised',
                       'pause_and_resume', 'human_stopped', 'goal_actually_occupied',
                       'cancel_triggered', 'motion_before_cancel', 'movement_observed'}
    if not missing:
        verdict = 'PASS'
    elif checks.get('geometry') is False and groups['geometry']:
        verdict = 'FAIL'
    elif any(name.endswith(('_samples', '_continuity', '_finite','_source_progress','_source_age')) or
             name == 'policy_observed' or name.endswith('_covers_motion') for name in missing):
        verdict = 'INFRA_FAILURE'
    elif any(name in invalid_fixture for name in missing):
        verdict = 'INVALID_FIXTURE'
    else:
        verdict = 'FAIL'
    return {'checks': checks, 'verdict': verdict, 'scenario_passed': verdict == 'PASS',
            'failed_checks': missing, 'evidence_schema_version': 1}
