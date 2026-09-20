#!/usr/bin/env python3
"""Read-only guard/cancel timing from an owned simulation's recorded evidence."""
import argparse
import json
import math
from pathlib import Path


def stamp(value):
    return value['sec'] + value['nanosec'] * 1e-9


def analyze(payload, task, injection=None):
    events = [json.loads(line) for line in (task / 'events.jsonl').read_text().splitlines()]
    sources = [json.loads(line) for line in (payload / 'sources.jsonl').read_text().splitlines()]
    faults = [r for r in sources if r['kind'] == 'execution_guard' and
              r['message']['active'] and not r['message']['healthy'] and
              r['message']['reason'] != 'WAITING_FOR_EXECUTION_EVIDENCE']
    result = dict(evidence='observed simulation; action terminal and measured stop are separate',
                  velocity_threshold_rad_s=.01, stable_window_sim_s=.3,
                  task_state=json.loads((task / 'state.json').read_text()))
    if not faults and injection is None:
        return dict(result, guard_fault_message_observed=False)
    fault = faults[0] if faults else None
    start_sim = injection['ros_time_s'] if injection else stamp(fault['message']['stamp'])
    start_wall = injection['wall_time'] if injection else fault['received_wall_s']
    result.update(guard_fault_message_observed=bool(faults), first_fault=fault,
                  timing_reference='injected guard exit' if injection else 'first received guard fault',
                  injection=injection,
                  transaction_events=[e for e in events if 'executor_event' in e])
    for name in ('fault_observed', 'cancel_requested', 'cancel_response', 'terminal_after_cancel'):
        matching = [e for e in events if e.get('executor_event') == name and e['wall_time'] >= start_wall-.1]
        if matching:
            result[name + '_latency_wall_s'] = matching[0]['wall_time'] - start_wall
    stable_start = previous = None
    maximum_error = 0.
    for row in sources:
        if row['kind'] != 'arm_controller':
            continue
        msg = row['message']
        t = stamp(msg['header']['stamp'])
        if t < start_sim:
            continue
        actual, desired = msg['actual']['positions'], msg['desired']['positions']
        if len(actual) == len(desired) and actual:
            maximum_error = max(maximum_error, max(abs(a-b) for a, b in zip(actual, desired)))
        v = msg['actual']['velocities']
        valid = len(v) == len(msg['joint_names']) > 0 and all(math.isfinite(x) and abs(x) <= .01 for x in v)
        # Repeated publication of one controller acquisition is not another
        # sample. It neither advances nor restarts a valid stop window.
        if previous == t:
            if not valid:
                stable_start = None
            continue
        if not valid or stable_start is None or previous is None or not 0 < t-previous <= .1:
            stable_start = t if valid else None
        previous = t
        if stable_start is not None and t-stable_start >= .3:
            result.update(measured_stop_onset_sim_s=stable_start-start_sim,
                          measured_stop_confirmed_sim_s=t-start_sim,
                          maximum_tracking_error_until_stop_rad=maximum_error)
            break
    else:
        result['measured_stop_confirmed_sim_s'] = None
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--payload', type=Path, required=True)
    p.add_argument('--task', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--injection', type=Path)
    args = p.parse_args()
    result = analyze(args.payload, args.task, json.loads(args.injection.read_text()) if args.injection else None)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('first_fault', 'transaction_events', 'task_state')}, indent=2))
