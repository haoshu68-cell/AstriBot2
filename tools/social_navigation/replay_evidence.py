#!/usr/bin/env python3
"""Recheck recorded episodes and inject bad evidence without publishing ROS data."""
import argparse
import copy
import json
from pathlib import Path
import sys

from episode_evidence import assess_evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episodes', nargs='+', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = dict(boundary='Offline replay and evidence fault injection; no new motion validation',
                  replays=[], negative_checks=[])
    for directory in args.episodes:
        case = json.loads((directory/'case.json').read_text())
        summary = json.loads((directory/'summary.json').read_text())
        rows = [json.loads(line) for line in (directory/'observations.jsonl').read_text().splitlines()]
        status_path = directory/'navigation/status.json'
        status = json.loads(status_path.read_text()) if status_path.exists() else {}
        policy = summary['expected_policy']
        stop_profile=None
        if case.get('pause',{}).get('stop_contract')=='source_hold_budget_v2':
            sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ws_robot/src/astribot_s1_navigation_policy'))
            from astribot_s1_navigation_policy.profile import Profile
            from pause_stop_evidence import assess_pause_stop
            stop_profile=Profile(json.loads((directory/'stop_profile.json').read_text())['resolved'])
            pause=next(e['ros_s'] for e in summary['events'] if e.get('command')=='pause')
            resume=next(e['ros_s'] for e in summary['events'] if e.get('command')=='start' and e['ros_s']>pause)
        def assess(data, result=None):
            result=copy.deepcopy(summary if result is None else result)
            if stop_profile:
                try:
                    evidence=assess_pause_stop(data,pause,resume,stop_profile.stopping_distance,
                        stop_profile.clearance_margin_m)
                    result['checks'].update(evidence['checks'])
                except (ValueError,KeyError,TypeError,ZeroDivisionError):
                    result['checks']['pause_stop_evidence']=False
            return assess_evidence(case, data, result, policy, status)
        report['replays'].append(dict(directory=str(directory.resolve()), **assess(rows)))
        def negative(name, data, result=None):
            verdict = assess(data, result)
            report['negative_checks'].append(dict(directory=str(directory.resolve()), fault=name,
                rejected=not verdict['scenario_passed'], verdict=verdict['verdict'], failed_checks=verdict['failed_checks']))
        for kind in ('geometry', 'command', 'motion'):
            negative('missing_' + kind, [r for r in rows if r['kind'] != kind])
            broken = copy.deepcopy(rows)
            positive = [r for r in broken if r['kind'] == kind and r['ros_s'] > 0]
            if len(positive) < 3:
                raise ValueError('Reference episode lacks evidence for ' + kind)
            positive[len(positive)//2]['ros_s'] = -1
            negative('clock_reversal_' + kind, broken)
        broken = copy.deepcopy(rows)
        next(r for r in broken if r['kind'] == 'geometry' and r['ros_s'] > 0)['conservative_overlap_steps'] = 1
        negative('physical_overlap', broken)
        for kind in ('command', 'motion'):
            broken = copy.deepcopy(rows)
            next(r for r in broken if r['kind'] == kind and r['ros_s'] > 0)['v'] = float('nan')
            negative('nonfinite_' + kind, broken)
        for kind,key in (('geometry','stamp_ns'),('motion','source_s')):
            eligible=[r for r in rows if r['kind']==kind and key in r and r['ros_s']>0]
            if eligible:
                broken=copy.deepcopy(rows)
                for r in broken:
                    if r['kind']==kind:r[key]=eligible[0][key]
                negative('frozen_source_'+kind,broken)
        if case.get('cancel'):
            broken = copy.deepcopy(rows)
            for r in broken:
                if r['kind'] == 'execution' and r.get('state') == 'CANCELED':
                    r['task_id'] = 'unrelated_task'
            negative('wrong_task_canceled', broken)
            broken = copy.deepcopy(rows)
            cancel = next(e for e in summary['events'] if e.get('command') == 'cancel_navigation')['ros_s']
            sample = next(r for r in reversed(broken) if r['kind'] == 'command' and r['ros_s'] > cancel+1.5)
            sample['v'] = .03
            negative('late_motion_after_cancel', broken)
        if case.get('expect_conflict'):
            result = copy.deepcopy(summary); result['checks']['conflict_exercised'] = False
            result['_force_conflict_false'] = True
            negative('conflict_not_triggered', rows, result)
        if stop_profile:
            settled=summary['pause_stop_evidence']['settled_source_s']
            for name,kind,key,value in (
                    ('motion_resumed_while_occupied','motion','v',.025),
                    ('occupied_command_leak','command','v',.03),
                    ('occupied_hold_release','constraint','hold',False),
                    ('occupied_clearance_lost','geometry','clearance_lower_bound_m',0.)):
                broken=copy.deepcopy(rows)
                sample=next(r for r in broken if r['kind']==kind and settled+.5<r['ros_s']<resume)
                sample[key]=value;negative(name,broken)
    report['passed'] = all(r['scenario_passed'] for r in report['replays']) and all(
        r['rejected'] for r in report['negative_checks'])
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(passed=report['passed'], replays=len(report['replays']),
                         negative_checks=len(report['negative_checks']))))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
