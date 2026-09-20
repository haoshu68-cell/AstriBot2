#!/usr/bin/env python3
"""Compare repeated empty-scene social off/on navigation results."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from statistics import median
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from run_waypoint_route import metrics, measured_motion


def read_samples(path):
    strings={'phase','policy_reason'}
    integers={'cycle','goal_index','revision','phase_received_ros_ns','phase_age_ns'}
    def value(key,text):
        if key in strings:return text
        if not text:return None
        if key=='policy_hold':return text=='True'
        number=int(text) if key in integers else float(text)
        if not math.isfinite(number):raise ValueError('Nonfinite metric input: '+key)
        return number
    with path.open() as source:
        return [{k:value(k,v) for k,v in row.items()} for row in csv.DictReader(source)]


def source_time_samples(rows,period=.05):
    """One pose per physical-time bin; polling a slow simulator adds no weight."""
    output=[];last_stamp=None;last_bin=None
    for row in rows:
        stamp=row.get('pose_stamp_s',row.get('velocity_stamp_s'))
        if stamp is None or not math.isfinite(stamp) or stamp<=0:
            raise ValueError('Source-time pose/odom sample required for comparison')
        if last_stamp is not None and stamp<last_stamp:
            raise ValueError('Source clock reversed in metric evidence')
        bucket=math.floor(stamp/period+1e-8)
        sample=dict(row,sim_s=stamp)
        if bucket==last_bin:output[-1]=sample
        else:output.append(sample)
        last_stamp,last_bin=stamp,bucket
    return output


def load(directory):
    directory=Path(directory)
    summary=json.loads((directory/'summary.json').read_text())
    case=json.loads((directory/'case.json').read_text())
    if case['people'] or case.get('input_fault'):
        raise ValueError('Only empty, non-fault episodes are comparable')
    results=[json.loads(line) for line in (directory/'navigation/results.jsonl').read_text().splitlines()]
    metadata=json.loads((directory/'navigation/metadata.json').read_text())
    samples=read_samples(directory/'navigation/samples.csv')
    for result in results:
        rows=[r for r in samples if (r['cycle'],r['goal_index'])==(result['cycle'],result['index'])]
        if not rows:raise ValueError('Missing per-goal raw samples')
        result['metrics']=metrics(source_time_samples(rows))
        # Keep actual acceleration/jerk at the original unique odometry rate.
        result['metrics']['measured_motion']=measured_motion(rows)
    return directory,summary,case,results,metadata


def comparison_identity(run):
    directory,_,case,_,metadata=run
    manifest=directory.parent/'runtime_manifest.json'
    libraries=json.loads(manifest.read_text()).get('libraries',{}) if manifest.is_file() else {}
    # Paths may differ between frozen installations; loaded content must match.
    controller={Path(p).name:sha for p,sha in libraries.items()
                if Path(p).name in ('libastribot_s1_path_tracking.so','libastribot_s1_mppi_critics.so')}
    parameters=metadata.get('controller_parameters_all')
    scene=directory.parent/'scene.yaml'
    return dict(initial_yaw_rad=case.get('initial_yaw_rad',0.),scene=case['scene'],
        scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest() if scene.is_file() else None,
        parameters=None if parameters is None else {k:v for k,v in parameters.items()
            if k not in ('navigation_policy_enabled','navigation_policy_stage')},
        policy_configuration=None if parameters is None else
            [parameters.get('navigation_policy_enabled'),parameters.get('navigation_policy_stage')],
        controller_libraries=controller)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--off',nargs='+',required=True)
    parser.add_argument('--on',nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();off=[load(p) for p in args.off];on=[load(p) for p in args.on]
    runs=off+on;reference=runs[0][2]['route']
    through=bool(runs[0][2].get('through_poses'));targets=[reference[-1]] if through else reference
    def motion_parameters(run):
        return {k:v for k,v in run[4]['controller_parameters'].items() if k!='navigation_policy_enabled'}
    def midroute_holds(run):
        moving=set();holds=[]
        with (run[0]/'navigation/samples.csv').open() as source:
            for sample in csv.DictReader(source):
                goal=(sample['cycle'],sample['goal_index'])
                if float(sample['speed'])>.02:moving.add(goal)
                if goal in moving and sample['policy_hold']=='True':
                    holds.append({'goal':goal,'sim_s':float(sample['sim_s']),
                                  'reason':sample['policy_reason']})
        return holds
    holds={str(r[0]):midroute_holds(r) for r in on}
    checks={
        'three_pairs':len(off)==len(on) and len(off)>=3,
        'distinct_episodes':len({r[0].resolve() for r in runs})==len(runs),
        'same_route':all(r[2]['route']==reference for r in runs),
        'same_navigation_mode':all(bool(r[2].get('through_poses'))==through for r in runs),
        'same_recorded_motion_parameters':all(motion_parameters(r)==motion_parameters(runs[0]) for r in runs),
        'expected_modes':all(r[1].get('expected_policy')=='off' for r in off) and all(r[1].get('expected_policy')=='h2' for r in on),
        'all_arrivals':all(r[1]['all_goals_passed'] and len(r[3])==len(targets) for r in runs),
        'all_episode_checks':all(r[1].get('scenario_passed',False) for r in runs),
        'event_only_planning':all(e['reason'] in ('initial_goal','new_goal') for r in runs for e in r[1]['replan_events']),
        'no_extra_midroute_hold':not any(holds.values()),
    }
    identities=[comparison_identity(r) for r in runs]
    checks['same_initial_heading_and_scene']=all(r['scene_sha256'] and
        (r['initial_yaw_rad'],r['scene_sha256'])==
        (identities[0]['initial_yaw_rad'],identities[0]['scene_sha256']) for r in identities)
    checks['complete_controller_evidence']=all(r['parameters'] and
        'libastribot_s1_path_tracking.so' in r['controller_libraries'] for r in identities)
    checks['expected_controller_policy_switches']=all(r['policy_configuration']==expected
        for group,expected in ((identities[:len(off)],[False,'off']),(identities[len(off):],[True,'p2']))
        for r in group)
    checks['same_complete_controller_configuration']=checks['complete_controller_evidence'] and all(
        (r['parameters'],r['controller_libraries'])==
        (identities[0]['parameters'],identities[0]['controller_libraries']) for r in identities)
    criteria=[('lateral_p95_m',('cross_track_m','p95'),.005,.10),
              ('heading_p95_deg',('front_heading_error_deg','p95'),1.,.10),
              ('actual_jerk_p95_m_s3',('measured_motion','jerk_m_s3','p95'),.1,.15)]
    comparisons=[]
    if checks['all_arrivals'] and checks['same_route']:
        for index,goal in enumerate(targets):
            for name,path,absolute,relative in criteria:
                def values(group):
                    output=[]
                    for run in group:
                        value=run[3][index]['metrics']
                        for key in path:value=value[key]
                        output.append(value)
                    return output
                before,after=values(off),values(on)
                valid=all(isinstance(v,(int,float)) and math.isfinite(v) for v in before+after)
                limit=median(before)+max(absolute,median(before)*relative) if valid else None
                passed=valid and median(after)<=limit
                comparisons.append(dict(goal=goal,metric=name,off=before,on=after,
                    off_median=median(before) if valid else None,on_median=median(after) if valid else None,
                    limit=limit,passed=passed,measurement_available=valid))
    checks['control_regression']=bool(comparisons) and all(r['passed'] for r in comparisons)
    report=dict(passed=all(checks.values()),checks=checks,comparisons=comparisons,
        off=args.off,on=args.on,midroute_holds=holds,comparison_identity=identities,
        metric_sampling={'pose_max_hz':20,'source':'pose_stamp_s; older CSV uses velocity_stamp_s proxy',
                         'actual_motion':'original unique odometry acquisition stamps',
                         'original_reports':'preserved; both groups recomputed from raw CSV'},
        boundary='Simulation truth and FOLLOW-only control metrics; total duration is not scored')
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':main()
