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
from run_waypoint_route import distribution, metrics, measured_motion


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


def path_normal_error(path, x, y):
    """Return signed normal distance to the nearest finite path segment.

    ``cross_track_m`` is intentionally kept unchanged for historical reports:
    it is the distance to the clamped segment and therefore includes a
    longitudinal residual when the pose is beyond an endpoint.  The A/B
    quality gate needs the path-normal component so a 180-degree start
    alignment cannot turn endpoint overshoot into a false lateral regression.
    """
    best = None
    for a, b in zip(path, path[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length < 1e-8:
            continue
        f = max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / (length * length)))
        px, py = a[0] + f * dx, a[1] + f * dy
        distance = math.hypot(x - px, y - py)
        normal = (dx * (y - a[1]) - dy * (x - a[0])) / length
        if best is None or distance < best[0]:
            best = (distance, normal)
    return None if best is None else best[1]


def plan_paths(directory):
    plans = {}
    path = Path(directory) / 'navigation' / 'plans.jsonl'
    if not path.is_file():
        return plans
    for line in path.read_text().splitlines():
        record = json.loads(line)
        if record.get('frame') != 'map':
            raise ValueError('Plan frame must match map-frame pose samples')
        key = (int(record['cycle']), int(record['goal_index']), int(record['revision']))
        poses = [(float(p[0]), float(p[1])) for p in record.get('poses', [])]
        if any(not math.isfinite(v) for p in poses for v in p):
            raise ValueError('Nonfinite plan coordinate')
        if key in plans and plans[key] != poses:
            raise ValueError('Conflicting plan identity: ' + str(key))
        plans[key] = poses
    return plans


def lateral_metrics(rows, paths):
    """Use only each sample's recorded plan; incomplete coverage is unavailable."""
    values = []; missing = invalid = invalid_pose = 0
    follow = [r for r in rows if r['phase'] == 'FOLLOW']
    for row in follow:
        path = paths.get((row['cycle'], row['goal_index'], row['revision']))
        if path is None:
            missing += 1
            continue
        if any(not isinstance(row.get(key), (int, float)) or not math.isfinite(row[key])
               for key in ('x', 'y')):
            invalid_pose += 1
            continue
        value = path_normal_error(path, row['x'], row['y'])
        if value is None or not math.isfinite(value):
            invalid += 1
        else:
            values.append(value)
    complete = bool(follow) and not missing and not invalid and not invalid_pose
    return dict(distribution=distribution(values) if complete else distribution([]),
                expected_samples=len(follow), matched_samples=len(values),
                missing_plan_samples=missing, invalid_path_samples=invalid,
                invalid_pose_samples=invalid_pose, complete=complete,
                definition='signed normal component of nearest finite segment; endpoint longitudinal residual excluded',
                association='cycle, goal_index, revision; map frame')


def alignment_diagnostics(rows, path=None):
    """Summarize the initial path-heading alignment without changing the gate.

    The empty-scene return leg starts almost 180 degrees from the planned
    direction.  Keep its signed command and pose drift visible so a quality
    regression can be attributed to the alignment transient instead of being
    hidden inside the aggregate FOLLOW metric.
    """
    aligned = [r for r in rows if r.get('phase') == 'ALIGN_START']
    if not aligned:
        return None

    def number(row, key):
        value = row.get(key)
        return None if value in (None, '') else float(value)

    first, last = aligned[0], aligned[-1]
    x0, y0 = number(first, 'x'), number(first, 'y')
    x1, y1 = number(last, 'x'), number(last, 'y')
    commands = [number(row, 'cmd_wz') for row in aligned]
    commands = [value for value in commands if value is not None and abs(value) > 1e-4]
    signs = sorted({1 if value > 0 else -1 for value in commands})
    follow = [r for r in rows if r.get('phase') == 'FOLLOW']
    follow_start = follow[0] if follow else None
    normal = None
    if path and follow_start:
        normal = path_normal_error(path, number(follow_start, 'x'), number(follow_start, 'y'))
    return {
        'samples': len(aligned),
        'start_sim_s': number(first, 'sim_s'),
        'end_sim_s': number(last, 'sim_s'),
        'duration_s': number(last, 'sim_s') - number(first, 'sim_s'),
        'start_pose': [x0, y0, number(first, 'yaw')],
        'end_pose': [x1, y1, number(last, 'yaw')],
        'displacement_m': math.hypot(x1 - x0, y1 - y0),
        'signed_dx_m': x1 - x0,
        'signed_dy_m': y1 - y0,
        'command_wz_min': min(commands) if commands else 0.0,
        'command_wz_max': max(commands) if commands else 0.0,
        'command_signs': signs,
        'follow_start_pose': ([number(follow_start, 'x'), number(follow_start, 'y'),
                               number(follow_start, 'yaw')] if follow_start else None),
        'follow_start_normal_error_m': normal,
        'heading_error_deg_at_follow_start': (number(follow_start, 'heading_error_deg')
                                               if follow_start else None),
    }


def validate_result_identities(results, case):
    """An episode records exactly one cycle, with one result per requested task."""
    route = case['route']
    if not route:
        raise ValueError('Result identities require a nonempty route')
    targets = [route[-1]] if case.get('through_poses') else route
    expected = {(1, index): goal for index, goal in enumerate(targets)}
    seen = set()
    for result in results:
        cycle, index = result.get('cycle'), result.get('index')
        if type(cycle) is not int or type(index) is not int:
            raise ValueError('Result identities must use integer cycle/index')
        identity = (cycle, index)
        if identity in seen:
            raise ValueError('Duplicate result identity: ' + str(identity))
        seen.add(identity)
        if identity not in expected:
            raise ValueError('Result identities do not match the requested episode: ' + str(identity))
        if result.get('goal') != expected[identity]:
            raise ValueError('Result target does not match requested index: ' + str(identity))
    if seen != set(expected):
        raise ValueError('Result identities do not cover every requested goal')


def load(directory):
    directory=Path(directory)
    summary=json.loads((directory/'summary.json').read_text())
    case=json.loads((directory/'case.json').read_text())
    if case['people'] or case.get('input_fault'):
        raise ValueError('Only empty, non-fault episodes are comparable')
    results=[json.loads(line) for line in (directory/'navigation/results.jsonl').read_text().splitlines()]
    validate_result_identities(results, case)
    metadata=json.loads((directory/'navigation/metadata.json').read_text())
    samples=read_samples(directory/'navigation/samples.csv')
    paths=plan_paths(directory)
    for result in results:
        rows=[r for r in samples if (r['cycle'],r['goal_index'])==(result['cycle'],result['index'])]
        if not rows:raise ValueError('Missing per-goal raw samples')
        sampled=source_time_samples(rows)
        result['metrics']=metrics(sampled)
        lateral = lateral_metrics(sampled, paths)
        result['metrics']['lateral_error_m'] = lateral['distribution']
        result['metrics']['lateral_measurement'] = lateral
        result['metrics']['lateral_error_definition'] = lateral['definition']
        first_follow = next((r for r in sampled if r['phase'] == 'FOLLOW'), None)
        normal_path = (paths.get((first_follow['cycle'], first_follow['goal_index'], first_follow['revision']))
                       if first_follow is not None else None)
        result['metrics']['start_alignment'] = alignment_diagnostics(sampled, normal_path)
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
    criteria=[('lateral_normal_p95_m',('lateral_error_m','p95'),0.,.30),
              ('heading_p95_deg',('front_heading_error_deg','p95'),0.,.30),
              ('actual_jerk_p95_m_s3',('measured_motion','jerk_m_s3','p95'),.1,.15)]
    results_by_identity = {run[0]: {(result['cycle'], result['index']): result
                                  for result in run[3]} for run in runs}
    comparisons=[]
    if checks['all_arrivals'] and checks['same_route']:
        for index,goal in enumerate(targets):
            for name,path,absolute,relative in criteria:
                def values(group):
                    output=[]
                    for run in group:
                        value=results_by_identity[run[0]][(1,index)]['metrics']
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
        quality_policy='2026-09-23 user provisional: lateral/heading median P95 increase <=30%; jerk unchanged',
        off=args.off,on=args.on,midroute_holds=holds,comparison_identity=identities,
        metric_sampling={'pose_max_hz':20,'source':'pose_stamp_s; older CSV uses velocity_stamp_s proxy',
                         'actual_motion':'original unique odometry acquisition stamps',
                         'original_reports':'preserved; both groups recomputed from raw CSV'},
        boundary='Simulation truth and FOLLOW-only control metrics; total duration is not scored')
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':main()
