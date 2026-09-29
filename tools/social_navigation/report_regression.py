#!/usr/bin/env python3
"""Summarize explicit regression batches without turning partial runs into acceptance."""
import argparse
import hashlib
import json
from pathlib import Path

from compare_empty_baseline import read_samples, source_time_samples, plan_paths, lateral_metrics
from run_waypoint_route import metrics, measured_motion


def read(path):
    return json.loads(path.read_text()) if path.is_file() else {}


def summarize(batches):
    episodes = []; seen = set(); duplicate_references = []; batch_status = []
    for batch in batches:
        summary = read(batch/'summary.json')
        batch_status.append(dict(directory=str(batch), status=summary.get('status', 'not_started'),
            planned_jobs=len(summary.get('jobs', [])), completed_results=len(summary.get('results', []))))
        for result in summary.get('results', []):
            directory = Path(result['directory']).resolve()
            if directory in seen:
                duplicate_references.append(str(directory)); continue
            seen.add(directory)
            episode = directory/'episode'; report = read(episode/'summary.json')
            runtime = read(directory/'runtime_manifest.json')
            libraries = {Path(p).name: sha for p, sha in runtime.get('libraries', {}).items()
                         if Path(p).name in ('libastribot_s1_path_tracking.so', 'libastribot_s1_mppi_critics.so')}
            metadata = read(episode/'navigation/metadata.json')
            parameters = metadata.get('controller_parameters_all')
            signature = hashlib.sha256(json.dumps(dict(libraries=libraries, parameters=parameters),
                sort_keys=True).encode()).hexdigest() if libraries and parameters else None
            goals = []
            nav_file = episode/'navigation/results.jsonl'
            if nav_file.is_file():
                samples = read_samples(episode/'navigation/samples.csv')
                paths = plan_paths(episode)
                for line in nav_file.read_text().splitlines():
                    nav = json.loads(line)
                    selected = [s for s in samples if (s['cycle'], s['goal_index']) == (nav['cycle'], nav['index'])]
                    sampled = source_time_samples(selected) if selected else []
                    quality = metrics(sampled) if selected else {}
                    lateral = lateral_metrics(sampled, paths)
                    actual = measured_motion(selected) if selected else {}
                    goals.append(dict(cycle=nav['cycle'], index=nav['index'], target=nav['goal'], passed=nav['passed'],
                        action_status=nav['action_status'], xy_m=nav['xy_m'], yaw_deg=nav['yaw_deg'],
                        follow_samples=quality.get('follow_samples', 0),
                        phase_coverage=quality.get('phase_coverage'),
                        unique_motion_samples=actual.get('unique_samples', 0),
                        follow_cross_track_p95_m=quality.get('cross_track_m', {}).get('p95'),
                        follow_lateral_p95_m=lateral['distribution']['p95'],
                        lateral_measurement=lateral,
                        follow_heading_p95_deg=quality.get('front_heading_error_deg', {}).get('p95'),
                        follow_actual_jerk_p95_m_s3=actual.get('jerk_m_s3', {}).get('p95')))
            expected_cancel = read(episode/'case.json').get('expected_task_outcome') == 'CANCELED'
            visual = read(directory/'paired_gazebo_rviz_review.json') or read(directory/'visual_review.json')
            episodes.append(dict(directory=str(directory), case=result['case'], job=result.get('job'),
                policy=result['policy'], verdict=result['verdict'], failed_checks=report.get('failed_checks', []),
                error=report.get('error', result.get('error')), cleanup_complete=result.get('cleanup_complete', False),
                expected_cancellation=expected_cancel, goals=goals,
                visual_review=visual.get('status', 'PENDING'),
                controller_libraries=libraries, controller_configuration_sha256=signature))
    ordinary = [e for e in episodes if not e['expected_cancellation']]
    goals = [g for e in ordinary for g in e['goals']]
    arrivals = [g for g in goals if g['passed']]
    return dict(stage_acceptance='PENDING', batches=batch_status, episodes=episodes,
        duplicate_references_excluded=duplicate_references,
        completed_episode_counts={v:sum(e['verdict'] == v for e in episodes)
                                  for v in sorted({e['verdict'] for e in episodes})},
        ordinary_goals=dict(recorded=len(goals), reached=len(arrivals),
            reached_in_passing_episodes=sum(g['passed'] for e in ordinary if e['verdict'] == 'PASS' for g in e['goals']),
            max_successful_xy_m=max((g['xy_m'] for g in arrivals), default=None),
            max_successful_yaw_deg=max((g['yaw_deg'] for g in arrivals), default=None)),
        canceled_episodes=sum(e['expected_cancellation'] for e in episodes),
        measurement=dict(position_source='simulation truth',
            path_quality='FOLLOW only; per-goal, at most 20 Hz by pose source timestamp',
            cross_track='historical distance to clamped segment, including endpoint longitudinal residual',
            lateral='normal component; exact cycle/goal/revision plan; unavailable if coverage incomplete',
            actual_jerk='original unique odometry source timestamps', elapsed_time_scored=False),
        boundary='Only completed recorded results. Unrecorded goals and canceled tasks are excluded from arrival counts; completed goals in a failed episode retain that episode failure. This report does not grant a stage or replace repeated A/B comparisons.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batches', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = summarize([p.resolve() for p in args.batches])
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ('stage_acceptance', 'batches', 'completed_episode_counts',
                                          'ordinary_goals', 'canceled_episodes')}, indent=2))


if __name__ == '__main__': main()
