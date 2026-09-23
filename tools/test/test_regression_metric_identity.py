import csv
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'social_navigation'))
import compare_empty_baseline as comparison
from report_regression import summarize


class RegressionMetricIdentityTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.run = self.root / 'run'
        self.episode = self.run / 'episode'
        self.nav = self.episode / 'navigation'
        self.nav.mkdir(parents=True)
        self.plans = [dict(cycle=1, goal_index=0, revision=1, frame='map', poses=[[0, 0], [1, 0]]),
                      dict(cycle=1, goal_index=0, revision=2, frame='map', poses=[[2, 0], [2, 1]]),
                      dict(cycle=2, goal_index=0, revision=1, frame='map', poses=[[0, 5], [1, 5]])]
        self.rows = [self.row(1, 1, 1, 2, 0, 1), self.row(1, 2, 2, 2.1, .5, .1),
                     self.row(2, 1, 3, .5, 5.2, .2)]
        self.write()

    def row(self, cycle, revision, stamp, x, y, cross):
        return dict(cycle=cycle, goal_index=0, revision=revision, pose_stamp_s=stamp,
                    velocity_stamp_s=stamp, sim_s=stamp, x=x, y=y, phase='FOLLOW',
                    cross_track_m=cross, heading_error_deg=0, speed=.1, wz=0,
                    reference_curvature=0, progress_m=0)

    def write(self):
        (self.nav / 'plans.jsonl').write_text(''.join(json.dumps(p)+'\n' for p in self.plans))
        with (self.nav / 'samples.csv').open('w') as f:
            writer = csv.DictWriter(f, fieldnames=self.rows[0])
            writer.writeheader(); writer.writerows(self.rows)
        (self.nav / 'results.jsonl').write_text(''.join(json.dumps(dict(cycle=c, index=0,
            goal=[1, 0, 0], passed=True, action_status=4, xy_m=.001, yaw_deg=.01))+'\n' for c in (1, 2)))
        (self.root / 'summary.json').write_text(json.dumps(dict(status='completed', jobs=[1], results=[
            dict(directory=str(self.run), case='endpoint', policy='off', verdict='FAIL')])) )

    def test_endpoint_residual_is_not_lateral_and_replans_use_exact_revision(self):
        result = summarize([self.root])
        first, second = result['episodes'][0]['goals']
        self.assertAlmostEqual(first['follow_lateral_p95_m'], .1)
        self.assertAlmostEqual(first['follow_cross_track_p95_m'], 1)
        self.assertAlmostEqual(second['follow_lateral_p95_m'], .2)
        self.assertEqual(second['cycle'], 2)
        self.assertEqual(result['completed_episode_counts'], {'FAIL': 1})
        self.assertEqual(result['ordinary_goals']['reached_in_passing_episodes'], 0)

    def test_missing_plan_does_not_substitute_other_revision_or_hide_coverage_gap(self):
        self.plans.pop(1); self.write()
        first = summarize([self.root])['episodes'][0]['goals'][0]
        self.assertIsNone(first['follow_lateral_p95_m'])
        self.assertEqual(first['lateral_measurement']['missing_plan_samples'], 1)

    def test_wrong_frame_and_conflicting_plan_identity_fail_explicitly(self):
        self.plans[0]['frame'] = 'odom'; self.write()
        with self.assertRaisesRegex(ValueError, 'frame'):
            comparison.plan_paths(self.episode)
        self.plans[0]['frame'] = 'map'
        self.plans.append(dict(self.plans[0], poses=[[0, 10], [1, 10]])); self.write()
        with self.assertRaisesRegex(ValueError, 'Conflicting'):
            comparison.plan_paths(self.episode)

    def test_duplicate_batch_reference_excluded_and_terminal_phase_not_scored(self):
        self.rows.append(dict(self.rows[-1], phase='REFINE', pose_stamp_s=4,
                              velocity_stamp_s=4, x=100, y=100, cross_track_m=100))
        self.write()
        result = summarize([self.root, self.root])
        self.assertEqual(len(result['episodes']), 1)
        self.assertEqual(len(result['duplicate_references_excluded']), 1)
        self.assertAlmostEqual(result['episodes'][0]['goals'][1]['follow_lateral_p95_m'], .2)

    def test_lateral_uses_all_follow_poses_without_legacy_cross_track(self):
        self.rows[1]['cross_track_m'] = None
        self.write()
        result = comparison.lateral_metrics(self.rows[:2], comparison.plan_paths(self.episode))
        self.assertEqual(result['expected_samples'], 2)
        self.assertEqual(result['matched_samples'], 2)
        self.assertTrue(result['complete'])
        self.assertAlmostEqual(result['distribution']['p95'], .1)

    def test_missing_plan_for_follow_without_cross_track_is_unavailable(self):
        self.rows[1]['cross_track_m'] = None
        self.plans.pop(1); self.write()
        result = comparison.lateral_metrics(self.rows[:2], comparison.plan_paths(self.episode))
        self.assertFalse(result['complete'])
        self.assertEqual(result['expected_samples'], 2)
        self.assertEqual(result['missing_plan_samples'], 1)
        self.assertIsNone(result['distribution']['p95'])

    def test_missing_pose_for_follow_without_cross_track_is_unavailable(self):
        self.rows[1].update(cross_track_m=None, x=None)
        self.write()
        result = comparison.lateral_metrics(self.rows[:2], comparison.plan_paths(self.episode))
        self.assertFalse(result['complete'])
        self.assertEqual(result['expected_samples'], 2)
        self.assertEqual(result['invalid_pose_samples'], 1)
        self.assertIsNone(result['distribution']['p95'])

    def comparison_episode(self, name='fixture', policy='off'):
        run = self.root / name
        episode = run / 'episode'; nav = episode / 'navigation'
        nav.mkdir(parents=True)
        route = [[1, 0, 0], [0, 0, 0]]
        (run / 'scene.yaml').write_text('same empty scene\n')
        (run / 'runtime_manifest.json').write_text(json.dumps(dict(libraries={
            '/fixture/libastribot_s1_path_tracking.so': 'same-controller'})))
        (episode / 'case.json').write_text(json.dumps(dict(people=[], route=route, scene='fixture')))
        (episode / 'summary.json').write_text(json.dumps(dict(expected_policy=policy,
            all_goals_passed=True, scenario_passed=True, replan_events=[])))
        parameters = dict(gain=1, navigation_policy_enabled=policy != 'off')
        (nav / 'metadata.json').write_text(json.dumps(dict(controller_parameters=parameters,
            controller_parameters_all=dict(parameters, navigation_policy_stage='off' if policy=='off' else 'p2'))))
        results = [dict(cycle=1, index=i, goal=goal, passed=True, action_status=4)
                   for i, goal in enumerate(route)]
        (nav / 'results.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in results))
        plans = [dict(cycle=1, goal_index=i, revision=1, frame='map', poses=[[0, 0], [1, 0]])
                 for i in range(2)]
        (nav / 'plans.jsonl').write_text(''.join(json.dumps(p)+'\n' for p in plans))
        rows = [dict(self.row(1, 1, 1+i+j*.05, .5, .1*(i+1), .1*(i+1)),
                     goal_index=i, heading_error_deg=10*(i+1), policy_hold=False, policy_reason='CLEAR')
                for i in range(2) for j in range(5)]
        with (nav / 'samples.csv').open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0])
            writer.writeheader(); writer.writerows(rows)
        return episode, results

    def test_duplicate_goal_result_cannot_replace_return_leg(self):
        episode, results = self.comparison_episode()
        (episode / 'navigation/results.jsonl').write_text((json.dumps(results[0])+'\n')*2)
        with self.assertRaisesRegex(ValueError, 'Duplicate result identity'):
            comparison.load(episode)

    def test_result_goal_must_match_requested_index(self):
        episode, results = self.comparison_episode()
        results[1]['goal'] = results[0]['goal']
        (episode / 'navigation/results.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in results))
        with self.assertRaisesRegex(ValueError, 'Result target'):
            comparison.load(episode)

    def test_missing_or_wrong_cycle_result_is_rejected(self):
        for name, change in (('missing', lambda results:results[:1]),
                             ('wrong_cycle', lambda results:[results[0], dict(results[1], cycle=2)])):
            with self.subTest(name=name):
                episode, results = self.comparison_episode(name)
                (episode / 'navigation/results.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in change(results)))
                if name == 'wrong_cycle':
                    sample_path = episode / 'navigation/samples.csv'
                    with sample_path.open() as stream:
                        rows = list(csv.DictReader(stream))
                    for row in rows:
                        if row['goal_index'] == '1': row['cycle'] = '2'
                    with sample_path.open('w') as stream:
                        writer = csv.DictWriter(stream, fieldnames=rows[0])
                        writer.writeheader(); writer.writerows(rows)
                with self.assertRaisesRegex(ValueError, 'Result identities'):
                    comparison.load(episode)

    def test_comparison_joins_goal_identity_when_result_lines_reordered(self):
        off = [self.comparison_episode('off_'+str(i))[0] for i in range(3)]
        on = []
        for i in range(3):
            episode, results = self.comparison_episode('on_'+str(i), 'h2')
            (episode / 'navigation/results.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in reversed(results)))
            on.append(episode)
        output = self.root / 'comparison.json'
        argv = ['compare', '--off', *map(str, off), '--on', *map(str, on), '--output', str(output)]
        with patch.object(sys, 'argv', argv), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit) as result:
                comparison.main()
        self.assertEqual(result.exception.code, 0)
        report = json.loads(output.read_text())
        self.assertTrue(report['passed'])
        self.assertEqual(report['comparisons'][0]['off'], [.1]*3)
        self.assertEqual(report['comparisons'][0]['on'], [.1]*3)
        self.assertEqual(report['comparisons'][3]['on'], [.2]*3)


if __name__ == '__main__':
    unittest.main()
