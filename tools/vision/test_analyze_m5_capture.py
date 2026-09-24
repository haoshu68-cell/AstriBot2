"""Offline evidence semantics; no ROS or simulation is started."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from analyze_m5_capture import analyze


class AnalyzeM5CaptureTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)

    def capture(self, events, topics=None, **overrides):
        manifest = dict(started_monotonic_ns=10_000_000_000,
                        ended_monotonic_ns=11_000_000_000, completed=True,
                        interrupted=False, writer_dropped=0, warmup_sec=0,
                        topics=topics or [dict(topic='/image', kind='image', type='Image',
                                              required=True, gap_budget_sec=.25)])
        manifest.update(overrides)
        (self.folder / 'manifest.json').write_text(json.dumps(manifest))
        (self.folder / 'events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in events))
        return analyze(self.folder)

    @staticmethod
    def event(receive, source=1, topic='/image', kind='image', ros=2, data=None):
        return dict(kind=kind, topic=topic,
                    receive_monotonic_ns=10_000_000_000 + round(receive * 1e9),
                    receive_wall_ns=1_800_000_000_000_000_000 + round(receive * 1e9),
                    ros_now_ns=round(ros * 1e9),
                    source_ns=round(source * 1e9) if source is not None else None,
                    data=data or {})

    def test_missing_required_stream_is_not_a_pass(self):
        result = self.capture([])
        self.assertEqual(result['sampling_criteria_status'], 'FAIL')
        self.assertEqual(result['streams']['/image']['presence'], 'MISSING')
        self.assertIsNone(result['streams']['/image']['initial_silence_wall_sec'])

    def test_null_stereo_budget_does_not_invent_250ms_failure(self):
        result = self.capture([self.event(.1), self.event(.9)], topics=[
            dict(topic='/image', kind='image', type='Image', required=True, gap_budget_sec=None)])
        self.assertEqual(result['sampling_criteria_status'], 'PASS')
        self.assertEqual(result['streams']['/image']['gap_budget_status'], 'NOT_EVALUATED')
        self.assertIsNone(result['streams']['/image']['receive_gaps_over_budget'])

    def test_percentile_is_linear_and_source_nonpositive_intervals_are_separate(self):
        rows = [self.event(t, stamp) for t, stamp in [(0, 2), (.1, 2), (.3, 1), (1, 3)]]
        stream = self.capture(rows)['streams']['/image']
        self.assertAlmostEqual(stream['receive_interval_wall_sec']['p99'], .69)
        self.assertEqual(stream['receive_interval_wall_sec']['percentile_method'], 'linear')
        self.assertEqual(stream['duplicate_source_intervals'], 1)
        self.assertEqual(stream['source_regressions'], 1)

    def test_initial_and_terminal_silence_are_checked_without_recovery_event(self):
        stream = self.capture([self.event(.3), self.event(.5)])['streams']['/image']
        self.assertAlmostEqual(stream['initial_silence_wall_sec'], .3)
        self.assertAlmostEqual(stream['tail_silence_wall_sec'], .5)
        self.assertEqual(stream['receive_gaps_over_budget'], 0)
        self.assertEqual(stream['gap_budget_status'], 'FAIL')

    def test_warmup_excludes_initial_intervals_but_not_last_silence(self):
        rows = [self.event(t) for t in [0, .9, 1, 1.2, 1.4, 1.6, 1.8, 2]]
        result = self.capture(rows, warmup_sec=1, ended_monotonic_ns=12_000_000_000)
        stream = result['streams']['/image']
        self.assertEqual(stream['samples_full_window'], 8)
        self.assertEqual(stream['samples'], 6)
        self.assertAlmostEqual(stream['receive_interval_wall_sec']['max'], .2)
        self.assertEqual(result['sampling_criteria_status'], 'PASS')

    def test_no_postwarmup_data_is_not_a_pass(self):
        result = self.capture([self.event(.1)], warmup_sec=.5)
        self.assertEqual(result['sampling_criteria_status'], 'FAIL')

    def test_unknown_ros_clock_and_future_samples_are_not_clamped(self):
        rows = [self.event(.1, source=5, ros=0), self.event(.2, source=3, ros=2),
                self.event(.3, source=1, ros=2)]
        stream = self.capture(rows)['streams']['/image']
        self.assertEqual(stream['source_age_unavailable_samples'], 1)
        self.assertEqual(stream['source_age_negative_future_samples'], 1)
        self.assertEqual(stream['source_age_ros_sec']['min'], -1)

    def test_repeated_capture_heartbeat_does_not_reset_steady_age(self):
        rows = [self.event(t, source=10, ros=10, kind='projection_health', data={
            'capture_stamp': {'sec': 10, 'nanosec': 0}, 'processing_epoch': 'one',
            'valid': True, 'pending_depth': 1}) for t in [0, .2, .4, .6, .8, 1]]
        stream = self.capture(rows, topics=[dict(topic='/image', kind='projection_health',
            type='ProjectionHealth', required=True, gap_budget_sec=.25)])['streams']['/image']
        self.assertEqual(stream['health']['capture_steady_age_sec']['max'], 1)
        self.assertEqual(stream['health']['capture_age_ros_sec']['max'], 0)
        self.assertEqual(stream['health']['pending_depth']['max'], 1)
        self.assertEqual(stream['health']['dds_queue_occupancy'], 'NOT_MEASURED')

    def test_health_capture_age_uses_capture_stamp_and_preserves_future_samples(self):
        event = self.event(.5, source=2, ros=2, kind='raw_health', data={
            'capture_stamp': {'sec': 3, 'nanosec': 0}, 'source_epoch': 'one', 'valid': True})
        stream = self.capture([event], topics=[dict(topic='/image', kind='raw_health',
            type='CameraHealth', required=True, gap_budget_sec=None)])['streams']['/image']
        self.assertEqual(stream['source_age_ros_sec']['max'], 0)
        self.assertEqual(stream['health']['capture_age_ros_sec']['min'], -1)
        self.assertEqual(stream['health']['capture_age_negative_future_samples'], 1)

    def test_capture_age_retains_prewarmup_first_observation(self):
        rows = [self.event(t, kind='raw_health', data={
            'capture_stamp': {'sec': 1, 'nanosec': 0}, 'source_epoch': 'one', 'valid': True})
            for t in [0, .5, 1]]
        stream = self.capture(rows, warmup_sec=.5, topics=[dict(topic='/image', kind='raw_health',
            type='CameraHealth', required=True, gap_budget_sec=None)])['streams']['/image']
        self.assertEqual(stream['health']['capture_steady_age_sec']['min'], .5)

    def test_epoch_change_starts_new_capture_identity(self):
        rows = [self.event(t, kind='raw_health', data={
            'capture_stamp': {'sec': 1, 'nanosec': 0}, 'source_epoch': epoch, 'valid': valid})
            for t, epoch, valid in [(0, 'one', True), (.5, 'one', False), (1, 'two', True)]]
        health = self.capture(rows, topics=[dict(topic='/image', kind='raw_health', type='CameraHealth',
            required=True, gap_budget_sec=None)])['streams']['/image']['health']
        self.assertEqual(health['capture_steady_age_sec']['max'], .5)
        self.assertEqual(health['invalid_samples'], 1)
        self.assertEqual(health['epochs'], ['one', 'two'])

    def test_incomplete_or_dropped_capture_cannot_pass(self):
        rows = [self.event(t / 5) for t in range(6)]
        for options in [dict(completed=False), dict(interrupted=True), dict(writer_dropped=1)]:
            with self.subTest(options=options):
                result = self.capture(rows, **options)
                self.assertEqual(result['observation_status'], 'INCOMPLETE')
                self.assertNotEqual(result['sampling_criteria_status'], 'PASS')

    def test_status_stage_is_observed_context_not_actual_motion(self):
        rows = [self.event(.1, topic='/task', kind='status', source=None,
                           data={'data': json.dumps({'stage': 'TRANSPORT'})})]
        result = self.capture(rows, topics=[dict(topic='/task', kind='status', type='String',
            required=False, gap_budget_sec=None)])
        self.assertEqual(result['phase_observations'][0]['stage'], 'TRANSPORT')
        for item in ['actual_motion', 'control_latency', 'control_causality', 'roi_coverage',
                     'three_dimensional_coverage', 'graph_ownership']:
            self.assertEqual(result['not_measured'][item], 'NOT_MEASURED')

    def test_clock_rtf_uses_observed_receive_span_and_counts_frozen_intervals(self):
        rows = [self.event(t, source=source, topic='/clock', kind='clock')
                for t, source in [(0, 10), (.25, 10), (.5, 10.2)]]
        result = self.capture(rows, topics=[dict(topic='/clock', kind='clock', type='Clock',
            required=True, gap_budget_sec=None)])
        clock = result['streams']['/clock']['clock']
        self.assertAlmostEqual(clock['source_advance_sec'], .2)
        self.assertAlmostEqual(clock['receive_steady_span_sec'], .5)
        self.assertAlmostEqual(clock['observed_rtf'], .4)
        self.assertEqual(clock['frozen_source_intervals'], 1)

    def test_clock_regression_does_not_produce_misleading_rtf(self):
        rows = [self.event(t, source=source, topic='/clock', kind='clock')
                for t, source in [(0, 10), (.5, 1)]]
        stream = self.capture(rows, topics=[dict(topic='/clock', kind='clock', type='Clock',
            required=True, gap_budget_sec=None)])['streams']['/clock']
        self.assertEqual(stream['source_regressions'], 1)
        self.assertEqual(stream['clock']['source_advance_sec'], -9)
        self.assertIsNone(stream['clock']['observed_rtf'])
        self.assertEqual(stream['clock']['rtf_status'], 'SOURCE_REGRESSION')

    def test_structured_and_unknown_status_messages_are_preserved_without_guessing(self):
        payloads = [{'stage': 'LIFT'}, {'phase': 'FOLLOW'}, {'data': 'alive'},
                    {'data': '{"mode":"active"}'}, {'arbitrary': 7}]
        rows = [self.event(i * .1, topic='/task', kind='status', source=None, data=data)
                for i, data in enumerate(payloads)]
        result = self.capture(rows, topics=[dict(topic='/task', kind='status', type='Status',
            required=False, gap_budget_sec=None)])
        self.assertEqual(result['phase_observations'][0]['stage'], 'LIFT')
        self.assertEqual(result['phase_observations'][1]['phase'], 'FOLLOW')
        status = result['status_observations']
        self.assertEqual([item['data'] for item in status], payloads)
        self.assertEqual(status[2]['parse_status'], 'UNPARSED')
        self.assertEqual(status[2]['reason'], 'STRING_IS_NOT_JSON')
        self.assertEqual(status[3]['reason'], 'NO_STAGE_OR_PHASE_FIELD')
        self.assertEqual(status[4]['parse_status'], 'UNPARSED')

    def test_health_state_and_reason_counts_keep_stale_observations(self):
        rows = [self.event(i * .1, kind='raw_health', data={
            'capture_stamp': {'sec': 1, 'nanosec': 0}, 'source_epoch': 'one',
            'valid': i == 0, 'state': 'OK' if i == 0 else 'STALE',
            'reason_code': 'CAMERA_READY' if i == 0 else 'CAMERA_STALE'}) for i in range(3)]
        health = self.capture(rows, topics=[dict(topic='/image', kind='raw_health', type='CameraHealth',
            required=True, gap_budget_sec=None)])['streams']['/image']['health']
        self.assertEqual(health['state_counts'], {'OK': 1, 'STALE': 2})
        self.assertEqual(health['reason_counts'], {'CAMERA_READY': 1, 'CAMERA_STALE': 2})

    def test_cli_refuses_to_overwrite_existing_analysis(self):
        self.capture([])
        output = self.folder / 'analysis.json'
        output.write_text('preserve')
        run = subprocess.run([sys.executable, str(Path(__file__).with_name('analyze_m5_capture.py')),
                              '--capture', str(self.folder), '--output', str(output)],
                             capture_output=True, text=True)
        self.assertNotEqual(run.returncode, 0)
        self.assertEqual(output.read_text(), 'preserve')


if __name__ == '__main__':
    unittest.main()
