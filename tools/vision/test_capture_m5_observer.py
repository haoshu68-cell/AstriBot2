"""Offline checks: a passive observer must never turn image bytes into evidence."""
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace as N

from capture_m5_observer import main, metadata, validate_topics
from analyze_m5_capture import analyze


class M5CaptureTest(unittest.TestCase):
    def test_image_preserves_capture_time_and_layout_without_payload(self):
        image = N(header=N(stamp=N(sec=12, nanosec=34), frame_id='optical'),
                  width=640, height=320, encoding='32FC1', step=2560,
                  is_bigendian=False, data=b'\x00'*64)
        row = metadata('image', image, None)
        self.assertEqual(row['source_ns'], 12_000_000_034)
        self.assertEqual(row['frame_id'], 'optical')
        self.assertEqual(row['data']['bytes'], 64)
        self.assertNotIn('payload', row['data'])

    def test_unstamped_command_has_no_fabricated_source_time(self):
        row = metadata('twist', N(), lambda _: {'linear': {'x': 0.1}})
        self.assertIsNone(row['source_ns'])

    def test_cloud_point_count_does_not_claim_valid_points(self):
        cloud = N(header=N(stamp=N(sec=1, nanosec=0), frame_id='optical'),
                  width=100, height=1, is_dense=False, point_step=16,
                  row_step=1600, data=b'\x00'*1600)
        row = metadata('cloud', cloud, None)
        self.assertEqual(row['data']['point_count'], 100)
        self.assertNotIn('valid_points', row['data'])

    def test_duplicate_topics_rejected(self):
        topic = {'topic': '/clock', 'type': 'rosgraph_msgs/msg/Clock',
                 'kind': 'clock', 'required': True, 'gap_budget_sec': None,
                 'qos': 'sensor'}
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            validate_topics([topic, topic])

    def test_unknown_qos_rejected(self):
        with self.assertRaisesRegex(ValueError, 'QoS'):
            validate_topics([{'topic': '/x', 'type': 'std_msgs/msg/String',
                              'kind': 'status', 'required': False,
                              'gap_budget_sec': None, 'qos': 'invented'}])

    def test_clock_is_required_for_age_interpretation(self):
        with self.assertRaisesRegex(ValueError, 'clock'):
            validate_topics([{'topic': '/x', 'type': 'std_msgs/msg/String',
                              'kind': 'status', 'required': False,
                              'gap_budget_sec': None, 'qos': 'sensor'}])

    def test_canonical_capture_preserves_guard_reason_and_native_hold_phase(self):
        config = Path(__file__).resolve().parents[2]/'docs/evidence/mainline_m5_20260924/topics.json'
        guard_topic = '/transport/execution_guard/status'
        phase_topic = '/transport/hold_executor/status'
        guard_data = {
            'stamp': {'sec': 38, 'nanosec': 639000000},
            'joint_stamp': {'sec': 38, 'nanosec': 610000000},
            'context_id': 'fixture-context', 'active': True, 'healthy': False,
            'reason': 'JOINT_TRACKING_ERROR', 'maximum_joint_error_rad': .051,
        }
        phase_data = {'phase': '5', 'reason': 'EXECUTION_GUARD_UNHEALTHY',
                      'authority_reason': 'RESOURCE_CLOCK_RESET', 'hold_reason': 'HOLD_CANCELED',
                      'hold_confirmed': False, 'lease_id': 'fixture-lease', 'epoch': 'fixture-epoch'}
        subscriptions, now, signal_handlers = {}, [1_000_000_000], {}

        class ObserverNode:
            def __init__(self, *args, **kwargs):
                pass

            def create_subscription(self, message_type, topic, callback, qos):
                subscriptions[topic] = callback
                return object()

            def destroy_node(self):
                pass

        def spin_once(node, timeout_sec):
            startup = json.loads((root/'capture/manifest.json').read_text())
            self.assertEqual(len(startup['topics']), 41)
            self.assertEqual(set(subscriptions), {item['topic'] for item in startup['topics']})
            self.assertFalse(startup['completed'])
            messages = {
                '/clock': N(clock=N(sec=38, nanosec=650000000)),
                guard_topic: N(**guard_data),
                phase_topic: N(data=json.dumps(phase_data)),
            }
            for topic, message in messages.items():
                if topic in subscriptions:
                    subscriptions[topic](message)
            now[0] += 2_000_000_000
            if interrupt_capture:
                signal_handlers[signal.SIGINT](signal.SIGINT, None)

        modules = {
            'rclpy': N(init=Mock(), shutdown=Mock(), spin_once=spin_once),
            'rclpy.node': N(Node=ObserverNode),
            'rclpy.qos': N(QoSProfile=lambda **kw: N(**kw),
                           DurabilityPolicy=N(TRANSIENT_LOCAL=1), qos_profile_sensor_data=object()),
            'rclpy.signals': N(SignalHandlerOptions=N(NO=0)),
            'rosidl_runtime_py.convert': N(message_to_ordereddict=lambda message:
                                          json.loads(json.dumps(message, default=vars))),
            'rosidl_runtime_py.utilities': N(get_message=lambda name: name),
        }
        for interrupt_capture in (False, True):
            subscriptions.clear()
            with self.subTest(interrupted=interrupt_capture), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                session = root/'session.json'
                session.write_text(json.dumps({'isolation': {'ROS_DOMAIN_ID': '89'}}))
                argv = ['capture', '--topic-config', str(config), '--session-json', str(session),
                        '--output', str(root/'capture'), '--seconds', '1', '--warmup-sec', '0',
                        '--phase-topic', phase_topic, '--phase-type', 'std_msgs/msg/String']
                with patch.dict(sys.modules, modules), patch.dict(os.environ, {'ROS_DOMAIN_ID': '89'}), \
                        patch.object(sys, 'argv', argv), \
                        patch('capture_m5_observer.signal.signal', side_effect=signal_handlers.__setitem__), \
                        patch('capture_m5_observer.time.monotonic_ns', side_effect=lambda: now[0]), \
                        patch('builtins.print'):
                    if interrupt_capture:
                        with self.assertRaises(SystemExit) as stopped:
                            main()
                        self.assertEqual(stopped.exception.code, 130)
                    else:
                        main()
                events = [json.loads(line) for line in (root/'capture/events.jsonl').read_text().splitlines()]
                guards = [row for row in events if row['topic'] == guard_topic]
                self.assertEqual(len(guards), 1, 'Required guard raw status was not captured')
                self.assertEqual(guards[0]['data'], guard_data)
                manifest = json.loads((root/'capture/manifest.json').read_text())
                self.assertEqual(len(manifest['topics']), 41)
                self.assertEqual(manifest['completed'], not interrupt_capture)
                self.assertEqual(manifest['interrupted'], interrupt_capture)
                guard_spec = next(item for item in manifest['topics'] if item['topic'] == guard_topic)
                self.assertEqual(guard_spec['type'], 'astribot_transport_msgs/msg/ExecutionGuardStatus')
                self.assertEqual(guard_spec['qos'], 'reliable')
                self.assertTrue(guard_spec['required'])
                report = analyze(root/'capture')
                self.assertEqual(report['observation_status'], 'INCOMPLETE' if interrupt_capture else 'COMPLETE')
                self.assertEqual(report['streams'][guard_topic]['presence'], 'PRESENT')
                guard_status = next(item for item in report['status_observations'] if item['topic'] == guard_topic)
                self.assertEqual(guard_status['data']['reason'], 'JOINT_TRACKING_ERROR')
                self.assertEqual(report['phase_observations'][0]['phase'], '5')
                phase_status = next(item for item in report['status_observations'] if item['topic'] == phase_topic)
                self.assertEqual(json.loads(phase_status['data']['data']), phase_data)

    def test_setup_failures_leave_analyzable_incomplete_evidence_without_ros(self):
        for failure in ('init', 'subscription'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                config = root/'topics.json'
                config.write_text(json.dumps({'topics': [{
                    'topic': '/clock', 'type': 'rosgraph_msgs/msg/Clock',
                    'kind': 'clock', 'required': True, 'gap_budget_sec': None, 'qos': 'sensor'}]}))
                session = root/'session.json'
                session.write_text(json.dumps({'isolation': {'ROS_DOMAIN_ID': '89'}}))
                node = Mock()
                node.create_subscription.side_effect = RuntimeError('subscription failed')
                modules = {
                    'rclpy': N(init=Mock(side_effect=RuntimeError('init failed') if failure == 'init' else None),
                               shutdown=Mock()),
                    'rclpy.node': N(Node=Mock(return_value=node)),
                    'rclpy.qos': N(QoSProfile=Mock(), DurabilityPolicy=N(TRANSIENT_LOCAL=1),
                                   qos_profile_sensor_data=object()),
                    'rclpy.signals': N(SignalHandlerOptions=N(NO=0)),
                    'rosidl_runtime_py.convert': N(message_to_ordereddict=Mock()),
                    'rosidl_runtime_py.utilities': N(get_message=Mock(return_value=object)),
                }
                argv = ['capture', '--topic-config', str(config), '--session-json', str(session),
                        '--output', str(root/'capture')]
                with patch.dict(sys.modules, modules), patch.dict(os.environ, {'ROS_DOMAIN_ID': '89'}), \
                        patch.object(sys, 'argv', argv), patch('capture_m5_observer.signal.signal'):
                    with self.assertRaisesRegex(RuntimeError, f'{failure} failed'):
                        main()
                report = analyze(root/'capture')
                self.assertEqual(report['observation_status'], 'INCOMPLETE')
                self.assertEqual(report['sampling_criteria_status'], 'INCOMPLETE')
                manifest = json.loads((root/'capture'/'manifest.json').read_text())
                self.assertIn(f'{failure} failed', manifest['error'])


if __name__ == '__main__':
    unittest.main()
