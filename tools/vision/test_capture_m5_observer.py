"""Offline checks: a passive observer must never turn image bytes into evidence."""
import json
import os
from pathlib import Path
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
