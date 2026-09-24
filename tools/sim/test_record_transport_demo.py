"""Offline recorder checks: real OpenCV encode/decode, substituted ROS delivery."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np


SCRIPT = Path(__file__).with_name('record_transport_demo.py')


def image(stamp_ns, head=False, encoding='rgb8'):
    height, width = (240, 320) if head else (720, 1280)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(
            sec=stamp_ns//10**9, nanosec=stamp_ns % 10**9), frame_id='head' if head else 'overview'),
        encoding=encoding, height=height, width=width, step=width*3,
        data=np.full((height, width, 3), 80, dtype=np.uint8).tobytes())


class RecorderTest(unittest.TestCase):
    def run_recording(self, directory, events, native=True, error=None, error_type=RuntimeError):
        callbacks = {}
        now = [100.0]
        node = SimpleNamespace(
            create_subscription=lambda typ, topic, callback, qos: callbacks.setdefault(topic, callback),
            destroy_node=lambda: None)
        ros = ModuleType('rclpy')
        ros.init = lambda: None
        ros.create_node = lambda name: node
        ros.shutdown = lambda: None
        ros.ok = lambda: True

        def spin_once(node, timeout_sec):
            if events:
                delta, topic, message = events.pop(0)
                now[0] = 100.0 + delta
                if isinstance(message, BaseException):
                    raise message
                # Old recorder did not subscribe /clock; still deliver other events
                # so missing output fails on the feature assertion, not the fixture.
                if topic in callbacks:
                    callbacks[topic](message)
            else:
                now[0] = 120.0

        ros.spin_once = spin_once
        qos = ModuleType('rclpy.qos')
        qos.qos_profile_sensor_data = object()
        sensor = ModuleType('sensor_msgs.msg')
        sensor.Image = object
        std = ModuleType('std_msgs.msg')
        std.String = object
        graph = ModuleType('rosgraph_msgs.msg')
        graph.Clock = object
        modules = {'rclpy': ros, 'rclpy.qos': qos, 'sensor_msgs.msg': sensor,
                   'std_msgs.msg': std, 'rosgraph_msgs.msg': graph}
        output = Path(directory)/'clip.mp4'
        args = [str(SCRIPT), '--output', str(output), '--timeout', '10',
                '--head-topic', '/head', '--status-topic', '/status',
                '--status-format', 'native_hold' if native else 'legacy', '--task-id', 'test-task']
        with patch.dict(sys.modules, modules), patch.object(sys, 'argv', args):
            spec = importlib.util.spec_from_file_location('recorder_under_test', SCRIPT)
            recorder = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(recorder)
            recorder.time = SimpleNamespace(
                monotonic=lambda: now[0], monotonic_ns=lambda: round(now[0]*1e9),
                time=lambda: now[0]+1000, time_ns=lambda: round((now[0]+1000)*1e9))
            if error:
                with self.assertRaisesRegex(error_type, error):
                    recorder.main()
            else:
                recorder.main()
        self.assertTrue(output.with_suffix('.frames.jsonl').exists(),
                        'Each actual encoded frame needs its original source and receive times')
        rows = [json.loads(line) for line in output.with_suffix('.frames.jsonl').read_text().splitlines()]
        summary = json.loads(output.with_suffix('.json').read_text())
        decoder = cv2.VideoCapture(str(output))
        decoded = 0
        while decoder.read()[0]:
            decoded += 1
        decoder.release()
        self.assertEqual(decoded, len(rows), 'Sidecar must map one-to-one to decodable frames')
        self.assertEqual(summary['frames'], decoded)
        return rows, summary

    def test_native_frames_preserve_cached_head_gaps_and_intermediate_status(self):
        # Refreshing cached metadata or attaching status to a nonexistent frame
        # would falsify the input age / motion-coverage evidence.
        status = lambda reason: SimpleNamespace(data=json.dumps(
            dict(reason=reason, phase='EXECUTING', hold_confirmed=False, context_id='context-1')))
        clock = SimpleNamespace(clock=SimpleNamespace(sec=10, nanosec=500))
        events = [(.1, '/transport/overview', image(10_000_000_000)),
                  (.15, '/clock', clock), (.2, '/head', image(10_100_000_000, head=True)),
                  (.3, '/transport/overview', image(10_200_000_000)),
                  (.8, '/status', status('EXECUTING_FIRST_MTC_STAGE')),
                  (.9, '/status', status('EXECUTION_GUARD_UNHEALTHY')),
                  (1.2, '/transport/overview', image(10_200_000_000)),
                  (1.3, '/transport/overview', image(9_000_000_000))]
        with tempfile.TemporaryDirectory() as directory:
            rows, summary = self.run_recording(directory, events)
        self.assertEqual([r['frame_index'] for r in rows], [0, 1, 2, 3])
        self.assertEqual([r['overview']['source_ns'] for r in rows],
                         [10_000_000_000, 10_200_000_000, 10_200_000_000, 9_000_000_000])
        self.assertIsNone(rows[0]['head'])
        self.assertIsNone(rows[0]['overview']['observed_clock'])
        self.assertEqual(rows[1]['overview']['observed_clock']['source_ns'], 10_000_000_500)
        self.assertEqual(rows[1]['head'], rows[3]['head'])
        self.assertEqual(rows[3]['head']['source_ns'], 10_100_000_000)
        self.assertEqual(rows[3]['head']['receive_monotonic_ns'], 100_200_000_000)
        self.assertEqual(rows[2]['overview']['receive_monotonic_ns'] -
                         rows[1]['overview']['receive_monotonic_ns'], 900_000_000)
        self.assertEqual([r['status']['receive_seq'] for r in rows], [0, 0, 2, 2])
        self.assertEqual([e['receive_seq'] for e in summary['status_events']], [1, 2])
        self.assertEqual([e['frame'] for e in summary['stage_frames']], [2, 2])
        self.assertEqual(rows[2]['status']['raw_status']['context_id'], 'context-1')
        self.assertEqual(summary['task_id'], 'test-task')
        self.assertEqual(summary['status_format'], 'native_hold')
        for row in rows:
            self.assertLessEqual(row['overview']['receive_monotonic_ns'], row['encode_started_monotonic_ns'])
            self.assertLessEqual(row['encode_started_monotonic_ns'], row['encode_finished_monotonic_ns'])

    def test_legacy_status_and_zero_source_stamp_are_not_rewritten(self):
        events = [(.1, '/status', SimpleNamespace(data='{"stage":"SUCCEEDED","object_state":"PLACED"}')),
                  (.2, '/transport/overview', image(0))]
        with tempfile.TemporaryDirectory() as directory:
            rows, summary = self.run_recording(directory, events, native=False)
        self.assertEqual(rows[0]['overview']['source_ns'], 0)
        self.assertIsNone(rows[0]['overview']['observed_clock'])
        self.assertEqual(summary['last_status']['stage'], 'SUCCEEDED')
        self.assertEqual(rows[0]['status']['raw_status']['object_state'], 'PLACED')

    def test_callback_error_still_flushes_existing_frame_evidence(self):
        events = [(.1, '/transport/overview', image(10)),
                  (.2, '/transport/overview', image(20, encoding='unexpected'))]
        with tempfile.TemporaryDirectory() as directory:
            rows, summary = self.run_recording(directory, events, error='Expected rgb8')
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['overview']['source_ns'], 10)

    def test_interrupt_preserves_frames_without_swallowing_interruption(self):
        events = [(.1, '/transport/overview', image(30)),
                  (.2, '/transport/overview', KeyboardInterrupt('owned stop'))]
        with tempfile.TemporaryDirectory() as directory:
            rows, summary = self.run_recording(directory, events, error='owned stop',
                                               error_type=KeyboardInterrupt)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['overview']['source_ns'], 30)


if __name__ == '__main__':
    unittest.main()
