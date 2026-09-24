"""Exercise the real snapshot main with ROS stand-ins; never initialize ROS."""
import json
from pathlib import Path
import runpy
import sys
import tempfile
from types import SimpleNamespace as N
import unittest
from unittest.mock import Mock, patch

import cv2  # Load numerical/image libraries before replacing the time module.
import numpy as np
import prepare_box_roi_snapshot


class SnapshotTruthFilterTest(unittest.TestCase):
    def test_capture_selects_matching_truth_and_preserves_default_path(self):
        for mode in ('matching', 'default', 'no_match'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                timestamp = N(sec=1, nanosec=0)
                header = N(stamp=timestamp, frame_id='optical')
                image_type, info_type, clock_type, health_type = (type(name, (), {}) for name in
                    ('Image', 'CameraInfo', 'Clock', 'CameraHealth'))
                health = dict(valid=True, camera_id='head_rgbd', source_epoch='fixture',
                    frame_id='optical', calibration_revision=1, capture_stamp={'sec': 1, 'nanosec': 0})
                rgb = N(header=header, encoding='rgb8', is_bigendian=False, height=10, width=10,
                        step=30, data=np.zeros((10, 10, 3), np.uint8).tobytes())
                depth = N(header=header, encoding='32FC1', is_bigendian=False, height=10, width=10,
                          step=40, data=np.full((10, 10), 2., np.float32).tobytes())
                info = N(header=header, height=10, width=10, k=[10,0,4.5,0,10,4.5,0,0,1],
                         d=[], p=[0.]*12, r=np.eye(3).ravel(), distortion_model='plumb_bob')
                health_message = N(capture_stamp=timestamp)

                class FakeNode:
                    def __init__(self, *args, **kwargs): pass
                    def destroy_node(self): pass
                    def create_subscription(self, kind, topic, callback, qos):
                        message = (health_message if kind is health_type else N(clock=timestamp)
                                   if kind is clock_type else info if kind is info_type else
                                   depth if topic.endswith('/depth_image') else rgb)
                        callback(message)
                        return object()

                class FakeBuffer:
                    def lookup_transform(self, parent, child, stamp):
                        return N(header=N(frame_id=parent, stamp=timestamp), child_frame_id=child,
                                 transform=N(translation=N(x=0., y=0., z=0.),
                                             rotation=N(x=0., y=0., z=0., w=1.)))

                fake_time = N(monotonic=Mock(side_effect=[0., .1, .1, .2, .2, 21., 21., 22.]))
                modules = {
                    'time': fake_time,
                    'rclpy': N(init=Mock(), shutdown=Mock(), spin_once=Mock()),
                    'rclpy.node': N(Node=FakeNode),
                    'rclpy.time': N(Time=N(from_msg=lambda message: message)),
                    'rclpy.qos': N(qos_profile_sensor_data=object()),
                    'sensor_msgs.msg': N(Image=image_type, CameraInfo=info_type),
                    'rosgraph_msgs.msg': N(Clock=clock_type),
                    'astribot_perception_msgs.msg': N(CameraHealth=health_type),
                    'tf2_ros': N(Buffer=FakeBuffer, TransformListener=lambda *_: object()),
                    'rosidl_runtime_py.convert': N(message_to_ordereddict=lambda _: health),
                }
                seconds = 1 if mode == 'matching' else 2
                truth = root/'truth.pbtxt'
                truth.write_text(f'header {{\n stamp {{ sec: {seconds} }}\n}}\n'
                    'pose {\n name: "fixture"\n position {}\n orientation { w: 1 }\n}\n'
                    f'header {{\n stamp {{ sec: {seconds+1} }}\n}}\n')
                script = Path(__file__).with_name('sim_pose_capture.py')
                argv = [str(script), '--seconds', '0', '--output', str(root/'capture')]
                if mode != 'default': argv += ['--truth-stream', str(truth)]
                with patch.dict(sys.modules, modules), patch.object(sys, 'argv', argv), patch('builtins.print'):
                    if mode == 'no_match':
                        with self.assertRaisesRegex(RuntimeError, 'bounded window'):
                            runpy.run_path(str(script), run_name='__main__')
                        self.assertFalse((root/'capture/camera_info.json').exists())
                    else:
                        runpy.run_path(str(script), run_name='__main__')
                        meta = json.loads((root/'capture/camera_info.json').read_text())
                        self.assertEqual(meta['capture_stamp_ns'], 1_000_000_000)
                        self.assertEqual(meta['truth_stamp_filter']['enabled'], mode == 'matching')
                        self.assertFalse(meta['truth_used_for_segmentation_or_projection'])


if __name__ == '__main__':
    unittest.main()
