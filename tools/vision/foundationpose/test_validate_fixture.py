"""Boundary checks for evidence acceptance, without starting ROS or a GPU."""
import copy
import unittest

from validate_fixture import check_output


def valid_output():
    header = {'stamp': {'sec': 12, 'nanosec': 34}, 'frame_id': 'camera'}
    return {
        'header': header,
        'detections': [{
            'header': copy.deepcopy(header),
            'bbox': {
                'center': {'position': {'x': 0., 'y': 0., 'z': 1.},
                           'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}},
                'size': {'x': .1, 'y': .2, 'z': .3}},
            'results': []}]}


class OutputBoundaryTest(unittest.TestCase):
    def test_matching_finite_pose(self):
        self.assertEqual(check_output(valid_output(), 12000000034, 'camera'), [])

    def test_empty_is_not_proof_of_life(self):
        msg = valid_output()
        msg['detections'] = []
        self.assertIn('empty_detections', check_output(msg, 12000000034, 'camera'))

    def test_foreign_stamp_and_frame_are_rejected(self):
        errors = check_output(valid_output(), 12000000035, 'different_camera')
        self.assertIn('array_stamp_mismatch', errors)
        self.assertIn('array_frame_mismatch', errors)

    def test_nonfinite_and_invalid_quaternion_are_rejected(self):
        msg = valid_output()
        pose = msg['detections'][0]['bbox']['center']
        pose['position']['z'] = float('nan')
        pose['orientation']['w'] = 0.
        errors = check_output(msg, 12000000034, 'camera')
        self.assertTrue(any('nonfinite' in e for e in errors))
        self.assertTrue(any('quaternion' in e for e in errors))

    def test_behind_camera_and_zero_size_are_rejected(self):
        msg = valid_output()
        msg['detections'][0]['bbox']['center']['position']['z'] = -1.
        msg['detections'][0]['bbox']['size']['x'] = 0.
        errors = check_output(msg, 12000000034, 'camera')
        self.assertTrue(any('behind_camera' in e for e in errors))
        self.assertTrue(any('size' in e for e in errors))

    def test_detection_header_cannot_disagree_with_array(self):
        msg = valid_output()
        msg['detections'][0]['header']['stamp']['sec'] = 13
        self.assertIn('detection_0_stamp_mismatch', check_output(msg, 12000000034, 'camera'))


if __name__ == '__main__':
    unittest.main()
