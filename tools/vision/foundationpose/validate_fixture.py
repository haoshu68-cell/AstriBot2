#!/usr/bin/env python3
"""One-shot offline replay/validation, never a robot runtime or pose estimator.

Run with the C++ backend inside the same network-isolated container. It reads
only explicitly named RGB-D/mask/camera files; no evaluation truth is consumed.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time


def stamp_ns(header):
    stamp = header.get('stamp', {})
    return int(stamp.get('sec', 0)) * 1_000_000_000 + int(stamp.get('nanosec', 0))


def check_output(message, expected_stamp_ns, expected_frame):
    """Check association and numeric sanity, NOT geometric pose accuracy."""
    errors = []

    def header_check(header, label):
        if stamp_ns(header) != expected_stamp_ns:
            errors.append(label + '_stamp_mismatch')
        if header.get('frame_id') != expected_frame:
            errors.append(label + '_frame_mismatch')

    header_check(message.get('header', {}), 'array')
    detections = message.get('detections', [])
    if not detections:
        errors.append('empty_detections')
    for index, detection in enumerate(detections):
        label = f'detection_{index}'
        header_check(detection.get('header', {}), label)
        try:
            box = detection['bbox']
            pose = box['center']
            xyz = [float(pose['position'][key]) for key in 'xyz']
            xyzw = [float(pose['orientation'][key]) for key in 'xyzw']
            size = [float(box['size'][key]) for key in 'xyz']
            if not all(math.isfinite(x) for x in xyz + xyzw + size):
                errors.append(label + '_nonfinite')
            norm = math.sqrt(sum(x*x for x in xyzw))
            if not math.isfinite(norm) or abs(norm - 1.) > .01:
                errors.append(label + '_invalid_quaternion')
            if xyz[2] <= 0:
                errors.append(label + '_behind_camera')
            if not all(x > 0 for x in size):
                errors.append(label + '_invalid_size')
            for hypothesis in detection.get('results', []):
                if not math.isfinite(float(hypothesis['hypothesis']['score'])):
                    errors.append(label + '_nonfinite_score')
        except (KeyError, TypeError, ValueError):
            errors.append(label + '_malformed_pose')
    return errors


def load_fixture(folder, kind):
    import numpy as np
    from PIL import Image

    names = (['mustard_rgb.png', 'mustard_depth.png', 'mustard_mask.png', 'camera_info.json']
             if kind == 'official' else ['rgb.png', 'depth.npz', 'mask.png', 'camera.json'])
    paths = [folder / name for name in names]
    rgb = np.asarray(Image.open(paths[0]).convert('RGB'))
    if kind == 'official':
        raw_depth = np.asarray(Image.open(paths[1]))
        if raw_depth.dtype.kind not in 'iu' or raw_depth.max() > 65535:
            raise ValueError('Official sample must contain uint16 millimetre depth')
        depth = raw_depth.astype(np.float32) / 1000.
    else:
        with np.load(paths[1], allow_pickle=False) as data:
            if len(data.files) != 1:
                raise ValueError('Expected exactly one depth array')
            depth = data[data.files[0]]
        if depth.dtype != np.float32:
            raise ValueError('Project depth must already be float32 metres')
    mask = np.asarray(Image.open(paths[2]).convert('L'))
    camera = json.loads(paths[3].read_text())
    shape = (camera['height'], camera['width'])
    if rgb.shape != (*shape, 3) or depth.shape != shape or mask.shape != shape:
        raise ValueError('RGB, depth, mask and camera dimensions disagree')
    valid = np.isfinite(depth) & (depth > 0)
    if not ((mask > 0) & valid).any():
        raise ValueError('No valid depth under segmentation mask')
    if len(camera['K']) != 9 or not all(math.isfinite(x) for x in camera['K']):
        raise ValueError('Invalid camera intrinsics')
    if camera['K'][0] <= 0 or camera['K'][4] <= 0:
        raise ValueError('Nonpositive focal length')
    if any(float(x) != 0. for x in camera['D']):
        raise ValueError('P0 fixtures must be undistorted before replay')
    if kind == 'project':
        if camera['depth_units'] != 'm' or camera['execution_authorized'] is not False:
            raise ValueError('Project fixture unit/authorization contract violated')
        capture_stamp = int(camera['capture_stamp_ns'])
        if any(int(v) != capture_stamp for v in camera['exact_sync_stamps'].values()):
            raise ValueError('Project fixture is not exactly synchronized')
        frame = camera['frame']
        stamp_kind = 'original_historical_capture'
    else:
        # The upstream image fixture has no capture timestamp. This synthetic
        # replay stamp identifies the request and makes no freshness claim.
        capture_stamp = time.time_ns()
        frame = camera['header']['frame_id']
        stamp_kind = 'synthetic_offline_replay_no_original_capture_stamp'
    if capture_stamp <= 0 or not frame:
        raise ValueError('Missing frame/stamp')
    receipt = {
        'kind': kind, 'capture_stamp_ns': capture_stamp, 'stamp_kind': stamp_kind,
        'frame': frame, 'shape': list(shape),
        'mask_pixels': int((mask > 0).sum()),
        'valid_mask_depth_pixels': int(((mask > 0) & valid).sum()),
        'files': {p.name: {'bytes': p.stat().st_size,
                         'sha256': hashlib.sha256(p.read_bytes()).hexdigest()} for p in paths}}
    return rgb, depth, mask, camera, receipt


def json_safe(value):
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--kind', choices=['official', 'project'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=120.)
    args = parser.parse_args()
    if args.output.exists() or not 1 <= args.timeout <= 300:
        parser.error('Output must be new; timeout must be within [1, 300] seconds')
    rgb, depth, mask, camera, receipt = load_fixture(args.input, args.kind)

    # ROS imports are intentionally local: CPU evidence checks run without ROS.
    import rclpy
    from cv_bridge import CvBridge
    from rosidl_runtime_py.convert import message_to_ordereddict
    from sensor_msgs.msg import CameraInfo, Image
    from vision_msgs.msg import Detection3DArray

    rclpy.init()
    node = rclpy.create_node('p0_offline_fixture_validator')
    bridge = CvBridge()
    images = [bridge.cv2_to_imgmsg(rgb, 'rgb8'),
              bridge.cv2_to_imgmsg(depth, '32FC1'),
              bridge.cv2_to_imgmsg(mask, 'mono8')]
    info = CameraInfo()
    info.width, info.height = camera['width'], camera['height']
    info.distortion_model = camera.get('distortion_model', 'plumb_bob')
    for field in ['d', 'k', 'r', 'p']:
        setattr(info, field, [float(v) for v in camera[field.upper()]])
    messages = [*images, info]
    for message in messages:
        message.header.frame_id = receipt['frame']
        message.header.stamp.sec = receipt['capture_stamp_ns'] // 1_000_000_000
        message.header.stamp.nanosec = receipt['capture_stamp_ns'] % 1_000_000_000
    topics = ['image', 'depth_image', 'segmentation', 'camera_info']
    publishers = [node.create_publisher(type(message), '/p0/pose_estimation/' + topic, 10)
                  for topic, message in zip(topics, messages)]
    received = []
    subscription = node.create_subscription(
        Detection3DArray, '/p0/pose_estimation/output',
        lambda msg: received.append((time.monotonic_ns(), message_to_ordereddict(msg))), 10)
    start = time.monotonic_ns()
    publications = []
    try:
        while time.monotonic_ns() - start < args.timeout * 1e9 and not received:
            now = time.monotonic_ns()
            ready = all(pub.get_subscription_count() > 0 for pub in publishers)
            if ready and (not publications or now - publications[-1] >= 10_000_000_000):
                publications.append(now)
                for pub, message in zip(publishers, messages):
                    pub.publish(message)
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()

    raw = received[0][1] if received else None
    errors = (check_output(raw, receipt['capture_stamp_ns'], receipt['frame'])
              if raw else ['output_timeout'])
    report = {
        'schema': 'astribot.foundationpose.offline_fixture_result/1',
        'status': 'PASS' if not errors else 'FAIL',
        'scope': 'offline registration numeric/header sanity, not geometric accuracy or tracking',
        'execution_authorized': False, 'input': receipt, 'errors': errors,
        'validation_start_monotonic_ns': start, 'publish_monotonic_ns': publications,
        'receive_monotonic_ns': received[0][0] if received else None,
        'first_publish_to_first_output_ms': ((received[0][0] - publications[0]) / 1e6
                                            if received and publications else None),
        'timing_limit': 'Includes transport, queueing and retries; not isolated model latency',
        'raw_detection3d_array': json_safe(raw)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'status': report['status'], 'errors': errors,
                      'output': str(args.output)}))
    return 0 if not errors else 2


if __name__ == '__main__':
    raise SystemExit(main())
