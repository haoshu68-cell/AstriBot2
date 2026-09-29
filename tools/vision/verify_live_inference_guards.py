#!/usr/bin/env python3
"""Fault injection against the perception-only Action; never commands a robot."""
import argparse
import copy
import json
import struct
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from astribot_perception_msgs.action import ComputeGrasps
from sim_pose_capture import Capture
from validate_inference_actions import await_future, live_snapshot


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = Capture('torso_rgbd')
    client = ActionClient(node, ComputeGrasps, '/perception/compute_grasps')
    assert client.wait_for_server(timeout_sec=10), 'Perception Action unavailable'
    cases = ['wrong_frame', 'stale_input', 'wrong_calibration', 'wrong_scene',
             'wrong_envelope', 'wrong_epoch', 'nonfinite', 'sparse',
             'timeout', 'expired_result', 'cancel', 'recovery']
    report = {'source': 'live RGB-D plus deliberate request faults',
              'real_worker': True, 'execution_permission': False, 'cases': []}
    try:
        for case in cases:
            cloud, health, age = live_snapshot(node)
            # A returned Header may share the immutable subscription cache.
            # Deliberate request corruption must not alter the next snapshot.
            cloud = copy.deepcopy(cloud)
            goal = ComputeGrasps.Goal()
            goal.header, goal.object_cloud = cloud.header, cloud
            goal.task_id, goal.object_id, goal.arm_id = 'guard_'+case, 'asymmetric_union', 'right'
            goal.camera_id, goal.source_epoch = 'torso_rgbd', health['source_epoch']
            goal.calibration_revision = health['calibration_revision']
            goal.planning_scene_revision, goal.envelope_epoch = 1, 1
            goal.timeout_sec, goal.max_candidates = 10., 8
            capture = Time.from_msg(cloud.header.stamp)
            goal.valid_until = (capture + Duration(seconds=5)).to_msg()
            if case == 'wrong_frame':
                goal.header.frame_id = 'wrong_optical_frame'
            elif case == 'stale_input':
                goal.header.stamp = (capture - Duration(seconds=2)).to_msg()
                goal.valid_until = (capture + Duration(seconds=1)).to_msg()
            elif case == 'wrong_calibration':
                goal.calibration_revision += 1
            elif case == 'wrong_scene':
                goal.planning_scene_revision += 1
            elif case == 'wrong_envelope':
                goal.envelope_epoch += 1
            elif case == 'wrong_epoch':
                goal.source_epoch += '_invalid'
            elif case == 'nonfinite':
                data = bytearray(cloud.data)
                struct.pack_into('<f', data, 0, float('nan'))
                goal.object_cloud.data = bytes(data)
            elif case == 'sparse':
                goal.object_cloud.width = 32
                goal.object_cloud.row_step = 32*12
                goal.object_cloud.data = bytes(cloud.data[:32*12])
            elif case == 'timeout':
                goal.timeout_sec = .05
            elif case == 'expired_result':
                goal.valid_until = (capture + Duration(seconds=.4)).to_msg()
            began = time.monotonic()
            handle = await_future(node, client.send_goal_async(goal), 3)
            row = {'case': case, 'accepted': handle.accepted, 'capture_age_sec': age}
            if handle.accepted:
                if case == 'cancel':
                    # Let the real worker enter loading/inference before cancellation.
                    until = time.monotonic()+.1
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=.01)
                    canceled = await_future(node, handle.cancel_goal_async(), 3)
                    row['cancel_return_code'] = canceled.return_code
                outcome = await_future(node, handle.get_result_async(), 15)
                row.update(status=outcome.status, success=outcome.result.success,
                           reason=outcome.result.reason_code, candidates=len(outcome.result.candidates))
            row['latency_sec'] = time.monotonic()-began
            if case == 'recovery':
                row['passed'] = row.get('success') is True and row.get('candidates', 0) > 0
            elif case in ('timeout', 'expired_result', 'cancel'):
                expected = {'timeout': {'INFERENCE_TIMEOUT', 'WORKER_TIMEOUT'},
                            'expired_result': {'RESULT_EXPIRED'}, 'cancel': {'CANCELED'}}[case]
                row['passed'] = (row.get('success') is False and row.get('candidates') == 0
                                 and row.get('reason') in expected
                                 and (case != 'cancel' or row.get('status') == 5))
            else:
                row['passed'] = not handle.accepted or (row.get('success') is False and row.get('candidates') == 0)
            report['cases'].append(row)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2)+'\n')
            print(json.dumps(row), flush=True)
        assert all(row['passed'] for row in report['cases']), 'A live inference guard failed'
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
