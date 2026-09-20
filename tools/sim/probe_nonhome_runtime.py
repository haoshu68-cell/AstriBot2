#!/usr/bin/env python3
"""Read-only, bounded recording of the fixed-posture navigation contract.

Run in the environment of the owned simulation. This does not grant a hold,
change a parameter, or send robot commands. Messages retain acquisition stamps.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from astribot_navigation_msgs.msg import (
    ArmHoldStatus, EnvelopeApplyStatus, NavigationEnvelopeV2, RobotGeometryState, PassageAssessment, SensorHealthArray,
)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=180.)
    parser.add_argument('--output', required=True, help='New evidence directory')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('duration must be finite and positive')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('nonhome_runtime_probe', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    counts, reasons = Counter(), Counter()
    ages, leases, ready_epochs = [], [], set()
    last_odom = -math.inf
    start = time.monotonic()
    with (output / 'messages.jsonl').open('w') as stream:
        def receive(kind, msg):
            nonlocal last_odom
            counts[kind] += 1
            now = node.get_clock().now().nanoseconds * 1e-9
            wall = time.monotonic() - start
            if kind == 'odom':
                if wall - last_odom < .1:
                    return
                last_odom = wall
            if kind == 'geometry':
                reasons[msg.reason] += 1
                if msg.complete:
                    age=now-seconds(msg.header.stamp)
                    if now>0 and age>=0:
                        ages.append(age)
                        leases.append(seconds(msg.valid_until) - now)
                    else:
                        counts['geometry_clock_not_aligned']+=1
            if kind == 'envelope' and msg.navigation_allowed:
                ready_epochs.add((msg.coordinator_session_id, msg.epoch))
            record = dict(kind=kind, received_ros_s=now, received_wall_s=wall,
                          message=message_to_ordereddict(msg))
            if kind == 'scan':
                # Record acquisition timing without duplicating full scans.
                record['message'] = dict(header=record['message']['header'],
                                         range_count=len(msg.ranges))
            stream.write(json.dumps(record, separators=(',', ':')) + '\n')
            stream.flush()

        subscriptions = []
        for kind, cls, topic, qos in [
            ('sensor_health', SensorHealthArray, '/navigation/sensor_health', 10),
            ('observation', String, '/navigation_policy/observation', 10),
            ('geometry', RobotGeometryState, '/navigation/geometry_state', 10),
            ('envelope', NavigationEnvelopeV2, '/navigation/envelope_v2', 10),
            ('ack', EnvelopeApplyStatus, '/navigation/envelope_applied', 20),
            ('hold', ArmHoldStatus, '/navigation/arm_hold', 10),
            ('passage', PassageAssessment, '/navigation/passage_assessment', 10),
            ('odom', Odometry, '/odom', qos_profile_sensor_data),
            ('scan', LaserScan, '/scan_from_cloud', qos_profile_sensor_data),
            ('raw_command', Twist, '/cmd_vel_nav_body_raw', 10),
            ('smooth_command', Twist, '/cmd_vel_nav_body', 10),
            ('world_command', Twist, '/cmd_vel_pre_arm_coupling', 10),
            ('policy_input_command', Twist, '/cmd_vel_policy_input', 10),
            ('final_command', Twist, '/cmd_vel', 10),
            ('phase', String, '/path_tracking/phase', qos_profile_sensor_data),
            ('task', String, '/transport/status', 10),
            ('policy', String, '/navigation_policy/state', 10),
            ('attachment_filter', String, '/navigation/attachment_filter_applied', 10),
            ('payload_sync', String, '/model/transport_box_01/kinematic_attachment/state', 10),
        ]:
            subscriptions.append(node.create_subscription(
                cls, topic, lambda msg, k=kind: receive(k, msg), qos))
        try:
            while time.monotonic() - start < args.duration:
                rclpy.spin_once(node, timeout_sec=.05)
        except (KeyboardInterrupt,ExternalShutdownException):
            pass
        except RuntimeError:
            # Humble can invalidate a subscription while SIGINT shuts down
            # the context. Preserve the summary only for that shutdown race;
            # a conversion error in a live context remains a real failure.
            if rclpy.ok():raise
    def stats(values):
        values = sorted(values)
        return (dict(n=len(values), minimum=values[0], maximum=values[-1],
                     p95=values[math.ceil(.95 * len(values)) - 1]) if values else None)
    summary = dict(wall_seconds=time.monotonic() - start, counts=counts,
                   geometry_reasons=reasons, geometry_age_s=stats(ages),
                   geometry_remaining_lease_s=stats(leases),
                   ready_epochs=sorted(ready_epochs),
                   evidence='passive ROS receipt; not contact or arrival acceptance')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary), flush=True)
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    main()
