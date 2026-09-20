#!/usr/bin/env python3
"""Read-only hardware input and control-ownership report; never opens an SDK session."""
import argparse
import json
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=8.0)
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--scan-topic', default='/scan_from_cloud')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--require-idle-navigation', action='store_true')
    parser.add_argument('--require-idle-bridge', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 2 <= args.seconds <= 60:
        parser.error('--seconds must be between 2 and 60')
    rclpy.init()
    node = rclpy.create_node('astribot_hardware_readiness')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    samples = {'odom': [], 'scan': [], 'map': []}
    messages = {}

    def receive(key, msg):
        samples[key].append((time.monotonic(), msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec))
        messages[key] = msg

    qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Odometry, args.odom_topic, lambda m: receive('odom', m), qos)
    node.create_subscription(LaserScan, args.scan_topic, lambda m: receive('scan', m), qos)
    discovery_deadline = time.monotonic() + 2.0
    map_endpoints = []
    while not map_endpoints and time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        map_endpoints = node.get_publishers_info_by_topic(args.map_topic)
    durability = DurabilityPolicy.VOLATILE
    if map_endpoints and all(p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                             for p in map_endpoints):
        durability = DurabilityPolicy.TRANSIENT_LOCAL
    node.create_subscription(OccupancyGrid, args.map_topic, lambda m: receive('map', m),
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                        durability=durability))
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    report = {'read_only': True, 'topics': {}, 'errors': []}
    for key, topic in [('odom', args.odom_topic), ('scan', args.scan_topic), ('map', args.map_topic)]:
        rows = samples[key]
        unique = len({stamp for _, stamp in rows})
        span = rows[-1][0] - rows[0][0] if len(rows) > 1 else 0.0
        rate = (len(rows) - 1) / span if span > 0 else 0.0
        endpoints = node.get_publishers_info_by_topic(topic)
        publishers = [f'{p.node_namespace.rstrip("/")}/{p.node_name}' for p in endpoints]
        data = {'topic': topic, 'messages': len(rows), 'unique_stamps': unique,
                'receive_hz': rate, 'publishers': publishers,
                'publisher_gids': [bytes(p.endpoint_gid).hex() for p in endpoints]}
        if rows:
            data['source_age_s'] = node.get_clock().now().nanoseconds / 1e9 - rows[-1][1] / 1e9
        report['topics'][key] = data
        if len(publishers) != 1:
            report['errors'].append(f'{topic}: expected one unambiguous publisher')
        if not rows:
            report['errors'].append(f'{topic}: no messages received')
        elif key != 'map':
            if rate < 5 or unique < 3 or not -0.1 <= data['source_age_s'] <= 0.5:
                report['errors'].append(f'{topic}: low rate, frozen timestamps or stale/future data')
    if 'map' in messages:
        grid = messages['map']
        report['topics']['map'].update(frame=grid.header.frame_id, width=grid.info.width,
                                      height=grid.info.height, resolution=grid.info.resolution)
        if grid.info.width == 0 or grid.info.height == 0:
            report['errors'].append('map must be nonempty')
        if grid.header.frame_id != 'map':
            try:
                buffer.lookup_transform('map', grid.header.frame_id, rclpy.time.Time())
            except Exception:
                report['errors'].append('map frame cannot be transformed to navigation frame map')
    if 'odom' in messages:
        odom = messages['odom']
        values = [odom.pose.pose.position.x, odom.pose.pose.position.y,
                  odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z]
        if not all(math.isfinite(v) for v in values):
            report['errors'].append('odom contains nonfinite values')
        report['topics']['odom']['frame'] = odom.header.frame_id
        report['topics']['odom']['child_frame'] = odom.child_frame_id
    try:
        tf = buffer.lookup_transform('map', 'astribot_torso_base', rclpy.time.Time())
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        age = node.get_clock().now().nanoseconds / 1e9 - stamp
        report['tf'] = {'age_s': age, 'x': tf.transform.translation.x, 'y': tf.transform.translation.y}
        if not -0.1 <= age <= 0.5:
            report['errors'].append('map -> astribot_torso_base TF stale or future')
    except Exception as exc:
        report['errors'].append('map -> astribot_torso_base TF unavailable: ' + str(exc))
    names = node.get_node_names_and_namespaces()
    nav_names = {'controller_server', 'planner_server', 'bt_navigator', 'velocity_smoother',
                 'xtalpi_navigation', 'task_arbiter', 'policy_controller', 'final_protection',
                 'exploration_coordinator', 'exploration_coordinator_node', 'frontier_explorer_node',
                 'navigation_task_arbiter'}
    bridge_names = {'chassis_cmd_bridge', 'astribot_bridge_container', 'arm_traj_bridge'}
    for key, selected, required in [('navigation_conflicts', nav_names, args.require_idle_navigation),
                                    ('bridge_conflicts', bridge_names, args.require_idle_bridge)]:
        report[key] = sorted({f'{ns.rstrip("/")}/{name}' for name, ns in names if name in selected})
        if required and report[key]:
            report['errors'].append(key + ': stop the previous owner before starting this role')
    report['ready'] = not report['errors']
    print(json.dumps(report, indent=2, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['ready'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
