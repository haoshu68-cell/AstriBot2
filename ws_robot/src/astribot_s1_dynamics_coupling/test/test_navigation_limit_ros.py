"""Bounded dedicated-domain test for upstream arm percentages, never chassis motion."""
import json
import os
from pathlib import Path
import subprocess
import signal
import time

import pytest
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import SpeedLimit
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.time import Time


@pytest.mark.parametrize("metric", ["joint_deviation", "horizontal_reach"])
def test_arm_limit_capture_and_invalid_input(tmp_path, metric):
    binary = Path(os.environ['DYNAMICS_CPP']).resolve()
    domain = int(os.environ['DYNAMICS_DOMAIN'])
    assert domain == int(os.environ['ROS_DOMAIN_ID']) and domain > 0
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert binary.is_file()
    evidence = Path(os.environ.get('DYNAMICS_EVIDENCE', str(tmp_path)))
    evidence = evidence / metric
    evidence.mkdir(parents=True, exist_ok=True)
    log = (evidence / 'arm_navigation_limit.log').open('w')
    command = [str(binary), '--ros-args', '-p', f'extension_metric:={metric}',
               '-p', 'use_sim_time:=false']
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    record = {'command': command, 'pid': process.pid, 'domain': domain,
              'start_ticks': Path(f'/proc/{process.pid}/stat').read_text().rsplit(')', 1)[1].split()[19],
              'received': [], 'passed': False}
    rclpy.init()
    node = Node('arm_navigation_limit_fixture')
    publisher = node.create_publisher(JointState, '/joint_states', 10)
    received = []
    tf_publisher = node.create_publisher(TFMessage, '/tf', 10)
    source_ns = 1_000_000_000

    def on_limit(msg):
        sample = {'wall': time.monotonic(), 'stamp_ns': msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                  'percentage': msg.percentage, 'speed_limit': msg.speed_limit,
                  'frame': msg.header.frame_id}
        received.append(sample)
        record['received'].append(sample)

    subscription = node.create_subscription(SpeedLimit, '/navigation_policy/arm_speed_limit', on_limit, 10)

    def pump(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert process.poll() is None, 'producer exited before test completion'
            rclpy.spin_once(node, timeout_sec=min(.01, max(0., deadline - time.monotonic())))

    def await_condition(predicate, seconds=3.):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            pump(.01)
        assert predicate(), 'bounded condition did not complete'

    def sample():
        nonlocal source_ns
        source_ns += 10_000_000
        msg = JointState()
        msg.header.stamp = Time(nanoseconds=source_ns).to_msg()
        msg.name = [f'astribot_arm_{side}_joint_{index}' for side in ('left', 'right') for index in range(1, 8)]
        msg.position = [0.] * 14
        msg.velocity = [0.] * 14
        return msg

    try:
        await_condition(lambda: publisher.get_subscription_count() == 1 and bool(received), 5.)
        pubs = node.get_publisher_names_and_types_by_node('arm_chassis_speed_coupling_node', '/')
        subs = node.get_subscriber_names_and_types_by_node('arm_chassis_speed_coupling_node', '/')
        record['publisher_interfaces'] = pubs
        record['subscriber_interfaces'] = subs
        assert all('geometry_msgs/msg/Twist' not in types for _, types in pubs + subs)
        assert any(name == '/navigation_policy/arm_speed_limit' and 'nav2_msgs/msg/SpeedLimit' in types for name, types in pubs)
        assert received[-1]['speed_limit'] == 0.
        if metric == 'horizontal_reach':
            await_condition(lambda: tf_publisher.get_subscription_count() == 1)
            transforms = []
            for link in ('astribot_arm_left_tcp_link', 'astribot_arm_right_tcp_link',
                         'astribot_gripper_left_Link_L11', 'astribot_gripper_right_Link_R11'):
                tf = TransformStamped()
                tf.header.frame_id = 'astribot_torso_base'; tf.child_frame_id = link
                tf.header.stamp = Time(seconds=1).to_msg(); tf.transform.rotation.w = 1.
                transforms.append(tf)
            tf_publisher.publish(TFMessage(transforms=transforms)); pump(.08)
        msg = sample()
        capture = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        publisher.publish(msg)
        await_condition(lambda: any(v['stamp_ns'] == capture and v['speed_limit'] == 100. for v in received))
        assert received[-1]['frame'] == 'astribot_torso_base' and received[-1]['percentage']
        # Delayed source data remains available; repeated packets do not restamp it.
        end = time.monotonic() + .65
        while time.monotonic() < end:
            publisher.publish(msg)
            pump(.03)
        assert received[-1]['speed_limit'] == 100.
        assert received[-1]['stamp_ns'] == capture
        assert all(v['stamp_ns'] in (0, capture) for v in received)
        # Future-to-local-clock data is newest and accepted. Older malformed
        # data cannot overwrite it or cause a transient HOLD.
        source_ns = node.get_clock().now().nanoseconds + 2_000_000_000
        msg = sample(); publisher.publish(msg)
        capture = source_ns
        await_condition(lambda: received[-1]['stamp_ns'] == capture)
        older = JointState(); older.header.stamp = Time(seconds=1).to_msg()
        publisher.publish(older); pump(.08)
        assert received[-1]['speed_limit'] == 100. and received[-1]['stamp_ns'] == capture
        for defect in ('missing_joint', 'missing_velocity'):
            msg = sample()
            if defect == 'missing_joint':
                msg.name.pop(); msg.position.pop(); msg.velocity.pop()
            else:
                msg.velocity.pop()
            before = len(received)
            publisher.publish(msg)
            await_condition(lambda: len(received) > before)
            pump(.08)
            assert received[-1]['speed_limit'] == 0., defect
        msg = sample(); publisher.publish(msg)
        capture = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        await_condition(lambda: any(v['stamp_ns'] == capture and 0. < v['speed_limit'] <= 100. for v in received))
        record['passed'] = True
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait(timeout=5.)
            record['forced_kill'] = True
        record['exit_code'] = process.returncode
        log.close()
        (evidence / 'arm_navigation_limit_result.json').write_text(json.dumps(record, indent=2) + '\n')
