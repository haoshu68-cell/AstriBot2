#!/usr/bin/env python3
"""消息级回放：验证包络 C++ 节点的停车、双 footprint 确认和服务拒绝门槛。"""

import os
import copy
import signal
import subprocess
import sys
import time

import rclpy
from astribot_navigation_msgs.msg import RobotEnvelope
from astribot_navigation_msgs.srv import SetRobotEnvelope
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


FRAME = "astribot_torso_base"


class Replay(Node):
    def __init__(self):
        super().__init__("envelope_coordinator_cpp_replay")
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.acks = {
            name: self.create_publisher(PolygonStamped, f"/{name}/published_footprint", 10)
            for name in ("global_costmap", "local_costmap")
        }
        self.envelopes = []
        self.last_shape = {}
        self.last_clock_ns = 1_000_000_000
        self.moving = False
        self.create_subscription(RobotEnvelope, "/navigation/robot_envelope", self._on_envelope, 10)
        from geometry_msgs.msg import Polygon
        for name in self.acks:
            self.create_subscription(
                Polygon,
                f"/{name}/footprint",
                lambda message, n=name: self._on_footprint(n, message),
                10,
            )
        self.create_timer(0.02, self._publish)

    def _stamp(self):
        stamp = type(Clock().clock)()
        stamp.sec = int(self.last_clock_ns // 1_000_000_000)
        stamp.nanosec = int(self.last_clock_ns % 1_000_000_000)
        return stamp

    def _publish(self):
        self.last_clock_ns += 20_000_000
        clock = Clock()
        clock.clock.sec = int(self.last_clock_ns // 1_000_000_000)
        clock.clock.nanosec = int(self.last_clock_ns % 1_000_000_000)
        self.clock_pub.publish(clock)
        odom = Odometry()
        odom.header.stamp = clock.clock
        odom.twist.twist.linear.x = 0.12 if self.moving else 0.0
        self.odom_pub.publish(odom)

    def _on_footprint(self, name, polygon):
        self.last_shape[name] = polygon
        if self.moving:
            return
        message = PolygonStamped()
        message.header.stamp = self._stamp()
        message.header.frame_id = FRAME
        message.polygon = polygon
        self.acks[name].publish(message)

    def _on_envelope(self, message):
        self.envelopes.append(message)


def spin_until(node, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def main(binary, profile):
    environment = os.environ.copy()
    environment.update({"ROS_DOMAIN_ID": "95", "ROS_LOCALHOST_ONLY": "1"})
    os.environ.update({"ROS_DOMAIN_ID": "95", "ROS_LOCALHOST_ONLY": "1"})
    process = subprocess.Popen(
        [
            binary,
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            f"profile:={profile}",
            "-p",
            "navigation_geometry_mode:=legacy",
        ],
        env=environment,
    )
    rclpy.init()
    replay = Replay()
    client = replay.create_client(SetRobotEnvelope, "/navigation/set_robot_envelope")
    try:
        assert spin_until(replay, lambda: bool(replay.envelopes)), "no envelope heartbeat"
        assert replay.envelopes[-1].transport_ready is False
        assert spin_until(replay, lambda: replay.envelopes[-1].transport_ready), replay.envelopes[-1]

        invalid = SetRobotEnvelope.Request()
        invalid.envelope = copy.deepcopy(replay.envelopes[-1])
        invalid.envelope.half_length_m -= 0.01
        assert spin_until(replay, client.service_is_ready), "service unavailable"
        future = client.call_async(invalid)
        assert spin_until(replay, future.done), "invalid service timeout"
        assert future.result() is not None and not future.result().accepted
        assert "undercut" in future.result().reason

        moving = SetRobotEnvelope.Request()
        moving.envelope = copy.deepcopy(replay.envelopes[-1])
        moving.envelope.transport_ready = True
        replay.moving = True
        # Let a fresh moving odometry sample cross the ROS boundary before the service call.
        for _ in range(8):
            rclpy.spin_once(replay, timeout_sec=0.03)
        future = client.call_async(moving)
        assert spin_until(replay, future.done), "moving service timeout"
        assert future.result() is not None and not future.result().accepted
        assert future.result().reason == "ROBOT_MUST_BE_STOPPED_WITH_FRESH_ODOMETRY", future.result().reason
        replay.moving = False
        for _ in range(8):
            rclpy.spin_once(replay, timeout_sec=0.03)

        valid = SetRobotEnvelope.Request()
        valid.envelope = copy.deepcopy(replay.envelopes[-1])
        valid.envelope.half_length_m += 0.02
        valid.envelope.transport_ready = True
        future = client.call_async(valid)
        assert spin_until(replay, future.done), "valid service timeout"
        assert future.result() is not None and future.result().accepted, future.result().reason
        accepted_epoch = future.result().epoch
        assert spin_until(replay, lambda: replay.envelopes[-1].epoch == accepted_epoch)
        assert spin_until(replay, lambda: replay.envelopes[-1].transport_ready)
    finally:
        replay.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: test_envelope_coordinator_cpp_isolated.py /path/to/node profile.json")
    main(sys.argv[1], sys.argv[2])
