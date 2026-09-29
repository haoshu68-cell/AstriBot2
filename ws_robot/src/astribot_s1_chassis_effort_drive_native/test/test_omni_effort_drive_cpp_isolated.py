#!/usr/bin/env python3
"""消息级回放：验证直接 C++ 力矩节点的输出和两个超时停车门槛。"""

import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINTS = ["wheel_RF_Joint", "wheel_LF_Joint", "wheel_RR_Joint", "wheel_LR_Joint"]


class Replay(Node):
    def __init__(self):
        super().__init__("omni_effort_drive_cpp_replay")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.last_effort = None
        self.create_subscription(Float64MultiArray, "/wheel_effort_controller/commands",
                                 self._on_effort, 10)
        self.phase = "active"
        self.started = time.monotonic()
        self.create_timer(0.05, self._publish_joint)
        self.create_timer(0.05, self._publish_cmd)

    def _publish_joint(self):
        if time.monotonic() - self.started >= 1.2:
            self.phase = "feedback_stale"
            return
        message = JointState()
        message.name = JOINTS
        message.position = [0.0] * 4
        message.velocity = [0.0] * 4
        self.joint_pub.publish(message)

    def _publish_cmd(self):
        elapsed = time.monotonic() - self.started
        if elapsed < 0.6:
            message = Twist()
            message.linear.x = 0.1
            self.cmd_pub.publish(message)
            self.phase = "active"
        else:
            self.phase = "command_stale"

    def _on_effort(self, message):
        self.last_effort = list(message.data)


def main(binary):
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = "91"
    environment["ROS_LOCALHOST_ONLY"] = "1"
    os.environ.update({"ROS_DOMAIN_ID": "91", "ROS_LOCALHOST_ONLY": "1"})
    process = subprocess.Popen([
        binary, "--ros-args",
        "-p", "use_sim_time:=false",
        "-p", "cmd_vel_timeout_sec:=0.2",
        "-p", "joint_state_timeout_sec:=0.3",
        "-p", "control_period_sec:=0.01",
    ], env=environment)
    rclpy.init()
    replay = Replay()
    try:
        deadline = time.monotonic() + 5.0
        active = None
        while time.monotonic() < deadline:
            rclpy.spin_once(replay, timeout_sec=0.05)
            if replay.phase == "active" and replay.last_effort is not None:
                active = replay.last_effort
                if any(abs(value) > 1e-6 for value in active):
                    break
        assert active is not None and len(active) == 4, active
        assert any(abs(value) > 1e-6 for value in active), active
        assert active[0] * active[1] < 0.0 and active[2] * active[3] < 0.0, active
        assert abs(active[0] - active[2]) < 1e-9, active
        assert abs(active[1] - active[3]) < 1e-9, active

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(replay, timeout_sec=0.05)
        assert replay.phase in ("command_stale", "feedback_stale")
        assert replay.last_effort is not None
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and replay.phase != "feedback_stale":
            rclpy.spin_once(replay, timeout_sec=0.05)
        assert replay.phase == "feedback_stale"
        deadline = time.monotonic() + 0.6
        while time.monotonic() < deadline:
            rclpy.spin_once(replay, timeout_sec=0.05)
        assert replay.last_effort is not None
        assert all(abs(value) < 1e-9 for value in replay.last_effort), replay.last_effort
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
    if len(sys.argv) != 2:
        raise SystemExit("usage: test_omni_effort_drive_cpp_isolated.py /path/to/omni_effort_drive_cpp")
    main(sys.argv[1])
