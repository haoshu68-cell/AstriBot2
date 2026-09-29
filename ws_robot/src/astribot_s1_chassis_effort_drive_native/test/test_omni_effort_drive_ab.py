#!/usr/bin/env python3
"""Run identical command/feedback replay against Python and C++ nodes."""

from pathlib import Path as _ReferencePath
import sys as _reference_sys
_reference_sys.path.insert(0, str(_ReferencePath(__file__).resolve().parents[4] / "tools/migration/python_reference"))
from reference_bootstrap import REFERENCE_ROOT, enable as _enable_references
_enable_references()

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

# The old module is an import-only oracle. Only this replay harness starts it.
PYTHON_REFERENCE = """from reference_bootstrap import enable
enable()

import rclpy
from astribot_s1_chassis_effort_drive.omni_effort_drive_node import OmniEffortDriveNode
rclpy.init()
node = OmniEffortDriveNode()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
"""


class Replay(Node):
    def __init__(self, domain):
        super().__init__("omni_effort_drive_ab_replay")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.samples = []
        self.started = time.monotonic()
        self.create_subscription(Float64MultiArray, "/wheel_effort_controller/commands",
                                 self._on_effort, 10)
        self.create_timer(0.05, self._publish_joint)
        self.create_timer(0.05, self._publish_cmd)

    def _elapsed(self):
        return time.monotonic() - self.started

    def _publish_joint(self):
        if self._elapsed() >= 1.2:
            return
        message = JointState()
        message.name = JOINTS
        message.position = [0.0] * 4
        message.velocity = [0.0] * 4
        self.joint_pub.publish(message)

    def _publish_cmd(self):
        if self._elapsed() >= 0.6:
            return
        message = Twist()
        message.linear.x = 0.1
        self.cmd_pub.publish(message)

    def _on_effort(self, message):
        self.samples.append((self._elapsed(), list(message.data)))


def _run(command, domain, python_path=None):
    env = os.environ.copy()
    env.update({"ROS_DOMAIN_ID": str(domain), "ROS_LOCALHOST_ONLY": "1"})
    if python_path is not None:
        env["PYTHONPATH"] = str(REFERENCE_ROOT) + ":" + python_path + ":" + env.get("PYTHONPATH", "")
    args = list(command) + [
        "--ros-args", "-p", "use_sim_time:=false",
        "-p", "cmd_vel_timeout_sec:=0.2",
        "-p", "joint_state_timeout_sec:=0.3",
        "-p", "control_period_sec:=0.01",
    ]
    process = subprocess.Popen(args, env=env)
    os.environ.update({"ROS_DOMAIN_ID": str(domain), "ROS_LOCALHOST_ONLY": "1"})
    rclpy.init()
    replay = Replay(domain)
    try:
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            rclpy.spin_once(replay, timeout_sec=0.02)
            if any(t < 0.6 and any(abs(v) > 1e-6 for v in values)
                   for t, values in replay.samples):
                break
        deadline = time.monotonic() + 1.8
        while time.monotonic() < deadline:
            rclpy.spin_once(replay, timeout_sec=0.02)
        active = [values for t, values in replay.samples if t < 0.6]
        stale = [values for t, values in replay.samples if t > 1.7]
        assert active and all(len(values) == 4 for values in active), replay.samples
        assert any(any(abs(v) > 1e-6 for v in values) for values in active), active
        assert stale and all(all(abs(v) < 1e-9 for v in values) for values in stale), stale[-3:]
        active_nonzero = [values for values in active if any(abs(v) > 1e-6 for v in values)]
        assert active_nonzero, active
        return active_nonzero[0], active_nonzero
    finally:
        replay.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)


def main(cpp_binary):
    python_path = ":".join([
        "/opt/ros/humble/local/lib/python3.10/dist-packages",
        "/opt/ros/humble/lib/python3.10/site-packages",
        "ws_robot/src/astribot_s1_chassis_effort_drive",
    ])
    cpp, cpp_trace = _run([cpp_binary], 92)
    python, python_trace = _run([sys.executable, "-c", PYTHON_REFERENCE],
                                93, python_path)
    assert len(cpp) == len(python) == 4
    print("cpp_trace=", [[round(v, 6) for v in row] for row in cpp_trace[:8]])
    print("python_trace=", [[round(v, 6) for v in row] for row in python_trace[:8]])
    for lhs, rhs in zip(cpp, python):
        assert abs(lhs - rhs) <= 1e-6 + 1e-4 * max(abs(lhs), abs(rhs)), (cpp, python)
    print("cpp_active=", ["%.9f" % value for value in cpp])
    print("python_active=", ["%.9f" % value for value in python])


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: test_omni_effort_drive_ab.py /path/to/omni_effort_drive_cpp")
    main(sys.argv[1])
