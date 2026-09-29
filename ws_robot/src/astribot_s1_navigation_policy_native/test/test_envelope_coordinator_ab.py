#!/usr/bin/env python3
"""同一消息回放下比较 legacy 包络 Python oracle 与直接 C++ 节点。"""

import copy
import os
import subprocess
import sys
import time

import rclpy
from astribot_navigation_msgs.msg import RobotEnvelope
from astribot_navigation_msgs.srv import SetRobotEnvelope

sys.path.insert(0, os.path.dirname(__file__))
from test_envelope_coordinator_cpp_isolated import Replay, spin_until  # noqa: E402
from test_envelope_protocol import REFERENCE_ROOT, PYTHON_MAIN  # validation-only Python oracle runner


def call_case(command, domain, extra_env):
    environment = os.environ.copy()
    environment.update(extra_env)
    environment.update({"ROS_DOMAIN_ID": str(domain), "ROS_LOCALHOST_ONLY": "1"})
    process = subprocess.Popen(command, env=environment)
    os.environ.update({"ROS_DOMAIN_ID": str(domain), "ROS_LOCALHOST_ONLY": "1"})
    rclpy.init()
    replay = Replay()
    client = replay.create_client(SetRobotEnvelope, "/navigation/set_robot_envelope")
    service_results = []
    try:
        assert spin_until(replay, lambda: bool(replay.envelopes)), "no heartbeat"
        assert spin_until(replay, lambda: replay.envelopes[-1].transport_ready), "no initial ready"

        invalid = SetRobotEnvelope.Request()
        invalid.envelope = copy.deepcopy(replay.envelopes[-1])
        invalid.envelope.half_width_m -= 0.01
        assert spin_until(replay, client.service_is_ready)
        future = client.call_async(invalid)
        assert spin_until(replay, future.done)
        result = future.result()
        service_results.append((bool(result.accepted), result.reason))

        moving = SetRobotEnvelope.Request()
        moving.envelope = copy.deepcopy(replay.envelopes[-1])
        moving.envelope.transport_ready = True
        replay.moving = True
        for _ in range(8):
            rclpy.spin_once(replay, timeout_sec=0.03)
        future = client.call_async(moving)
        assert spin_until(replay, future.done)
        result = future.result()
        service_results.append((bool(result.accepted), result.reason))
        replay.moving = False
        for _ in range(8):
            rclpy.spin_once(replay, timeout_sec=0.03)

        valid = SetRobotEnvelope.Request()
        valid.envelope = copy.deepcopy(replay.envelopes[-1])
        valid.envelope.half_width_m += 0.02
        valid.envelope.transport_ready = True
        future = client.call_async(valid)
        assert spin_until(replay, future.done)
        result = future.result()
        service_results.append((bool(result.accepted), result.reason))
        assert result.accepted, result.reason
        accepted_epoch = result.epoch
        assert spin_until(replay, lambda: replay.envelopes[-1].epoch == accepted_epoch)
        assert spin_until(replay, lambda: replay.envelopes[-1].transport_ready)
        trace = [
            (bool(message.transport_ready), message.reason,
             round(message.half_length_m, 8), round(message.half_width_m, 8))
            for message in replay.envelopes
        ]
        return trace, service_results
    finally:
        replay.destroy_node()
        rclpy.shutdown()
        # The Python oracle calls rclpy.shutdown() in its finally block; a hard
        # stop here avoids treating the second shutdown as a migration failure.
        process.kill()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)


def main(cpp_binary, profile):
    cpp_trace, cpp_services = call_case(
        [cpp_binary, "--ros-args", "-p", "use_sim_time:=true", "-p", f"profile:={profile}",
         "-p", "navigation_geometry_mode:=legacy"],
        96,
        {},
    )
    python_trace, python_services = call_case(
        [sys.executable, "-c", PYTHON_MAIN,
         "--ros-args", "-p", "use_sim_time:=true", "-p", f"profile:={profile}",
         "-p", "navigation_geometry_mode:=legacy"],
        97,
        {"PYTHONPATH": str(REFERENCE_ROOT) + os.pathsep + os.path.join(os.getcwd(), "ws_robot/src/astribot_s1_navigation_policy") + os.pathsep + os.environ.get("PYTHONPATH", "")},
    )
    assert cpp_services == python_services, (cpp_services, python_services)
    # The first heartbeat count is scheduler-dependent; compare the protocol states and final geometry.
    assert cpp_trace[-1] == python_trace[-1], (cpp_trace[-1], python_trace[-1])
    assert cpp_trace[-1][0] is True and cpp_trace[-1][1] == "TRANSPORT_READY"
    assert any(not item[0] for item in cpp_trace)
    print("cpp_services=", cpp_services)
    print("python_services=", python_services)
    print("cpp_final=", cpp_trace[-1])
    print("python_final=", python_trace[-1])


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: test_envelope_coordinator_ab.py /path/to/cpp profile.json")
    main(sys.argv[1], sys.argv[2])
