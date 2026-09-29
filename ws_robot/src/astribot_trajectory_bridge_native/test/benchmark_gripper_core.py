#!/usr/bin/env python3
"""Isolated Python/native gripper-controller benchmark.

This intentionally uses a non-blocking fake SDK.  It measures controller and
pybind overhead only; it is not a claim about the vendor SDK's blocking time.
"""

import statistics
import time

from astribot_trajectory_bridge import gripper_core
from astribot_trajectory_bridge_native import _chassis_math_native as native


class QuietSession:
    in_simulation = True

    def open_effector(self, names=None, duration=1.0):
        return None

    def close_effector(self, names=None, duration=1.0):
        return None

    def set_effector_max_force(self, names, max_force):
        return None

    def get_current_joints_position(self, names):
        return [[0.0] for _ in names]


class ZeroClock:
    def now(self):
        return 0.0


def run(enabled, count=100_000, repeats=7):
    old = gripper_core._native
    gripper_core._native = native if enabled else None
    try:
        cfg = gripper_core.GripperConfig(
            gripper_names=['g'], default_duration_sec=0.2,
            stream_freq=20.0, mid_stream_tolerance=0.5,
            mid_stream_timeout_sec=0.25)
        request = dict(name='g', opening_fraction=0.0, duration=0.01,
                       use_raw_cmd=False, raw_cmd=0.0, max_force=0.0,
                       write_allowed=True)
        samples = []
        checksum = 0.0
        for _ in range(repeats):
            controller = gripper_core.GripperController(
                cfg, QuietSession(), sleep_fn=lambda _seconds: None,
                clock_fn=ZeroClock().now, in_simulation=True)
            start = time.perf_counter()
            checksum = 0.0
            for _ in range(count):
                result = controller.execute(**request)
                checksum += result.dispatched_cmd + result.actual_cmd
            samples.append(time.perf_counter() - start)
        median = statistics.median(samples)
        return median, count / median, checksum
    finally:
        gripper_core._native = old


if __name__ == '__main__':
    for enabled in (False, True):
        elapsed, rate, checksum = run(enabled)
        print('%s %.6f s %.0f/s checksum=%.9e' % (
            'native' if enabled else 'python', elapsed, rate, checksum))
