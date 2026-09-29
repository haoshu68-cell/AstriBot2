#!/usr/bin/env python3
"""Isolated Python/native ArmTrajExecutor benchmark with a fake SDK."""

import statistics
import time

from astribot_trajectory_bridge import arm_bridge_core
from astribot_trajectory_bridge_native import _chassis_math_native as native


class Clock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def advance(self, seconds):
        self.t += seconds


class Session:
    def __init__(self):
        self.actual = [0.0, 0.0, 0.0]

    def get_joints_position_limit(self, _parts):
        return [[-2.0, -2.0, -2.0]], [[2.0, 2.0, 2.0]]

    def get_current_joints_position(self, _parts):
        return [list(self.actual)]

    def set_joints_position(self, _parts, positions, **_kwargs):
        target = list(positions[0])
        self.actual = [a + 0.65 * (q - a)
                       for a, q in zip(self.actual, target)]


def run(enabled, count=50_000, repeats=7):
    old = arm_bridge_core._native
    arm_bridge_core._native = native if enabled else None
    try:
        samples = []
        checksum = 0.0
        for _ in range(repeats):
            clock = Clock()
            session = Session()
            cfg = arm_bridge_core.ArmBridgeConfig(
                joint_names=['j0', 'j1', 'j2'], stream_freq=500.0,
                max_tracking_error_rad=10.0, settle_timeout_sec=2.0,
                max_traj_duration_sec=2000.0)
            executor = arm_bridge_core.ArmTrajExecutor(cfg, session, clock)
            assert executor.load_limits()[0]
            assert executor.start(
                ['j0', 'j1', 'j2'], [0.0, 1000.0],
                [[0.0, 0.0, 0.0], [1.0, -1.0, 0.5]],
                [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])[0]
            start = time.perf_counter()
            checksum = 0.0
            for _ in range(count):
                clock.advance(0.002)
                executor.step()
                checksum += session.actual[0]
            samples.append(time.perf_counter() - start)
        median = statistics.median(samples)
        return median, count / median, checksum
    finally:
        arm_bridge_core._native = old


if __name__ == '__main__':
    for enabled in (False, True):
        elapsed, rate, checksum = run(enabled)
        print('%s %.6f s %.0f/s checksum=%.9e' % (
            'native' if enabled else 'python', elapsed, rate, checksum))
