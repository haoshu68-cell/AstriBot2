#!/usr/bin/env python3
"""离线量化 ChassisBridgeCore 的 Python / C++ 全状态机候选。

基准使用同一个 FakeSession/FakePose 输入；先逐项比较最终状态和取样，再报告
整段 ``inner_tick`` 的中位数。它不启动 ROS、Gazebo 或真实 SDK。
"""

import importlib
import math
import os
import statistics
import time

from astribot_trajectory_bridge import chassis_bridge_core as bridge_core
from astribot_trajectory_bridge import chassis_integrator as integrator
from astribot_trajectory_bridge.ports import FakeClock, FakePose, FakeSession
from astribot_trajectory_bridge.chassis_bridge_core import ChassisBridgeConfig, ChassisBridgeCore
from astribot_trajectory_bridge_native import _chassis_math_native as native


TICKS = int(os.environ.get('ASTRIBOT_BENCH_TICKS', '50000'))
REPEATS = int(os.environ.get('ASTRIBOT_BENCH_REPEATS', '5'))


def make(enabled):
    old_core = bridge_core._native
    old_integrator = integrator._native
    bridge_core._native = native if enabled else None
    integrator._native = native if enabled else None
    clock = FakeClock()
    pose = FakePose(clock, [0.0, 0.0, 0.0])
    session = FakeSession(
        desired={'astribot_chassis': [0.0, 0.0, 0.0]},
        current={'astribot_chassis': [0.0, 0.0, 0.0]},
        follow_ratio=0.90)
    cfg = ChassisBridgeConfig(
        freq=250.0,
        outer_rate=10.0,
        require_fresh_scan=False,
        pose_source='ground_truth',
        cmd_vel_timeout_sec=1.0,
        leash_xy_m=0.25,
        leash_theta_rad=0.35,
        pose_preview_xy_sec=0.1,
        pose_preview_theta_sec=0.1,
        pose_preview_max_xy_m=0.2,
        pose_preview_max_theta_rad=0.34)
    core = ChassisBridgeCore(cfg, session, pose, clock)
    return core, session, clock, old_core, old_integrator


def run(enabled):
    core, session, clock, old_core, old_integrator = make(enabled)
    try:
        assert core.enable()[0]
        core.submit_twist(0.08, -0.015, 0.02)
        start = time.perf_counter()
        for _ in range(TICKS):
            clock.advance(0.004)
            if not core.inner_tick():
                raise AssertionError('core left ENABLED during benchmark')
        elapsed = time.perf_counter() - start
        trace = core.consume_vel_trace()
        stats = core.tick_stats()
        events = [(event.code, event.metric_1, event.metric_2)
                  for event in core.drain_events()]
        checksum = sum(core.pos_cmd) + trace.cmd_path + trace.dtheta_integrated
        snapshot = (core.state, tuple(core.pos_cmd), trace, stats, events,
                    len(session.set_position_calls), checksum)
        return elapsed, snapshot
    finally:
        bridge_core._native = old_core
        integrator._native = old_integrator


def close(a, b):
    if isinstance(a, (tuple, list)):
        return len(a) == len(b) and all(close(x, y) for x, y in zip(a, b))
    if isinstance(a, float):
        return math.isclose(a, b, rel_tol=2e-10, abs_tol=2e-12)
    return a == b


def main():
    py_elapsed, py_snapshot = run(False)
    native_elapsed, native_snapshot = run(True)
    if not close(py_snapshot[0:6], native_snapshot[0:6]):
        raise AssertionError('Python/native chassis snapshots diverged')
    py_times = [py_elapsed]
    native_times = [native_elapsed]
    for _ in range(max(0, REPEATS - 1)):
        py_times.append(run(False)[0])
        native_times.append(run(True)[0])
    py_median = statistics.median(py_times)
    native_median = statistics.median(native_times)
    py_rate = TICKS / py_median
    native_rate = TICKS / native_median
    print('ticks=%d repeats=%d' % (TICKS, REPEATS))
    print('python_seconds=%.6f python_ticks_per_sec=%.1f' % (py_median, py_rate))
    print('native_seconds=%.6f native_ticks_per_sec=%.1f' % (native_median, native_rate))
    print('native_delta_percent=%.2f' % ((native_median / py_median - 1.0) * 100.0))
    print('checksum=%.12e writes=%d events=%d' %
          (py_snapshot[-1], py_snapshot[-2], len(py_snapshot[-3])))


if __name__ == '__main__':
    main()
