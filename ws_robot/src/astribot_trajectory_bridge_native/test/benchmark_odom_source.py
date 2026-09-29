#!/usr/bin/env python3
"""量化 ChassisOdomSource 的 Python/native 只读状态回放。"""

import os
import sys
import time

import astribot_trajectory_bridge.chassis_odom_source as odom
from astribot_trajectory_bridge_native import _chassis_math_native as native


COUNT = int(os.environ.get('ODOM_BENCH_COUNT', '200000'))
REPEATS = int(os.environ.get('ODOM_BENCH_REPEATS', '5'))


def _cases():
    return [((0.001 * (i % 100), -0.0007 * (i % 80), 0.01 * (i % 31)),
             (0.2 + 0.0001 * (i % 19), -0.1 + 0.0002 * (i % 23),
              0.03 - 0.0001 * (i % 17))) for i in range(COUNT)]


def _run(cases, use_native):
    old = odom._native_odom
    odom._native_odom = native if use_native else None
    try:
        source = odom.ChassisOdomSource(0.3, 'world')
        checksum = 0.0
        start = time.perf_counter()
        for pos, vel in cases:
            row = source.sample(pos, vel)
            checksum += row.x + row.y + row.theta + row.vx_body + row.vy_body
            checksum += row.wz + row.jump_m + float(row.jumped)
        elapsed = time.perf_counter() - start
        return elapsed, checksum, source.stats.samples, source.stats.jumps
    finally:
        odom._native_odom = old


def _median(values):
    values = sorted(values)
    return values[len(values) // 2]


def main():
    cases = _cases()
    py_runs = [_run(cases, False) for _ in range(REPEATS)]
    native_runs = [_run(cases, True) for _ in range(REPEATS)]
    py_seconds = _median([row[0] for row in py_runs])
    native_seconds = _median([row[0] for row in native_runs])
    py_checksum = py_runs[0][1]
    native_checksum = native_runs[0][1]
    assert abs(py_checksum - native_checksum) < 1e-8
    assert py_runs[0][2:] == native_runs[0][2:]
    print(f'count={COUNT} repeats={REPEATS}')
    print(f'python_seconds={py_seconds:.6f} python_samples_per_sec={COUNT / py_seconds:.1f}')
    print(f'native_seconds={native_seconds:.6f} native_samples_per_sec={COUNT / native_seconds:.1f}')
    print(f'native_delta_percent={(native_seconds / py_seconds - 1.0) * 100.0:.2f}')
    print(f'checksum={py_checksum:.12e} jumps={py_runs[0][3]}')


if __name__ == '__main__':
    sys.exit(main())
