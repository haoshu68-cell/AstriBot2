#!/usr/bin/env python3
"""同输入量化 continuous_sweep.motion_clearance 的 Python/C++ 矩形路径。"""

import math
import random
import statistics
import time
from types import SimpleNamespace

import numpy as np

from astribot_s1_navigation_policy import continuous_sweep
from astribot_s1_navigation_policy_native import _navigation_math_native as native


COUNT = 1000
REPEATS = 5
PROFILE = SimpleNamespace(
    half_length_m=0.36,
    half_width_m=0.27,
    clearance_margin_m=0.08,
    payload_extra_margin_m=0.01,
)


def inputs():
    rng = random.Random(20260922)
    begin = np.asarray([rng.uniform(0.0, 1.0) for _ in range(COUNT)])
    end = begin + np.asarray([rng.uniform(0.0, 0.4) for _ in range(COUNT)])
    centers = np.asarray([[rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0)]
                          for _ in range(COUNT)])
    half = np.asarray([[rng.uniform(0.05, 0.4), rng.uniform(0.05, 0.4)]
                       for _ in range(COUNT)])
    return ([0.32, -0.11, 0.18], begin, end, centers - half,
            centers + half, PROFILE, (0.1, -0.2, 0.3))


def run(use_native, args):
    old = continuous_sweep._native
    continuous_sweep._native = native if use_native else None
    try:
        start = time.perf_counter()
        result = continuous_sweep.motion_clearance(*args)
        elapsed = time.perf_counter() - start
        return elapsed, float(np.sum(result[np.isfinite(result)]))
    finally:
        continuous_sweep._native = old


def main():
    args = inputs()
    py_times, native_times = [], []
    py_elapsed, py_checksum = run(False, args)
    native_elapsed, native_checksum = run(True, args)
    if not np.allclose(
            continuous_sweep.motion_clearance(*args),
            continuous_sweep.motion_clearance(*args),
            rtol=1e-10, atol=1e-10):
        raise AssertionError('unexpected baseline instability')
    for _ in range(REPEATS):
        py_times.append(run(False, args)[0])
        elapsed, checksum = run(True, args)
        native_times.append(elapsed)
        if not math.isclose(checksum, py_checksum, rel_tol=2e-9, abs_tol=2e-9):
            raise AssertionError('Python/native motion clearance checksum diverged')
    py_median = statistics.median(py_times)
    native_median = statistics.median(native_times)
    print('intervals=%d repeats=%d' % (COUNT, REPEATS))
    print('python_seconds=%.6f python_intervals_per_sec=%.1f' %
          (py_median, COUNT / py_median))
    print('native_seconds=%.6f native_intervals_per_sec=%.1f' %
          (native_median, COUNT / native_median))
    print('native_delta_percent=%.2f' %
          ((native_median / py_median - 1.0) * 100.0))
    print('checksum=%.12e' % py_checksum)


if __name__ == '__main__':
    main()
