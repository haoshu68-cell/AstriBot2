#!/usr/bin/env python3
"""量化 risk 路径位置批处理的 Python/C++ 同输入吞吐。"""

import math
import random
import statistics
import time

from astribot_s1_navigation_policy import risk
from astribot_s1_navigation_policy_native import _navigation_math_native as native


COUNT = 100000
REPEATS = 5


def main():
    rng = random.Random(20260922)
    path = [(rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0))
            for _ in range(24)]
    distances = [rng.uniform(-0.2, 8.0) for _ in range(COUNT)]
    fallback = risk.RobotState(0.4, -0.3, 0.7)

    def run(use_native):
        old = risk._native
        risk._native = native if use_native else None
        try:
            start = time.perf_counter()
            values = risk._path_position_batch(path, distances, fallback)
            elapsed = time.perf_counter() - start
            checksum = float(values.sum())
            if not math.isfinite(checksum):
                raise AssertionError('non-finite forecast checksum')
            return elapsed, checksum
        finally:
            risk._native = old

    py_times, native_times = [], []
    _, py_checksum = run(False)
    _, native_checksum = run(True)
    if not math.isclose(py_checksum, native_checksum, rel_tol=2e-10, abs_tol=2e-10):
        raise AssertionError('Python/native forecast checksum diverged')
    for _ in range(REPEATS):
        py_times.append(run(False)[0])
        native_times.append(run(True)[0])
    py_median = statistics.median(py_times)
    native_median = statistics.median(native_times)
    print('distances=%d repeats=%d' % (COUNT, REPEATS))
    print('python_seconds=%.6f python_positions_per_sec=%.1f' %
          (py_median, COUNT / py_median))
    print('native_seconds=%.6f native_positions_per_sec=%.1f' %
          (native_median, COUNT / native_median))
    print('native_delta_percent=%.2f' %
          ((native_median / py_median - 1.0) * 100.0))
    print('checksum=%.12e' % py_checksum)


if __name__ == '__main__':
    main()
