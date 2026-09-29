import math
import os
import time

from astribot_s1_navigation_policy import sensor_health
from astribot_s1_navigation_policy.contracts import BearingCone, Vec3
from astribot_s1_navigation_policy_native import _navigation_math_native as native


def _measure(function, repeats):
    start = time.perf_counter()
    checksum = 0.0
    for _ in range(repeats):
        result = function()
        if isinstance(result, bool):
            checksum += float(result)
        elif result:
            checksum += float(len(result))
    return time.perf_counter() - start, checksum


def main():
    ranges = [2.0 if index % 17 else float('nan') for index in range(360)]
    cones = tuple(BearingCone(
        Vec3(math.cos(index * math.pi / 16.0),
             math.sin(index * math.pi / 16.0), 0.0), math.pi / 8.0)
        for index in range(16))
    motion = (0.4, 0.1, 0.0)
    repeats = int(os.environ.get('BENCH_REPEATS', '20000'))
    old = sensor_health._native
    try:
        sensor_health._native = None
        py_scan, scan_checksum = _measure(
            lambda: sensor_health.scan_coverage(
                ranges, 0.1, 5.0, -math.pi, math.pi / 180.0, 0.0), repeats)
        py_dirs, dirs_checksum = _measure(
            lambda: sensor_health.movement_directions(*motion), repeats)
        py_cover, cover_checksum = _measure(
            lambda: sensor_health.coverage_allows_motion(cones, *motion), repeats)
        sensor_health._native = native
        native_scan, _ = _measure(
            lambda: sensor_health.scan_coverage(
                ranges, 0.1, 5.0, -math.pi, math.pi / 180.0, 0.0), repeats)
        native_dirs, _ = _measure(
            lambda: sensor_health.movement_directions(*motion), repeats)
        native_cover, _ = _measure(
            lambda: sensor_health.coverage_allows_motion(cones, *motion), repeats)
    finally:
        sensor_health._native = old
    print(f'repeats={repeats} beams={len(ranges)} cones={len(cones)}')
    print(f'python_scan_seconds={py_scan:.6f} native_scan_seconds={native_scan:.6f} '
          f'native_scan_delta_percent={(native_scan / py_scan - 1.0) * 100.0:.2f}')
    print(f'python_directions_seconds={py_dirs:.6f} native_directions_seconds={native_dirs:.6f} '
          f'native_directions_delta_percent={(native_dirs / py_dirs - 1.0) * 100.0:.2f}')
    print(f'python_coverage_seconds={py_cover:.6f} native_coverage_seconds={native_cover:.6f} '
          f'native_coverage_delta_percent={(native_cover / py_cover - 1.0) * 100.0:.2f}')
    print(f'checksums={scan_checksum:.6e},{dirs_checksum:.6e},{cover_checksum:.6e}')


if __name__ == '__main__':
    main()
