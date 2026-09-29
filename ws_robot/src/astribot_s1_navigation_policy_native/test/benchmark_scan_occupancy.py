#!/usr/bin/env python3
"""量化 scan_occupancy 的 Python/native 批处理路径。"""

from pathlib import Path as _ReferencePath
import sys as _reference_sys
_reference_sys.path.insert(0, str(_ReferencePath(__file__).resolve().parents[4] / "tools/migration/python_reference"))
from reference_bootstrap import enable as _enable_references
_enable_references()

import time

from astribot_s1_navigation_policy import scan_occupancy
from astribot_s1_navigation_policy_native import _navigation_math_native as native


POINTS = [(0.01 * (i % 160) - 0.8, 0.01 * (i % 120) - 0.6)
          for i in range(10000)]
RANGES = [2.0 + 0.001 * (i % 17) for i in range(720)]
CORNERS = [(1.0, -0.2), (1.0, 0.2), (1.4, 0.2), (1.4, -0.2)]


def _run(use_native):
    old = scan_occupancy._native
    scan_occupancy._native = native if use_native else None
    try:
        start = time.perf_counter()
        checksum = 0.0
        for _ in range(20):
            cells = scan_occupancy.occupied_cells(POINTS, 0.05)
            free = scan_occupancy.angular_box_free(
                CORNERS, RANGES, 0.1, 4.0, -3.141592653589793,
                2.0 * 3.141592653589793 / len(RANGES), 0.05)
            checksum += len(cells) + float(free)
        return time.perf_counter() - start, checksum
    finally:
        scan_occupancy._native = old


def main():
    py_seconds, py_checksum = _run(False)
    native_seconds, native_checksum = _run(True)
    assert py_checksum == native_checksum
    print('points=10000 repeats=20')
    print(f'python_seconds={py_seconds:.6f}')
    print(f'native_seconds={native_seconds:.6f}')
    print(f'native_delta_percent={(native_seconds / py_seconds - 1.0) * 100.0:.2f}')
    print(f'checksum={py_checksum:.12e}')


if __name__ == '__main__':
    main()
