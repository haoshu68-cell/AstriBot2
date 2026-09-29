#!/usr/bin/env python3
"""量化 ControlTime watchdog 的 Python/native facade。"""

from pathlib import Path as _ReferencePath
import sys as _reference_sys
_reference_sys.path.insert(0, str(_ReferencePath(__file__).resolve().parents[4] / "tools/migration/python_reference"))
from reference_bootstrap import enable as _enable_references
_enable_references()

import time

from astribot_s1_navigation_policy import control_time
from astribot_s1_navigation_policy_native import _navigation_math_native as native


COUNT = 200000


def _run(use_native):
    old = control_time._native_time
    control_time._native_time = native if use_native else None
    try:
        clock = control_time.ControlTime(True, 0.5)
        start = time.perf_counter()
        checksum = 0.0
        for i in range(COUNT):
            ros = 10.0 + 0.004 * i
            wall = 100.0 + 0.004 * i
            step = clock.advance(ros, wall)
            checksum += step.now + step.dt + step.wall_dt
            checksum += float(clock.accepts(ros))
            checksum += float(clock.fresh(ros, wall, 0.3, ros, wall))
            checksum += float(clock.command_fresh(ros, wall, 0.3, ros, wall))
        return time.perf_counter() - start, checksum
    finally:
        control_time._native_time = old


def main():
    py_seconds, py_checksum = _run(False)
    native_seconds, native_checksum = _run(True)
    assert abs(py_checksum - native_checksum) < 1e-7
    print(f'iterations={COUNT}')
    print(f'python_seconds={py_seconds:.6f}')
    print(f'native_seconds={native_seconds:.6f}')
    print(f'native_delta_percent={(native_seconds / py_seconds - 1.0) * 100.0:.2f}')
    print(f'checksum={py_checksum:.12e}')


if __name__ == '__main__':
    main()
