#!/usr/bin/env python3
"""量化姿态监控纯状态核的 Python/native facade。"""

from pathlib import Path as _ReferencePath
import sys as _reference_sys
_reference_sys.path.insert(0, str(_ReferencePath(__file__).resolve().parents[4] / "tools/migration/python_reference"))
from reference_bootstrap import REFERENCE_ROOT, enable as _enable_references
_enable_references()

import time

from astribot_s1_navigation import posture_monitor_policy as policy
from astribot_s1_navigation_policy_native import _navigation_math_native as native


SAMPLES = [(0.6 + 0.0001 * (i % 3), 0.001 * (i % 5),
            -0.001 * (i % 7)) for i in range(20)]
COUNT = 200000


def _run(use_native):
    old = policy._native
    policy._native = native if use_native else None
    try:
        start = time.perf_counter()
        checksum = 0.0
        for i in range(COUNT):
            tripped, _ = policy.posture_out_of_bounds(
                0.6 + 0.0001 * (i % 3), 0.001 * (i % 5),
                -0.001 * (i % 7), 0.6, 0.08, 0.2)
            action, _ = policy.evaluate_posture(
                True, SAMPLES, 0.6, 0.0, 0.0, 0.6, 0.08, 0.2)
            checksum += float(tripped) + float(action == policy.ACT_PASS)
        return time.perf_counter() - start, checksum
    finally:
        policy._native = old


def main():
    py_seconds, py_checksum = _run(False)
    native_seconds, native_checksum = _run(True)
    assert py_checksum == native_checksum
    print(f'iterations={COUNT}')
    print(f'python_seconds={py_seconds:.6f}')
    print(f'native_seconds={native_seconds:.6f}')
    print(f'native_delta_percent={(native_seconds / py_seconds - 1.0) * 100.0:.2f}')
    print(f'checksum={py_checksum:.12e}')


if __name__ == '__main__':
    main()
