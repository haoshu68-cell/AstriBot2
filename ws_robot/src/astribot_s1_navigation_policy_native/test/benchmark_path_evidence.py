#!/usr/bin/env python3
"""量化路径证据裁决的 Python/native facade。"""

import time
from types import SimpleNamespace

from astribot_s1_navigation_policy import path_evidence
from astribot_s1_navigation_policy_native import _navigation_math_native as native


PROFILE = SimpleNamespace(path_risk_timeout_s=0.5, max_speed_m_s=0.7,
                          clearance_margin_m=0.08,
                          payload_extra_margin_m=0.02)
EVIDENCE = path_evidence.PathEvidence(
    path_key='p', stamp_s=10.0, received_wall_s=100.0, epoch=1,
    known=True, blocked=True, distance_m=1.2)
COUNT = 200000


def _run(use_native):
    old = path_evidence._native
    path_evidence._native = native if use_native else None
    try:
        start = time.perf_counter()
        checksum = 0.0
        for i in range(COUNT):
            result = path_evidence.assess_path(
                EVIDENCE, 'p', 10.1 + 0.00001 * (i % 10),
                100.1 + 0.00001 * (i % 10), 1, False, PROFILE)
            checksum += float(result.blocked) + result.conflict_time_s + result.distance_m
        return time.perf_counter() - start, checksum
    finally:
        path_evidence._native = old


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
