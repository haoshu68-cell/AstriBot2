#!/usr/bin/env python3
"""量化 lateral_variants 的整段 Python/native facade。"""

import time

from astribot_s1_navigation_policy import candidate_variants
from astribot_s1_navigation_policy_native import _navigation_math_native as native


ROUTE = [(0.2 * i, 0.15 * ((i % 5) - 2)) for i in range(24)]
COUNT = 50000


def _run(use_native):
    old = candidate_variants._native
    candidate_variants._native = native if use_native else None
    try:
        start = time.perf_counter()
        checksum = 0.0
        for _ in range(COUNT):
            variants = candidate_variants.lateral_variants(ROUTE, 0.2)
            checksum += sum(point[0] + point[1]
                            for variant in variants for point in variant)
        return time.perf_counter() - start, checksum
    finally:
        candidate_variants._native = old


def main():
    py_seconds, py_checksum = _run(False)
    native_seconds, native_checksum = _run(True)
    assert abs(py_checksum - native_checksum) < 1e-7
    print(f'iterations={COUNT} route_points={len(ROUTE)}')
    print(f'python_seconds={py_seconds:.6f}')
    print(f'native_seconds={native_seconds:.6f}')
    print(f'native_delta_percent={(native_seconds / py_seconds - 1.0) * 100.0:.2f}')
    print(f'checksum={py_checksum:.12e}')


if __name__ == '__main__':
    main()
