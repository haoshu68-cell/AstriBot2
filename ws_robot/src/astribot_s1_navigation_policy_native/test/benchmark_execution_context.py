#!/usr/bin/env python3
"""量化 ExecutionContext 的 Python/native facade。"""

import time

from astribot_s1_navigation_policy import execution_context
from astribot_s1_navigation_policy_native import _navigation_math_native as native


COUNT = 100000


def _run(use_native):
    old = execution_context._native
    execution_context._native = native if use_native else None
    try:
        context = execution_context.ExecutionContext()
        start = time.perf_counter()
        checksum = 0.0
        for i in range(COUNT):
            context.task('goal', 'EXECUTING', i + 1)
            context.map(('map', i % 10), bytes([i % 251]))
            context.localization((0.001 * i, 0.0, 0.001 * (i % 7)), 0.2, 0.15)
            context.path()
            checksum += context.version.path_revision + context.version.map_epoch
        return time.perf_counter() - start, checksum
    finally:
        execution_context._native = old


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
