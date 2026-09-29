#!/usr/bin/env python3
"""量化 YieldPolicy 状态机的 Python/native facade。"""

import time
from types import SimpleNamespace

from astribot_s1_navigation_policy import behavior
from astribot_s1_navigation_policy_native import _navigation_math_native as native


PROFILE = SimpleNamespace(max_speed_m_s=0.7, narrow_speed_m_s=0.2,
                          reaction_time_s=0.18,
                          brake_deceleration_m_s2=0.72,
                          angular_brake_deceleration_rad_s2=1.3,
                          linear_stop_delay_s=0.04,
                          wait_budget_s=2.0, clear_hold_s=0.3)
RISK = SimpleNamespace(immediate=False, blocked=True, uncertain=False,
                       conflict_time_s=0.2)
COUNT = 100000


def _run(use_native):
    old = behavior._native
    behavior._native = native if use_native else None
    try:
        policy = behavior.YieldPolicy(PROFILE)
        start = time.perf_counter()
        checksum = 0.0
        for i in range(COUNT):
            result = policy.select(RISK if i % 7 else None, True,
                                   10.0 + 0.02 * i)
            checksum += result.speed + result.episode + float(result.motion == 'HOLD')
        return time.perf_counter() - start, checksum
    finally:
        behavior._native = old


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
