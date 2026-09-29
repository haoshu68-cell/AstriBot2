import os
import time

from astribot_s1_navigation_policy import planning_session
from astribot_s1_navigation_policy.contracts import Planning, Stamp, Trigger, Version
from astribot_s1_navigation_policy_native import _navigation_math_native as native


def _run(session, repeats):
    kinds = frozenset({Planning.LOCAL})
    checksum = 0
    start = time.perf_counter()
    for index in range(repeats):
        now = 1_000_000_000 + index * 2_000_000
        version = Version(f'goal-{index}', 1, 2, 3, 4, 5)
        stamp = Stamp(now, 'steady', 5)
        session.activate(version, stamp)
        request = session.request(version, Trigger.PATH_RISK, kinds, index, stamp)
        checksum += request.episode + len(request.request_id)
        session.response_current(request, version,
                                 Stamp(now + 500_000, 'steady', 5))
        session.retire(request)
    return time.perf_counter() - start, checksum


def main():
    repeats = int(os.environ.get('BENCH_REPEATS', '20000'))
    budget = planning_session.PlanningBudget(0.1, 0.5, repeats + 1)
    old = planning_session._native
    try:
        planning_session._native = None
        python_seconds, python_checksum = _run(
            planning_session.PlanningSession('bench', budget), repeats)
        planning_session._native = native
        native_seconds, native_checksum = _run(
            planning_session.PlanningSession('bench', budget), repeats)
    finally:
        planning_session._native = old
    print(f'repeats={repeats}')
    print(f'python_seconds={python_seconds:.6f}')
    print(f'native_seconds={native_seconds:.6f}')
    print(f'native_delta_percent={(native_seconds / python_seconds - 1.0) * 100.0:.2f}')
    print(f'checksums={python_checksum},{native_checksum}')


if __name__ == '__main__':
    main()
