#!/usr/bin/env python3
"""Benchmark the three Python-visible paths during the bridge migration.

This intentionally measures the pybind boundary separately from the direct C++
kernel.  It must not be used as an end-to-end ROS or SDK latency claim.
"""

import argparse
import gc
import resource
import statistics
import time

from astribot_trajectory_bridge_native import _chassis_math_native as native
from astribot_trajectory_bridge import chassis_integrator as bridge


def run(mode, loops, repeats):
    if mode == "python_reference":
        bridge._native = None
        call = bridge.integrate_step_dt
    elif mode == "python_native_compat":
        bridge._native = native
        call = bridge.integrate_step_dt
    elif mode == "native_direct":
        call = native.integrate_step_dt
    else:
        raise ValueError(mode)

    samples = []
    checksums = []
    for _ in range(repeats):
        gc.collect()
        state = [1.0, 2.0, 0.3]
        velocity = [0.2, -0.1, 0.05]
        checksum = 0.0
        start = time.perf_counter()
        for _ in range(loops):
            state = call(state, velocity, 0.004)
            checksum += sum(state)
        samples.append(time.perf_counter() - start)
        checksums.append(checksum)
    return {
        "median_s": statistics.median(samples),
        "min_s": min(samples),
        "max_s": max(samples),
        "calls_per_s": loops / statistics.median(samples),
        "checksum": statistics.median(checksums),
        "maxrss_kb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--loops", type=int, default=1_000_000)
    parser.add_argument("--repeats", type=int, default=7)
    args = parser.parse_args()
    for mode in ("python_reference", "python_native_compat", "native_direct"):
        result = run(mode, args.loops, args.repeats)
        print(mode, " ".join(f"{key}={value}" for key, value in result.items()))


if __name__ == "__main__":
    main()
