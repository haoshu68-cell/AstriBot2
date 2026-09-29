#!/usr/bin/env python3
"""Compare the Python oracle with the directly linked C++ wheel state object."""

from pathlib import Path as _ReferencePath
import sys as _reference_sys
_reference_sys.path.insert(0, str(_ReferencePath(__file__).resolve().parents[4] / "tools/migration/python_reference"))
from reference_bootstrap import enable as _enable_references
_enable_references()

import statistics
import subprocess
import sys
import time

from astribot_s1_chassis_effort_drive import omni_effort_drive_node as drive


def run_python(count=500_000, repeats=7):
    samples = []
    checksum = 0.0
    for _ in range(repeats):
        loop = drive._WheelLoop()
        start = time.perf_counter()
        checksum = 0.0
        for index in range(count):
            tau, error = loop.update(
                2.0 + (index % 13) * 0.01, 0.2, 0.4, 0.1, 0.0,
                0.1, 1.0, 0.05, 15.0, 0.01)
            checksum += tau + error
        samples.append(time.perf_counter() - start)
    elapsed = statistics.median(samples)
    return elapsed, count / elapsed, checksum


if __name__ == "__main__":
    elapsed, rate, checksum = run_python()
    print("python %.6f s %.0f/s checksum=%.9e" % (elapsed, rate, checksum))
    if len(sys.argv) == 2:
        subprocess.run([sys.argv[1], "500000", "7"], check=True)
