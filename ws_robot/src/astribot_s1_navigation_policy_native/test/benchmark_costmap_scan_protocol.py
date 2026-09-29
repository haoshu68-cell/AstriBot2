"""Serial full-scan DDS A/B; correctness checks are outside the latency sample."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import time

from benchmark_envelope_protocol import process_stats, percentile
from test_costmap_scan_protocol import ScanReplay, protection, scan_values


def trial(impl, binary, count, directory, index):
    replay = ScanReplay(impl, binary, directory / f'scan_{index}_{impl}.log')
    try:
        ranges = ([.5, 1., math.inf, 8., math.nan] * 216)
        for i in range(20):
            replay.events.clear()
            replay.expect(replay.scan(i + 1, ranges), 5.5)
        replay.events.clear()
        # Compute the reference once; mutate only its source stamp per frame.
        expected = replay.scan(21, ranges)
        expected.ranges = protection.costmap_clearing_ranges(expected.ranges, expected.range_max, 5.5)
        replay.until(lambda: bool(replay.events))
        replay.events.clear()
        cpu0, rss = process_stats(replay.proc.pid)
        start = time.perf_counter()
        latencies = []
        for i in range(count):
            tick = time.perf_counter_ns()
            sent = replay.scan(i + 100, ranges)
            replay.until(lambda: bool(replay.events))
            latencies.append((time.perf_counter_ns() - tick) / 1e6)
            expected.header.stamp = sent.header.stamp
            assert len(replay.events) == 1
            assert scan_values(replay.events[0]) == scan_values(expected)
            replay.events.clear()
            if i % 100 == 0: rss = max(rss, process_stats(replay.proc.pid)[1])
        elapsed = time.perf_counter() - start
        cpu1, end_rss = process_stats(replay.proc.pid)
        return dict(implementation=impl, frames=count, rays_per_frame=1080, elapsed_s=elapsed,
            cpu_s=cpu1 - cpu0, sampled_rss_kib=max(rss, end_rss),
            p50_ms=statistics.median(latencies), p95_ms=percentile(latencies, .95),
            p99_ms=percentile(latencies, .99), frames_per_s=count / elapsed, latency_ms=latencies)
    finally:
        replay.close()


from reference_bootstrap import source_or_reference


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--frames', type=int, default=5000)
    args = parser.parse_args()
    assert args.frames > 0
    args.output.parent.mkdir(parents=True, exist_ok=True)
    logs = args.output.parent / 'benchmark_logs'
    logs.mkdir(exist_ok=True)
    trials = []
    for order in [('python', 'cpp'), ('cpp', 'python'), ('python', 'cpp')]:
        for impl in order:
            result = trial(impl, args.binary, args.frames, logs, len(trials))
            trials.append(result)
            print({k: v for k, v in result.items() if k != 'latency_ms'}, flush=True)
    medians = {impl: {key: statistics.median(t[key] for t in trials if t['implementation'] == impl)
                     for key in ('elapsed_s', 'cpu_s', 'sampled_rss_kib', 'p50_ms', 'p95_ms', 'p99_ms',
                                 'frames_per_s')} for impl in ('python', 'cpp')}
    root = Path(__file__).resolve().parents[1]
    sources = [Path(__file__), Path(__file__).with_name('test_costmap_scan_protocol.py'),
               Path(__file__).with_name('benchmark_envelope_protocol.py'),
               root / 'src/costmap_scan_node.cpp', root / 'src/navigation_math.cpp',
               root.parent / 'astribot_s1_navigation_policy/astribot_s1_navigation_policy/costmap_scan_node.py',
               root.parent / 'astribot_s1_navigation_policy/astribot_s1_navigation_policy/protection.py']
    report = dict(scope='isolated full LaserScan DDS adapter; no Nav2/hardware; serial client',
        wall_time_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), domain=110, localhost_only=True,
        host=platform.platform(), ros_distro=os.environ.get('ROS_DISTRO'),
        source_sha256={str(source_or_reference(p).resolve()): hashlib.sha256(source_or_reference(p).read_bytes()).hexdigest() for p in sources},
        binary_sha256=hashlib.sha256(Path(args.binary).read_bytes()).hexdigest(),
        median_by_implementation=medians, trials=trials)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(medians, indent=2))


if __name__ == '__main__':
    main()
