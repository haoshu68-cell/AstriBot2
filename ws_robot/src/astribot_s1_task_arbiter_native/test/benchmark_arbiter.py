#!/usr/bin/env python3
"""Alternating same-input fake Nav2 trials; measures action proxy, not navigation."""
import argparse
import json
import os
from pathlib import Path
import statistics
import threading
import time

from nav2_msgs.action import NavigateToPose
from test_arbiter_protocol import Runtime


def resolved(future):
    ready = threading.Event()
    future.add_done_callback(lambda _: ready.set())
    if not ready.wait(5.): raise RuntimeError('benchmark action future timed out')
    return future.result()


def proc_stats(pid):
    fields = Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()
    cpu = (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')
    rss = int(Path(f'/proc/{pid}/statm').read_text().split()[1]) * os.sysconf('SC_PAGE_SIZE')
    return cpu, rss


def trial(impl, directory, count):
    directory.mkdir(parents=True, exist_ok=True)
    r = Runtime(impl, directory, timeout=2.)
    samples = []
    try:
        b = r.backends['pose']; b.done.set()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = 7.25
        goal.pose.pose.orientation.w = 1.
        goal.behavior_tree = 'benchmark_fake.xml'
        def one():
            start = time.perf_counter_ns()
            handle = resolved(r.clients['pose', 'operator'].send_goal_async(goal))
            accepted = time.perf_counter_ns()
            if not handle.accepted: raise RuntimeError('unexpected busy/reject in serial trial')
            result = resolved(handle.get_result_async())
            end = time.perf_counter_ns()
            if result.status != 4: raise RuntimeError('unexpected backend terminal')
            return {'start_ns': start, 'accepted_ns': accepted, 'terminal_ns': end,
                    'task_id': bytes(handle.goal_id.uuid).hex()}
        for _ in range(8): one()
        cpu0, rss0 = proc_stats(r.proc.pid)
        wall0 = time.perf_counter()
        for _ in range(count): samples.append(one())
        elapsed = time.perf_counter() - wall0
        cpu1, rss1 = proc_stats(r.proc.pid)
        # A retained executing task proves idle executor responsiveness without
        # pretending that missing result is a terminal outcome.
        latencies = sorted((s['terminal_ns'] - s['start_ns']) / 1e6 for s in samples)
        return dict(implementation=impl, count=count, elapsed_s=elapsed,
            median_ms=statistics.median(latencies), p95_ms=latencies[int(.95*(count-1))],
            max_ms=max(latencies), cpu_ms_per_goal=(cpu1-cpu0)*1000/count,
            rss_start_bytes=rss0, rss_end_bytes=rss1, samples=samples)
    finally:
        r.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--pairs', type=int, default=5)
    parser.add_argument('--count', type=int, default=100)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for pair in range(args.pairs):
        order = ['python', 'cpp'] if pair % 2 == 0 else ['cpp', 'python']
        for impl in order:
            result = trial(impl, args.output / f'pair_{pair}_{impl}', args.count)
            result['pair'] = pair
            rows.append(result)
            print(json.dumps({k: v for k, v in result.items() if k != 'samples'}), flush=True)
            (args.output / 'results.json').write_text(json.dumps(rows, indent=2))


if __name__ == '__main__': main()
