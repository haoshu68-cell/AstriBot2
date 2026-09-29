#!/usr/bin/env python3
"""Paired one-way OccupancyGrid forwarding overhead, not SLAM performance."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import statistics
import time

from test_map_relay import Relay, signature


def process_sample(pid):
    text = Path(f'/proc/{pid}/stat').read_text().split(') ', 1)[1].split()
    cpu = (int(text[11]) + int(text[12])) / os.sysconf('SC_CLK_TCK')
    rss = int(Path(f'/proc/{pid}/statm').read_text().split()[1]) * os.sysconf('SC_PAGE_SIZE') / 2**20
    return cpu, rss


def quantile(values, q):
    ordered = sorted(values)
    point = (len(ordered) - 1) * q
    lower = int(point)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (point - lower)


def run(impl, side, count, trial, output):
    relay = Relay(impl, f'benchmark_{side}_{trial}', start=False)
    samples = []
    try:
        msg = relay.message(1, width=side, height=side, data=[-1, 0, 64, 100] * (side*side//4))
        relay.publisher.publish(msg)
        relay.start()
        # Set every extant thread and the main thread before warmup; future
        # threads inherit the main thread mask. Record masks after warmup.
        os.sched_setaffinity(relay.p.pid, {26})
        for task in Path(f'/proc/{relay.p.pid}/task').iterdir():
            os.sched_setaffinity(int(task.name), {26})
        relay.ready()
        relay.until(lambda: relay.messages)
        for i in range(10):
            msg.header.stamp.sec = 10 + i
            relay.send(msg)
        masks = {t.name: sorted(os.sched_getaffinity(int(t.name)))
                 for t in Path(f'/proc/{relay.p.pid}/task').iterdir()}
        assert all(mask == [26] for mask in masks.values()), masks
        initial_count = len(relay.messages)
        cpu0, rss0 = process_sample(relay.p.pid)
        start = time.perf_counter()
        for index in range(count):
            msg.header.stamp.sec = 100 + index
            latency = relay.send(msg)
            _, rss = process_sample(relay.p.pid)
            samples.append({'index': index, 'latency_ms': latency, 'rss_mib': rss})
        duration = time.perf_counter() - start
        cpu1, rss1 = process_sample(relay.p.pid)
        new = relay.messages[initial_count:]
        assert len(new) == count
        assert [v[1].header.stamp.sec for v in new] == list(range(100, 100+count))
        latencies = [s['latency_ms'] for s in samples]
        result = {'implementation': impl, 'side': side, 'cells': side*side,
                  'trial': trial, 'count': count, 'received': len(new),
                  'p50_ms': statistics.median(latencies), 'p95_ms': quantile(latencies, .95),
                  'p99_ms': quantile(latencies, .99), 'max_ms': max(latencies),
                  'cpu_ms_per_map': (cpu1-cpu0)*1000/count,
                  'cpu_single_core_pct': (cpu1-cpu0)/duration*100,
                  'rss_mib': statistics.median([s['rss_mib'] for s in samples]),
                  'rss_growth_mib': rss1-rss0, 'duration_sec': duration,
                  'thread_affinity': masks, 'driver_affinity': sorted(os.sched_getaffinity(0)),
                  'pid': relay.p.pid, 'node_log': str(relay.path / 'node.log'),
                  'command': relay.command,
                  'message_payload_sha256': hashlib.sha256(bytes(msg.data)).hexdigest()}
        with (output / f'{side}_{trial}_{impl}.csv').open('w') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(samples[0]))
            writer.writeheader()
            writer.writerows(samples)
        (output / f'{side}_{trial}_{impl}.json').write_text(json.dumps(result, indent=2)+'\n')
        return result
    finally:
        relay.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--pairs', type=int, default=4)
    parser.add_argument('--count-small', type=int, default=100)
    parser.add_argument('--count-large', type=int, default=40)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.sched_setaffinity(0, {27})
    trials = []
    for side, count in [(256, args.count_small), (1024, args.count_large)]:
        for trial in range(args.pairs):
            order = ['python', 'cpp'] if trial % 2 == 0 else ['cpp', 'python']
            for impl in order:
                item = run(impl, side, count, trial, args.output)
                trials.append(item)
                print(json.dumps({k: item[k] for k in ('implementation', 'side', 'trial', 'p50_ms', 'p95_ms')}), flush=True)
    summary = {}
    for side in (256, 1024):
        summary[str(side)] = {}
        for impl in ('python', 'cpp'):
            group = [t for t in trials if t['side'] == side and t['implementation'] == impl]
            summary[str(side)][impl] = {key: statistics.median([t[key] for t in group])
                for key in ('p50_ms', 'p95_ms', 'p99_ms', 'cpu_ms_per_map', 'rss_mib')}
            summary[str(side)][impl]['worst_ms'] = max(t['max_ms'] for t in group)
            summary[str(side)][impl]['count'] = sum(t['received'] for t in group)
    (args.output / 'summary.json').write_text(json.dumps({'trials': trials, 'summary': summary}, indent=2)+'\n')


if __name__ == '__main__':
    main()
