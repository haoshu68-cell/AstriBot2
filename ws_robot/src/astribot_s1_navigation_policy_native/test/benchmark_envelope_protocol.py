"""Same-host isolated DDS proposal-to-envelope A/B; no hardware or Nav2.

Paused ROS time intentionally holds the same fresh odometry for all requests.
Each accepted request clears both costmap acks and emits a pending envelope.
Latency includes service response AND observation of the resulting envelope.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import statistics
import subprocess
import time

from test_envelope_protocol import Protocol


def process_stats(pid):
    root = Path('/proc') / str(pid)
    fields = (root / 'stat').read_text().rsplit(')', 1)[1].split()
    cpu = (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')
    status = dict(line.split(':', 1) for line in (root / 'status').read_text().splitlines())
    return cpu, int(status['VmRSS'].split()[0])


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int((len(ordered) - 1) * fraction))]


def trial(impl, binary, count, output, index):
    protocol = Protocol(impl, binary, output / f'{index}_{impl}.log')
    try:
        protocol.ready()
        protocol.stopped()
        for _ in range(30):
            assert protocol.request(half_length_m=.35, transport_ready=True).accepted
        epoch = protocol.events[-1].epoch
        cpu_before, rss = process_stats(protocol.proc.pid)
        cpu_start = time.perf_counter()
        latencies = []
        for i in range(count):
            half_length = .35 + (i % 2) * .01
            start = time.perf_counter_ns()
            reply = protocol.request(half_length_m=half_length, transport_ready=True)
            latencies.append((time.perf_counter_ns() - start) / 1e6)
            assert reply.accepted and reply.epoch == epoch + i + 1
            message = protocol.events[-1]
            assert message.half_length_m == half_length and not message.transport_ready
            assert message.reason == 'POSTURE_OR_FOOTPRINT_PENDING'
            if i % 100 == 0:
                rss = max(rss, process_stats(protocol.proc.pid)[1])
        elapsed = time.perf_counter() - cpu_start
        cpu_after, rss_end = process_stats(protocol.proc.pid)
        return dict(implementation=impl, requests=count, elapsed_s=elapsed,
                    cpu_s=cpu_after - cpu_before, sampled_rss_kib=max(rss, rss_end),
                    p50_ms=statistics.median(latencies), p95_ms=percentile(latencies, .95),
                    p99_ms=percentile(latencies, .99), max_ms=max(latencies),
                    requests_per_s=count / elapsed, latency_ms=latencies)
    finally:
        protocol.close()


from reference_bootstrap import source_or_reference


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--requests', type=int, default=1000)
    parser.add_argument('--pairs', type=int, default=3)
    args = parser.parse_args()
    assert args.requests > 0 and args.pairs > 0
    args.output.parent.mkdir(parents=True, exist_ok=True)
    logs = args.output.parent / 'benchmark_logs'
    logs.mkdir(exist_ok=True)
    trials = []
    for pair in range(args.pairs):
        for impl in (('python', 'cpp') if pair % 2 == 0 else ('cpp', 'python')):
            result = trial(impl, args.binary, args.requests, logs, len(trials))
            trials.append(result)
            print({k: v for k, v in result.items() if k != 'latency_ms'}, flush=True)
    sources = [Path(__file__), Path(__file__).with_name('test_envelope_protocol.py')]
    native = Path(__file__).resolve().parents[1]
    sources += list((native / 'src').glob('envelope*.cpp')) + [native / 'src/policy_profile.cpp']
    sources += [native.parent / 'astribot_s1_navigation_policy/astribot_s1_navigation_policy/envelope_node.py',
                native.parent / 'astribot_s1_navigation_policy/config/simulation.json']
    sources += list((native.parent / 'astribot_navigation_msgs').glob('msg/RobotEnvelope.msg'))
    sources += list((native.parent / 'astribot_navigation_msgs').glob('srv/SetRobotEnvelope.srv'))
    medians = {impl: {key: statistics.median(t[key] for t in trials if t['implementation'] == impl)
                     for key in ('elapsed_s', 'cpu_s', 'sampled_rss_kib', 'p50_ms', 'p95_ms', 'p99_ms',
                                 'requests_per_s')} for impl in ('python', 'cpp')}
    report = dict(scope='isolated DDS proposal-to-pending-envelope; paused ROS time; no Nav2/hardware',
        host=platform.platform(), python=platform.python_version(), domain=109, localhost_only=True,
        ros_distro=os.environ.get('ROS_DISTRO'), rmw=os.environ.get('RMW_IMPLEMENTATION', 'ROS default'),
        wall_time_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        source_sha256={str(source_or_reference(p).resolve()): hashlib.sha256(source_or_reference(p).read_bytes()).hexdigest() for p in sources},
        binary_sha256=hashlib.sha256(Path(args.binary).read_bytes()).hexdigest(),
        git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        cpu_description=[line for line in Path('/proc/cpuinfo').read_text().splitlines()
                         if line.startswith('model name')][:1],
        median_by_implementation=medians, trials=trials)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(medians, indent=2))


if __name__ == '__main__':
    main()
