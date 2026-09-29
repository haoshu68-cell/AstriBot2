#!/usr/bin/env python3
"""Delay only the diagnostic worker, then invalidate its source before completion.

Run against a copy made by prepare_geometry_timing_probe.py --compute-delay-ms 120.
Production input, completion checks and output code are unchanged in that copy.
No robot or simulator commands are published.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_state_phase_sweep import PhaseFixture
from benchmark_geometry_state_ab import digest, ns


def trace_events(log):
    return [json.loads(line.removeprefix('GEOMETRY_TIMING ')) for line in log.read_text().splitlines()
            if line.startswith('GEOMETRY_TIMING ')]


def replay(fixture, scenario, args):
    fixture.configure('offset_payload')
    folder = Path(args.output) / scenario
    folder.mkdir(parents=True, exist_ok=True)
    log = folder / 'session.log'
    command = [args.cpp, '--ros-args', '-p', 'base_frame:=base', '-p', 'use_sim_time:=true']
    now = 2_000_000_000
    with log.open('w') as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
        try:
            def spin(duration):
                fixture.until(time.monotonic() + duration, process)

            def clock(value):
                fixture.clock_pub.publish(Clock(clock=Time(sec=value // 10**9, nanosec=value % 10**9)))

            def step():
                nonlocal now
                now += 20_000_000
                message = JointState()
                message.header.stamp = Time(sec=now // 10**9, nanosec=now % 10**9)
                message.name = ['arm']
                message.position = [.2]
                fixture.joints.publish(message)
                spin(.005)
                clock(now)
                if fixture.current_ack:
                    fixture.ack.publish(String(data=fixture.current_ack))
                spin(.015)

            for _ in range(110):
                step()
            assert any(message.complete for message, _, _ in fixture.outputs), 'warmup never became valid'
            previous = max(event['source_ns'] for event in trace_events(log) if event['event'] == 'compute_begin')
            selected = None
            for _ in range(30):
                step()
                candidates = [event for event in trace_events(log)
                              if event['event'] == 'compute_begin' and event['source_ns'] > previous]
                if candidates:
                    selected = candidates[-1]
                    break
            assert selected is not None, 'no in-flight work observed'
            source = selected['source_ns']
            before = len(fixture.outputs)
            if scenario == 'source_deadline':
                now = source + 300_000_000
                clock(now)
                expected = 'GEOMETRY_COMPUTE_EXCEEDED_SOURCE_LEASE'
            elif scenario == 'clock_rollback':
                now = source - 100_000_000
                clock(now)
                expected = 'GEOMETRY_INPUT_CHANGED_DURING_COMPUTE'
            else:
                message = JointState()
                message.header.stamp = Time(sec=now // 10**9, nanosec=now % 10**9)
                message.name = ['arm']
                message.position = [float('nan')]
                fixture.joints.publish(message)
                expected = 'GEOMETRY_INPUT_CHANGED_DURING_COMPUTE'
            # The current ROS clock remains fixed while the independent
            # completion callback consumes the delayed immutable result.
            spin(.25)
            outputs = [message for message, _, _ in fixture.outputs[before:]]
            events = trace_events(log)
            assert any(e['event'] == 'compute_end' and e['source_ns'] == source for e in events)
            assert not any(m.complete and ns(m.header.stamp) == source for m in outputs)
            assert any(not m.complete and expected in m.reason for m in outputs), [m.reason for m in outputs]
            actual_environment = Path(f'/proc/{process.pid}/environ').read_bytes().split(b'\0')
            assert b'ROS_DOMAIN_ID=115' in actual_environment
            mappings = Path(f'/proc/{process.pid}/maps').read_text()
            assert 'libpython' not in mappings and '_geometry_native' not in mappings
            result = dict(scenario=scenario, source_ns=source, clock_after_fault_ns=now,
                          expected_reason=expected, observed_reasons=[m.reason for m in outputs],
                          rejected_complete_source=True, worker_delay_ms=120, pid=process.pid,
                          session_log=str(log.resolve()))
            (folder / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
            return result
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cpp', required=True)
    parser.add_argument('--diagnostic-source', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') == '115'
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    cpp_source = Path(args.diagnostic_source) / 'src/geometry_state_node.cpp'
    assert 'sleep_for(std::chrono::milliseconds(120))' in cpp_source.read_text()
    (output / 'diagnostic_source.cpp').write_bytes(cpp_source.read_bytes())
    (output / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    (output / 'manifest.json').write_text(json.dumps(dict(
        argv=sys.argv, domain=115, mode='injected worker delay; not performance evidence',
        sha256={str(p.resolve()): digest(p) for p in (Path(args.cpp), cpp_source, Path(__file__))}), indent=2) + '\n')
    rclpy.init()
    fixture = PhaseFixture()
    try:
        results = [replay(fixture, case, args)
                   for case in ('source_deadline', 'clock_rollback', 'malformed_joint')]
    finally:
        fixture.destroy_node()
        rclpy.shutdown()
    (output / 'summary.json').write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
