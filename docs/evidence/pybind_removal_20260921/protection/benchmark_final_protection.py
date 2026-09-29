"""Interleaved owned-process A/B resource replay; not an algorithm latency test.

Twist input has no sequence/stamp and MotionConstraint.sequence belongs to the
output writer. No reliable input/output correlation exists, so this experiment
reports child CPU/RSS and schedule/delivery evidence only, never fixture sleeps
as an implementation speedup. Domain 114; no Gazebo, Nav2 or hardware.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import subprocess
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from astribot_navigation_msgs.msg import MotionConstraint

from test_final_protection_protocol import Replay, POLICY


def process_stats(pid):
    root = Path('/proc') / str(pid)
    fields = (root / 'stat').read_text().rsplit(')', 1)[1].split()
    status = dict(line.split(':', 1) for line in (root / 'status').read_text().splitlines())
    hz = os.sysconf('SC_CLK_TCK')
    return dict(user_s=int(fields[11]) / hz, system_s=int(fields[12]) / hz,
                rss_kib=int(status['VmRSS'].split()[0]), peak_rss_kib=int(status['VmHWM'].split()[0]))


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def percentile(values, fraction):
    return sorted(values)[min(len(values) - 1, math.ceil(len(values) * fraction) - 1)]


def trial(implementation, mode, pair, ticks, hz, rays, root):
    folder = root / f'{pair}_{mode}_{implementation}'
    folder.mkdir(parents=True, exist_ok=True)
    replay = Replay(implementation, mode, folder)
    try:
        replay.warm(command=(.1, 0., 0.), measured=(.12, 0., 0.))
        replay.wait(lambda: replay.diagnostics and replay.diagnostics[-1]['scan_received'] >= 40)
        initial_scan_count = replay.diagnostics[-1]['scan_received']
        scan = LaserScan(); scan.header.frame_id = replay.p.base_frame
        scan.range_min = .01; scan.range_max = 10.
        scan.angle_min = -math.pi; scan.angle_increment = 2 * math.pi / (rays - 1); scan.angle_max = math.pi
        clear_ranges = [2.] * rays
        blocked_ranges = clear_ranges.copy()
        blocked_ranges[(rays - 1) // 2] = .45
        odom = Odometry(); odom.twist.twist.linear.x = .12
        command = Twist(); command.linear.x = .1
        proposal = MotionConstraint(epoch=1, lease_s=.3, max_linear_speed=.2,
                                    max_angular_speed=.4, reason='CLEAR')
        envelope = replay.make_envelope()
        start_ns = round(replay.seconds * 1e9)
        step_ns = round(1e9 / hz)
        # Fixed duration refusal window is identical for all implementations.
        blocked_begin, blocked_end = ticks // 3, ticks // 3 + 40
        result_index, output_index = len(replay.results), len(replay.outputs)
        stats_start = process_stats(replay.proc.pid)
        cpu_load_start = os.getloadavg()
        start = time.perf_counter()
        frames, resources = [], []
        for i in range(ticks):
            stamp_ns = start_ns + (i + 1) * step_ns
            replay.seconds = stamp_ns / 1e9
            stamp = replay.stamp()
            replay.clock.publish(Clock(clock=stamp))
            # Prove the child processed this ROS clock before sending source-
            # stamped V2 data. Separate DDS topics do not promise arrival order.
            # This barrier is harness synchronization, never measured latency.
            replay.wait(lambda: ns(replay.results[-1].stamp) >= stamp_ns)
            odom.header.stamp = stamp; scan.header.stamp = stamp
            blocked = blocked_begin <= i < blocked_end
            scan.ranges = blocked_ranges if blocked else clear_ranges
            replay.sequence += 1; proposal.sequence = replay.sequence; proposal.stamp = stamp
            if mode == 'legacy': envelope.stamp = stamp
            else:
                envelope.header.stamp = stamp; envelope.limits.stamp = stamp
                envelope.valid_until = replay.stamp(-.3)
            sent_at = time.perf_counter()
            replay.odom.publish(odom); replay.scan.publish(scan)
            replay.envelope.publish(envelope); replay.proposal.publish(proposal); replay.cmd.publish(command)
            deadline = start + (i + 1) / hz
            while time.perf_counter() < deadline:
                assert replay.proc.poll() is None, f'node exited: {folder / "session.log"}'
                replay.executor.spin_once(timeout_sec=min(.001, max(0., deadline - time.perf_counter())))
            frames.append(dict(input_index=i, source_stamp_ns=stamp_ns, obstacle=blocked,
                publication_wall_s=sent_at - start, finish_wall_s=time.perf_counter() - start,
                deadline_lateness_s=max(0., time.perf_counter() - deadline),
                received_constraint_count=len(replay.results) - result_index,
                received_twist_count=len(replay.outputs) - output_index))
            if i % 20 == 0:
                resources.append(dict(input_index=i, wall_s=time.perf_counter() - start,
                                      **process_stats(replay.proc.pid)))
        elapsed = time.perf_counter() - start
        stats_end = process_stats(replay.proc.pid)
        # Drain messages already in flight outside resource measurement; source
        # stamps let us exclude unrelated pre-window outputs precisely.
        replay.spin(.04)
        replay.wait(lambda: replay.diagnostics[-1]['scan_received'] >= initial_scan_count + ticks)
        messages = replay.results[result_index:]
        outputs = replay.outputs[output_index:]
        wire_path = folder / 'wire.json'
        wire_path.write_text(json.dumps(dict(frames=frames, resources=resources,
            process_stats_start=stats_start, process_stats_end=stats_end,
            constraints=[dict(stamp_ns=ns(m.stamp), sequence=m.sequence, hold=m.hold,
                reason=m.reason, linear_cap=m.max_linear_speed, angular_cap=m.max_angular_speed)
                for m in messages], twists=[[m.linear.x, m.linear.y, m.angular.z] for m in outputs],
            final_diagnostics=replay.diagnostics[-1]), indent=2) + '\n')
        constraints, checked_clear, checked_blocked = [], 0, 0
        previous_sequence = None
        for message in messages:
            index = (ns(message.stamp) - start_ns) // step_ns - 1
            assert previous_sequence is None or message.sequence > previous_sequence
            previous_sequence = message.sequence
            assert math.isfinite(message.max_linear_speed) and 0 <= message.max_linear_speed <= .2
            assert math.isfinite(message.max_angular_speed) and 0 <= message.max_angular_speed <= .4
            assert not message.alignment_required and not message.centering_required
            if message.hold: assert message.max_linear_speed == message.max_angular_speed == 0.
            assert message.reason not in ('INPUT_UNAVAILABLE', 'POLICY_UNAVAILABLE',
                'ROBOT_ENVELOPE_UNAVAILABLE', 'SIM_CLOCK_STALLED'), (implementation, mode, index, message.reason)
            # DDS topic arrival order may differ for transition frames. Interior
            # windows require the exact reason, and every frame keeps invariants.
            if blocked_begin + 5 <= index < blocked_end - 2:
                assert message.hold and message.reason == 'INDEPENDENT_SWEEP_RISK', (index, message.reason)
                checked_blocked += 1
            elif (5 <= index < blocked_begin - 2 or blocked_end + math.ceil(replay.p.clear_hold_s * hz) + 5 <= index < ticks):
                assert not message.hold and message.reason == 'CLEAR', (index, message.reason)
                checked_clear += 1
            constraints.append(dict(input_clock_index=index, sequence=message.sequence,
                stamp_ns=ns(message.stamp), hold=message.hold, reason=message.reason,
                linear_cap=message.max_linear_speed, angular_cap=message.max_angular_speed))
        for message in outputs:
            assert math.isfinite(message.linear.x) and 0 <= message.linear.x <= .100000000001
            assert message.linear.y == message.angular.z == 0.
        assert checked_clear > ticks // 2 and checked_blocked >= 25
        assert len(messages) >= ticks, 'expected command-triggered and watchdog outputs'
        user = stats_end['user_s'] - stats_start['user_s']
        system = stats_end['system_s'] - stats_start['system_s']
        cpu = user + system
        return dict(pair=pair, implementation=implementation, mode=mode, pid=replay.proc.pid,
            ticks=ticks, target_hz=hz, scan_rays=rays, elapsed_s=elapsed, achieved_input_hz=ticks / elapsed,
            user_cpu_s=user, system_cpu_s=system, cpu_s=cpu, cpu_percent_of_one_core=cpu / elapsed * 100,
            cpu_ms_per_input_frame=cpu * 1000 / ticks,
            median_rss_kib=statistics.median([s['rss_kib'] for s in resources]),
            max_sampled_rss_kib=max([s['rss_kib'] for s in resources] + [stats_end['rss_kib']]),
            peak_process_rss_kib=stats_end['peak_rss_kib'],
            checked_clear_outputs=checked_clear, checked_refusal_outputs=checked_blocked,
            output_constraints=len(messages), output_twists=len(outputs),
            deadline_lateness_p95_ms=percentile([f['deadline_lateness_s'] * 1000 for f in frames], .95),
            loadavg_start=cpu_load_start, loadavg_end=os.getloadavg(),
            process_stats_start=stats_start, process_stats_end=stats_end,
            received_scans=replay.diagnostics[-1]['scan_received'] - initial_scan_count,
            final_diagnostics=replay.diagnostics[-1], raw_wire_file=str(wire_path.resolve()),
            blocked_frames=[blocked_begin, blocked_end], frames=frames, resources=resources,
            constraints=constraints,
            twists=[[m.linear.x, m.linear.y, m.angular.z] for m in outputs],
            session_log=str((folder / 'session.log').resolve()))
    finally:
        replay.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--pairs', type=int, default=3)
    parser.add_argument('--ticks', type=int, default=400)
    parser.add_argument('--hz', type=float, default=50.)
    parser.add_argument('--rays', type=int, default=1080)
    args = parser.parse_args()
    if args.ticks < 300: parser.error('at least 300 ticks required for stable acceptance windows')
    os.environ['FINAL_PROTECTION_CPP_BINARY'] = str(Path(args.binary).resolve())
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    logs = args.output.parent / 'sessions'; logs.mkdir(exist_ok=True)
    native = Path(__file__).resolve().parents[1]
    sources = [Path(__file__), Path(__file__).with_name('test_final_protection_protocol.py'),
               POLICY / 'config/simulation.json']
    sources += list((native / 'src').glob('final_protection*.cpp'))
    sources += [native / 'src/navigation_math.cpp', native / 'src/policy_profile.cpp']
    sources += list((native / 'include/astribot_s1_navigation_policy_native').glob('*protection*.hpp'))
    sources += [POLICY / 'astribot_s1_navigation_policy' / name for name in
                ('protection_node.py', 'protection.py', 'control_time.py', 'robot_envelope.py', 'continuous_sweep.py')]
    geometry = native.parent / 'astribot_s1_robot_geometry'
    sources += [geometry / 'include/astribot_s1_robot_geometry/geometry_kernels.hpp',
                geometry / 'astribot_s1_robot_geometry/polygon.py']
    report = dict(scope='isolated DDS final-protection node process CPU/RSS; startup excluded; no Nav2/Gazebo/hardware',
        latency_not_measured='Twist input has no source sequence/stamp; output constraint sequence cannot identify the triggering input',
        pacing='precomputed geometry; same target 50Hz wall-paced inputs; each ROS clock is acknowledged by a stamped output before sending its inputs; barrier and scheduling lateness are harness evidence, not latency',
        resource_definition='owned child /proc user+system CPU across all its threads; RSS includes interpreter/runtime/libraries; excludes benchmark process',
        host=platform.platform(), cpu_model=next((x.split(':', 1)[1].strip() for x in Path('/proc/cpuinfo').read_text().splitlines()
            if x.startswith('model name')), ''), python=platform.python_version(), domain=114,
        ros_distro=os.environ.get('ROS_DISTRO'), rmw=os.environ.get('RMW_IMPLEMENTATION', 'ROS default'),
        wall_time_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        source_sha256={str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        binary_sha256=hashlib.sha256(Path(args.binary).read_bytes()).hexdigest(),
        git_head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(), trials=[])
    for pair in range(args.pairs):
        modes = ('legacy', 'fixed_v2') if pair % 2 == 0 else ('fixed_v2', 'legacy')
        for mode in modes:
            for implementation in (('python', 'cpp') if pair % 2 == 0 else ('cpp', 'python')):
                result = trial(implementation, mode, pair, args.ticks, args.hz, args.rays, logs)
                report['trials'].append(result)
                args.output.write_text(json.dumps(report, indent=2) + '\n')
                print({key: result[key] for key in ('pair', 'mode', 'implementation', 'cpu_s',
                    'cpu_percent_of_one_core', 'median_rss_kib', 'achieved_input_hz', 'checked_refusal_outputs')}, flush=True)
    keys = ('cpu_s', 'cpu_percent_of_one_core', 'cpu_ms_per_input_frame', 'median_rss_kib',
            'max_sampled_rss_kib', 'achieved_input_hz')
    report['median_by_mode_and_implementation'] = {
        mode: {implementation: {key: statistics.median(t[key] for t in report['trials']
            if t['mode'] == mode and t['implementation'] == implementation) for key in keys}
            for implementation in ('python', 'cpp')} for mode in ('legacy', 'fixed_v2')}
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report['median_by_mode_and_implementation'], indent=2), flush=True)


if __name__ == '__main__':
    main()
