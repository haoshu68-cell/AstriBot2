#!/usr/bin/env python3
"""Persistent-process fake-port C++/Python core benchmark; no ROS or vendor SDK.

Timing starts after setup and ends before RSS/JSON collection. Verification keeps
full per-write traces outside the measured samples. Run at least three alternating
pairs, and keep startup, IPC/serialization and baseline-loop figures separate.
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
import sys
import time

KINDS = ('gripper', 'arm', 'chassis')
PACKAGE = Path(__file__).resolve().parents[1]
PYTHON_PACKAGE = PACKAGE.parent / 'astribot_trajectory_bridge'


def memory_kib(field):
    with open('/proc/self/status') as f:
        return next(int(line.split()[1]) for line in f if line.startswith(field + ':'))


def rss_kib():
    return memory_kib('VmRSS')


def python_worker():
    for key in tuple(os.environ):
        if key.startswith('ASTRIBOT_') and 'NATIVE' in key:
            os.environ.pop(key)
    sys.path.insert(0, str(PYTHON_PACKAGE))
    from astribot_trajectory_bridge import arm_bridge_core as arm
    from astribot_trajectory_bridge import arm_traj_math, chassis_integrator, chassis_feedback, gripper_math
    from astribot_trajectory_bridge import chassis_bridge_core as chassis
    from astribot_trajectory_bridge import gripper_core as gripper
    for module in (arm, arm_traj_math, chassis, chassis_integrator, chassis_feedback, gripper, gripper_math):
        module._native = None

    class Clock:
        def __init__(self): self.t = 0.0
        def now(self): return self.t
        def sleep(self, dt): self.t += dt

    class Session:
        def __init__(self, clock, part, verify):
            self.clock, self.part, self.verify = clock, part, verify
            self.actual = [0.] * (1 if part == 1 else 3)
            self.trace = []
            self.writes = self.opens = self.closes = 0
        def get_current_joints_position(self, names): return [list(self.actual)]
        def get_joints_position_limit(self, names): return [[-2., -2., -2.]], [[2., 2., 2.]]
        def set_joints_position(self, names, q, control_way='filter', use_wbc=False, add_default_torso=True):
            self.writes += 1
            if self.verify:
                assert names == [{1:"g",2:"astribot_chassis",3:"astribot_arm_left"}[self.part]]
                assert control_way in ("direct", "filter")
                self.trace.append([0, self.clock.t, self.part, list(q[0]),
                    0 if control_way == 'direct' else 1, int(use_wbc), int(add_default_torso), 0.])
            follow = 1. if self.part == 1 else .87 if self.part == 2 else .65
            self.actual = [a + follow * (target-a) for a, target in zip(self.actual, q[0])]
        def open_effector(self, names, duration):
            self.opens += 1
            self.actual[0] = 0.
            if self.verify:
                assert names == ["g"]
                self.trace.append([1, self.clock.t, self.part, list(self.actual), 0, 0, 0, duration])
        def close_effector(self, names, duration):
            self.closes += 1
            self.actual[0] = 100.
            if self.verify:
                assert names == ["g"]
                self.trace.append([2, self.clock.t, self.part, list(self.actual), 0, 0, 0, duration])

    class Pose:
        def __init__(self, clock): self.clock, self.pose = clock, [0., 0., 0.]
        def lookup(self): return list(self.pose), self.clock.t

    def run(kind, mode, count):
        out = dict(kind=kind, mode=mode, iterations=count, events=0, feedbacks=0,
                   checksum=0., rss_entry_kib=rss_kib())
        clock = Clock()
        part = {'gripper': 1, 'chassis': 2, 'arm': 3}[kind]
        session = Session(clock, part, mode == 'VERIFY')
        if mode == 'BASE':
            def loop():
                for _ in range(count):
                    clock.t += .002 if part == 3 else .004
                    out['checksum'] += clock.t
            state = lambda: 'BASE'
        elif kind == 'gripper':
            cfg = gripper.GripperConfig(gripper_names=['g'], enable_service=True,
                default_duration_sec=.2, settle_extra_sec=0., stream_freq=20.,
                mid_stream_tolerance=.5, mid_stream_timeout_sec=.25)
            core = gripper._PythonGripperController(cfg, session,
                sleep_fn=clock.sleep, clock_fn=clock.now, in_simulation=True)
            def loop():
                for i in range(count):
                    result = core.execute('g', 0., .01, True, 25. * (i % 5), 0., True)
                    if not result.ok: raise RuntimeError(result.error_code)
                    out['checksum'] += result.dispatched_cmd + result.actual_cmd
                    out['events'] += len(core.drain_events())
            state = lambda: 'SUCCESS'
        elif kind == 'arm':
            cfg = arm.ArmBridgeConfig(joint_names=['j0','j1','j2'], stream_freq=500.,
                max_tracking_error_rad=10., max_traj_duration_sec=2000.)
            core = arm._PythonArmTrajExecutor(cfg, session, clock)
            assert core.load_limits()[0]
            assert core.start(cfg.joint_names, [0., count * .002 + 1.],
                [[0.,0.,0.],[1.,-1.,.5]], [[0.,0.,0.],[0.,0.,0.]])[0]
            def loop():
                for _ in range(count):
                    clock.t += .002
                    core.step()
                    out['events'] += len(core.drain_events())
                    out['feedbacks'] += len(core.drain_feedbacks())
                    out['checksum'] += session.actual[0]
            state = lambda: core.phase
        else:
            pose = Pose(clock)
            cfg = chassis.ChassisBridgeConfig(require_fresh_scan=False, pose_source='ground_truth')
            core = chassis._PythonChassisBridgeCore(cfg, session, pose, clock)
            assert core.enable()[0]
            def loop():
                for i in range(count):
                    clock.t += .008 if i % 113 == 0 else .004
                    if i in (100,101): pose.pose = [.002*i,.0005*i,.0002*i]
                    if i == 300: core.submit_twist(0.,0.,0.)
                    elif i == 301: core.submit_twist(-.08,.02,.03)
                    else: core.submit_twist(.12,-.03,.04)
                    if not core.inner_tick(): raise RuntimeError('chassis stopped')
                    out['events'] += len(core.drain_events())
                    out['checksum'] += session.actual[0]
            state = lambda: core.state
        out['rss_ready_kib'] = rss_kib()
        wall = time.perf_counter()
        cpu = time.process_time()
        loop()
        out['cpu_seconds'] = time.process_time() - cpu
        out['wall_seconds'] = time.perf_counter() - wall
        out['rss_after_kib'] = rss_kib()
        out['process_peak_rss_kib'] = memory_kib('VmHWM')
        out.update(state=state(), position=session.actual, writes=session.writes,
                   opens=session.opens, closes=session.closes, trace=session.trace)
        return out

    print(json.dumps({'ready': True, 'language': 'python'}), flush=True)
    for line in sys.stdin:
        mode, kind, count = line.split()
        try:
            print(json.dumps(run(kind, mode, int(count)), allow_nan=False), flush=True)
        except Exception as error:
            print(json.dumps({'error': str(error)}), flush=True)
            raise


class Worker:
    def __init__(self, argv, language, cpu):
        self.language = language
        start = time.perf_counter()
        self.process = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, text=True, bufsize=1)
        os.sched_setaffinity(self.process.pid, {cpu})
        self.ready = json.loads(self.process.stdout.readline())
        if not self.ready.get('ready'): raise RuntimeError(self.ready)
        self.startup_wall_seconds = time.perf_counter() - start
    def sample(self, mode, kind, count):
        start = time.perf_counter()
        self.process.stdin.write(f'{mode} {kind} {count}\n')
        self.process.stdin.flush()
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError(self.process.stderr.read())
        result = json.loads(line)
        result['rpc_wall_seconds'] = time.perf_counter() - start
        if 'error' in result: raise RuntimeError(result['error'])
        result['outside_measured_wall_seconds'] = max(0., result['rpc_wall_seconds']-result['wall_seconds'])
        result['language'] = self.language
        return result
    def close(self):
        self.process.stdin.close()
        self.process.wait(timeout=5)
        stderr = self.process.stderr.read()
        if self.process.returncode: raise RuntimeError(stderr)
        return stderr


def close_enough(left, right, path='root'):
    if isinstance(left, (list, tuple)):
        assert len(left) == len(right), path
        for i, (a,b) in enumerate(zip(left,right)): close_enough(a,b,f'{path}[{i}]')
    elif isinstance(left, str): assert left == right, (path,left,right)
    elif isinstance(left, (int,float)):
        assert math.isclose(left,right,rel_tol=3e-10,abs_tol=3e-12), (path,left,right)
    else: assert left == right, (path,left,right)


def verify_pair(left, right):
    for key in ('state','position','writes','opens','closes','events','feedbacks','checksum','trace'):
        close_enough(left[key],right[key],key)


def hashes():
    files = list((PACKAGE/'src').glob('*.cpp')) + list((PACKAGE/'include').rglob('*.hpp'))
    files += list((PYTHON_PACKAGE/'astribot_trajectory_bridge').glob('*.py'))
    files += [Path(__file__).resolve(), PACKAGE/'test/benchmark_standalone_driver.cpp', PACKAGE/'CMakeLists.txt']
    return {str(p.relative_to(PACKAGE.parent)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(files))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker', action='store_true')
    parser.add_argument('--driver', type=Path)
    parser.add_argument('--output', type=Path, default=Path('/tmp/astribot-bridge-performance.json'))
    parser.add_argument('--iterations', type=int, default=50000)
    parser.add_argument('--warmup', type=int, default=5000)
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--cpu', type=int)
    args = parser.parse_args()
    if args.worker: return python_worker()
    if not args.driver or args.iterations < 1 or args.repeats < 3:
        parser.error('provide --driver, positive --iterations, and --repeats >= 3')
    allowed = sorted(os.sched_getaffinity(0))
    cpu = args.cpu if args.cpu is not None else allowed[0]
    if cpu not in allowed: parser.error('CPU is outside allowed affinity')
    workers = {}
    output = dict(schema='astribot.bridge.fake-core-benchmark/1',
        generated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        scope='Offline controller and fake-port cost only; no ROS, real SDK, or hardware evidence.',
        methodology=dict(iterations=args.iterations, warmup_iterations=args.warmup,
            repeats=args.repeats, pinned_cpu=cpu, allowed_cpus=allowed,
            pairing='Persistent workers; Python/C++ order reverses each round, core order rotates.',
            timing='CPU process time and monotonic wall time only inside bulk loops; setup/RSS/JSON/IPC excluded.',
            memory='RSS in KiB before setup, before loop and after loop; VmHWM is cumulative post-exec address-space peak, not per-sample allocation.',
            baseline='Separate empty-loop samples are reported, never subtracted.',
            gripper_unit='One request with immediate fake actuation; commands cycle 0,25,50,75,100.',
            arm_unit='One 2 ms synthetic streaming step with events/feedback drained.',
            chassis_unit='One synthetic 4/8 ms control tick including submit and event drain; original Python timing/locking included; C++ host uses one mutex.',
            trace='All preflight writes, flags, numeric positions, counters and terminal/current state checked before timing.'),
        system=dict(platform=platform.platform(), python=sys.version,
            cpu_model=next((x.split(':',1)[1].strip() for x in Path('/proc/cpuinfo').read_text().splitlines() if x.startswith('model name')), 'unknown'),
            load_average_start=os.getloadavg()), source_sha256=hashes(), samples=[], verification=[],warmup=[],baseline=[])
    try:
        workers['cpp'] = Worker([str(args.driver.resolve())], 'cpp', cpu)
        workers['python'] = Worker([sys.executable, str(Path(__file__).resolve()), '--worker'], 'python', cpu)
        output['startup'] = {name: dict(wall_seconds=w.startup_wall_seconds,pid=w.process.pid) for name,w in workers.items()}
        for kind in KINDS:
            pair = [workers[name].sample('VERIFY', kind, 700 if kind=='chassis' else 300) for name in ('python','cpp')]
            verify_pair(*pair)
            output['verification'].extend(pair)
            for name in ('cpp','python'):
                output['warmup'].append(workers[name].sample('BENCH',kind,args.warmup))
                output['baseline'].append(workers[name].sample('BASE',kind,args.iterations))
        for repeat in range(args.repeats):
            order = ('python','cpp') if repeat % 2 == 0 else ('cpp','python')
            for kind in KINDS[repeat%3:] + KINDS[:repeat%3]:
                pair = []
                for name in order:
                    result = workers[name].sample('BENCH',kind,args.iterations)
                    result['round'] = repeat
                    result['order_in_pair'] = order.index(name)
                    output['samples'].append(result)
                    pair.append(result)
                verify_pair(*pair)
        output['summary'] = {}
        for kind in KINDS:
            summary = {}
            for name in ('python','cpp'):
                rows = [r for r in output['samples'] if r['kind']==kind and r['language']==name]
                wall = [r['wall_seconds']/r['iterations']*1e9 for r in rows]
                cpu_values = [r['cpu_seconds']/r['iterations']*1e9 for r in rows]
                summary[name] = dict(wall_ns_per_operation_median=statistics.median(wall),
                    wall_ns_per_operation_min=min(wall),wall_ns_per_operation_max=max(wall),
                    cpu_ns_per_operation_median=statistics.median(cpu_values),
                    rss_ready_kib_median=statistics.median(r['rss_ready_kib'] for r in rows),
                    rss_after_kib_median=statistics.median(r['rss_after_kib'] for r in rows),
                    process_peak_rss_kib_max=max(r['process_peak_rss_kib'] for r in rows))
            summary['wall_speedup_python_over_cpp'] = summary['python']['wall_ns_per_operation_median']/summary['cpp']['wall_ns_per_operation_median']
            summary['cpu_speedup_python_over_cpp'] = summary['python']['cpu_ns_per_operation_median']/summary['cpp']['cpu_ns_per_operation_median']
            output['summary'][kind] = summary
        output['semantic_checks'] = 'passed: full preflight traces and every measured pair snapshot'
        output['driver_sha256'] = hashlib.sha256(args.driver.read_bytes()).hexdigest()
        build_dir = args.driver.parent
        output['build'] = {str(p.relative_to(build_dir)):p.read_text() for p in [
            build_dir/'CMakeFiles/bridge_runtime.dir/flags.make',
            build_dir/'CMakeFiles/benchmark_standalone_driver.dir/flags.make',
            build_dir/'CMakeFiles/benchmark_standalone_driver.dir/link.txt'] if p.exists()}
        output['compiler_version'] = subprocess.check_output(['c++','--version'],text=True).splitlines()[0]
        output['system']['load_average_end'] = os.getloadavg()
    finally:
        output['worker_stderr'] = {name:w.close() for name,w in workers.items()}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output':str(args.output),'summary':output.get('summary',{})},indent=2))


if __name__ == '__main__':
    main()
