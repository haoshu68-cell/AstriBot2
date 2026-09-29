#!/usr/bin/env python3
"""Serial cold-start Gazebo regressions; only stop supervisors created here."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim_isolation import SimulationIsolation
from sim_camera_preset import DEFAULT_RELATIVE_PRESET, load_camera_preset
from owned_command import run_owned_capture

ROOT = Path(__file__).resolve().parents[2]
CORE = ('r01_cancel_waiting', 'r01_cancel_moving', 'h2_empty', 'h2_cross_left',
        'h2_two_cross', 'h2_input_loss', 'h2_goal_occupied')
EMPTY_CASES = ('h2_empty','s01_yaw90','s01_l','s01_u')
HEADING_CASES = tuple(f's01_start_{sign}_{shape}' for sign in ('left','right') for shape in ('straight','l','u'))
EMPTY_CASES += HEADING_CASES
STOP_CASES = tuple(p.stem for p in sorted((ROOT/'tools/social_navigation/cases').glob('r03_*.json')))
CORRIDOR_CASES = ('r02_fixed_p4_13','r02_fixed_p5_13','r02_cancel_moving',
                  'r02_legacy_p4_13','r02_legacy_p5_13','r02_legacy_p4_18','r02_legacy_p5_18',
                  'r02_legacy_p4_grid18','r02_legacy_p5_grid18')
PROBE_CASES = STOP_CASES + ('oracle_geometry',) + CORRIDOR_CASES
ENV_KEYS = ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION',
            'FASTRTPS_DEFAULT_PROFILES_FILE', 'IGN_IP', 'GZ_IP', 'IGN_PARTITION', 'GZ_PARTITION',
            'IGN_DISCOVERY_MSG_PORT', 'IGN_DISCOVERY_SRV_PORT', 'ASTRIBOT_SIM_INSTANCE',
            'DISPLAY', 'XAUTHORITY', 'LP_NUM_THREADS')


def save(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def descendants(pid):
    parents = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            parents[int(entry.name)] = int((entry/'stat').read_text().rsplit(')', 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            pass
    result = {pid}
    while True:
        expanded = result | {child for child, parent in parents.items() if parent in result}
        if expanded == result:
            return result
        result = expanded


def is_primary_slice_process(command):
    """Recognize the root slice node without counting renamed camera filters."""
    args = [os.fsdecode(item) for item in command if item]
    default = 'pointcloud_slice_scan_node'
    if not args or Path(args[0]).name != default:
        return False
    rules = []
    in_ros_args = False
    index = 1
    while index < len(args):
        token = args[index]
        index += 1
        if token == '--ros-args':
            in_ros_args = True
            continue
        if token == '--':
            in_ros_args = False
            continue
        if not in_ros_args:
            if ':=' not in token or token.startswith('-'):
                continue
            rule = token  # rcl also accepts legacy bare remaps outside --ros-args.
        elif token in ('-r', '--remap'):
            if index >= len(args):
                return False
            rule = args[index]
            index += 1
        elif token.startswith('--remap='):
            return False  # Humble treats this as an unparsed ROS argument.
        else:
            continue
        key, separator, value = rule.partition(':=')
        if not separator:
            return False
        scope, scoped, identity = key.rpartition(':')
        if not scoped:
            scope, identity = None, key
        if identity in ('__node', '__name', '__ns'):
            rules.append((scope, '__node' if identity == '__name' else identity, value))
    # Humble rcl uses the first matching rule, including node-specific remaps.
    def remapped(identity, fallback):
        return next((value for scope, key, value in rules
                     if key == identity and scope in (None, default)), fallback)
    return remapped('__node', default) == default and remapped('__ns', '/') == '/'


def runtime_environment(supervisor, directory, overlay, isolation):
    owned = descendants(supervisor)
    candidates = []
    for pid in owned:
        try:
            args = (Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0')
            if args and Path(os.fsdecode(args[0])).name == 'controller_server':
                candidates.append(pid)
        except OSError:
            pass
    if len(candidates) != 1:
        raise RuntimeError('Expected exactly one controller belonging to this supervisor')
    pid = candidates[0]
    raw = (Path('/proc')/str(pid)/'environ').read_bytes().split(b'\0')
    environment = dict(os.fsdecode(item).split('=', 1) for item in raw if b'=' in item)
    if environment.get('ROS_DOMAIN_ID') != str(isolation.domain):
        raise RuntimeError('Unexpected simulation DDS environment')
    if isolation.instance and any(environment.get(k) != v for k, v in isolation.environment().items()
                                  if k != 'ROS_LOCALHOST_ONLY'):
        raise RuntimeError('Live navigation process differs from requested isolated identity')
    if environment.get('ROS_LOCALHOST_ONLY') != '1':
        profile = ET.parse(environment['FASTRTPS_DEFAULT_PROFILES_FILE'])
        ns = {'f': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
        if ([n.text for n in profile.findall('.//f:interfaceWhiteList/f:address', ns)] != ['127.0.0.1'] or
                [n.text for n in profile.findall('.//f:useBuiltinTransports', ns)] != ['false']):
            raise RuntimeError('Navigation UDP profile is not the expected loopback transport')
    libraries = {}
    for line in (Path('/proc')/str(pid)/'maps').read_text().splitlines():
        fields = line.split()
        if len(fields) >= 6 and fields[-1].startswith('/') and 'astribot' in fields[-1]:
            path = Path(fields[-1])
            if path.is_file():
                libraries[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    save(directory/'runtime_manifest.json', dict(controller_pid=pid,
        environment={k: environment[k] for k in ENV_KEYS if k in environment}, libraries=libraries))
    planners = []
    for child in owned:
        try:
            command = (Path('/proc')/str(child)/'cmdline').read_bytes().split(b'\0')
            if command and Path(os.fsdecode(command[0])).name == 'planner_server':
                planners.append(child)
        except OSError:
            pass
    if len(planners) != 1:
        raise RuntimeError('Expected exactly one planner belonging to this supervisor')
    planner = planners[0]
    planner_libraries = {}
    for line in (Path('/proc')/str(planner)/'maps').read_text().splitlines():
        fields = line.split()
        if len(fields) >= 6 and fields[-1].startswith('/') and 'astribot' in fields[-1]:
            path = Path(fields[-1])
            if path.is_file():
                planner_libraries[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    save(directory/'planner_runtime_manifest.json', dict(planner_pid=planner,
        libraries=planner_libraries))
    perception = []
    for child in owned:
        try:
            command = (Path('/proc')/str(child)/'cmdline').read_bytes().split(b'\0')
            if is_primary_slice_process(command):
                perception.append(child)
        except OSError:
            pass
    if len(perception) != 1:
        raise RuntimeError('Expected exactly one root slice node belonging to this supervisor')
    perception_libraries = {}
    for line in (Path('/proc')/str(perception[0])/'maps').read_text().splitlines():
        fields = line.split()
        if len(fields) >= 6 and fields[-1].startswith('/') and 'astribot' in fields[-1]:
            path = Path(fields[-1])
            if path.is_file():
                perception_libraries[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    save(directory/'perception_runtime_manifest.json', dict(perception_pid=perception[0],
        libraries=perception_libraries))
    prefix = 'set -e\nset +u\nsource ' + shlex.quote(str(ROOT/'tools/ros_overlay_env.sh')) + \
        '\nastribot_prepare_overlay_environment\nsource /opt/ros/humble/setup.bash\nsource ' + shlex.quote(
        str(ROOT/'ws_robot/install/setup.bash')) + '\nsource ' + shlex.quote(str(overlay)) + '\n'
    prefix += '\n'.join('export ' + k + '=' + shlex.quote(environment[k])
                        for k in ENV_KEYS if k in environment) + '\n'
    (directory/'query_env.sh').write_text(prefix)
    return prefix


def stop_owned(process, timeout):
    if process is not None and process.poll() is None:
        handles=[]
        for pid in descendants(process.pid):
            try:handles.append(os.pidfd_open(pid))
            except ProcessLookupError:pass
        try:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                # Bind fallback signals to captured processes, not names or a
                # potentially reused PID. This also covers the owned nav runner.
                for sig in (signal.SIGTERM,signal.SIGKILL):
                    for fd in handles:
                        try:signal.pidfd_send_signal(fd,sig)
                        except ProcessLookupError:pass
                    try:process.wait(timeout=3)
                    except subprocess.TimeoutExpired:continue
                    if sig==signal.SIGKILL:break
                raise RuntimeError('Owned process required forced cleanup; run is invalid')
        finally:
            for fd in handles:os.close(fd)


def run_case(case_path, policy, directory, overlay, isolation, *, headless=False,
             capture_visual=True, exclusive_performance=False, camera_preset=None,
             visual_display=None):
    directory.mkdir(parents=True, exist_ok=False)
    case = read_json(case_path)
    scene = (ROOT/case['scene']).resolve()
    (directory/'scene.yaml').write_bytes(scene.read_bytes())
    camera_preset = Path(camera_preset or ROOT/DEFAULT_RELATIVE_PRESET).resolve()
    camera_parameters, camera_baseline = load_camera_preset(camera_preset)
    save(directory/'camera_baseline.json', camera_baseline)
    tracked = list((ROOT/'tools/social_navigation').rglob('*.py')) + [case_path, scene, overlay] + [
        ROOT/'tools'/name for name in ('run_waypoint_route.py','sim_stack_supervisor.py','sim_stack_probe.py','sim_isolation.py','launch_sim_stack.sh','ros_overlay_env.sh')]
    tracked += [camera_preset, ROOT/'tools/sim_camera_preset.py',
                Path(camera_parameters['camera_mounts_profile'])]
    tracked += list(Path(camera_parameters['camera_calibration_dir']).glob('camera_*.yaml'))
    if case.get('kind') in ('corridor_probe','legacy_corridor_probe'):
        tracked += [ROOT/'tools/sim/verify_fixed_hold_expiry.py',ROOT/case['corridor_file'],
            ROOT/'ws_robot/src/astribot_s1_transport/config/warehouse_transfer.json']
    save(directory/'input_manifest.json', {
        str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked})
    env = os.environ.copy()
    env.update(ASTRIBOT_OVERLAY_SETUP=str(overlay), DISPLAY=env.get('DISPLAY', ':1'))
    if headless:
        # Prefer the host NVIDIA EGL device for off-screen sensor rendering.
        # This is an acceleration hint only; physics, controller limits and
        # freshness guards remain unchanged.  If the vendor manifest is not
        # installed, keep the normal loader selection and record the fallback.
        vendor_manifest = Path('/usr/share/glvnd/egl_vendor.d/10_nvidia.json')
        env.update(EGL_PLATFORM='surfaceless', LIBGL_ALWAYS_SOFTWARE='0',
                   __GLX_VENDOR_LIBRARY_NAME='nvidia')
        if vendor_manifest.is_file():
            env['__EGL_VENDOR_LIBRARY_FILENAMES'] = str(vendor_manifest)
    launch = runner = None
    result = dict(case=case['id'], policy=policy, directory=str(directory), verdict='INFRA_FAILURE')
    try:
        launch_args=['bash', str(ROOT/'tools/launch_sim_stack.sh'), '--mode', 'baseline',
                '--instance', isolation.instance, '--ros-domain-id', str(isolation.domain),
                '--spawn-x', str(case.get('initial_xy_m',[0.,0.])[0]),
                '--spawn-y', str(case.get('initial_xy_m',[0.,0.])[1]),
                '--spawn-yaw', str(case.get('initial_yaw_rad',0.)),
                '--social-policy', policy, '--social-scenario', str(scene), '--ready-timeout', '180',
                '--camera-preset', str(camera_preset),
                '--log-dir', str(directory/'stack')]
        # A shared workstation may contain unrelated GUI-heavy stacks.  Keep
        # the functional regression strict about source freshness and stop
        # budgets, while allowing an explicitly recorded headless run to avoid
        # starving Gazebo's physics thread.  Visual evidence is collected in a
        # separate run with the normal GUI path.
        if headless:
            launch_args += ['--headless', '--no-rviz']
        if exclusive_performance:
            launch_args += ['--exclusive-performance']
        if case.get('kind') in ('corridor_probe','legacy_corridor_probe'):
            corridor=ROOT/case['corridor_file']
            (directory/'corridor.json').write_bytes(corridor.read_bytes())
            launch_args+=['--navigation-policy',case['navigation_policy'],
                '--navigation-geometry-mode',case.get('navigation_geometry_mode','fixed_v2'),
                '--corridor-file',str(directory/'corridor.json')]
        with (directory/'launcher.log').open('w') as log:
            launch = subprocess.Popen(launch_args, cwd=ROOT, env=env,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        until = time.monotonic() + 200
        while launch.poll() is None and time.monotonic() < until:
            session = read_json(directory/'stack/session.json')
            if session.get('state') == 'ready':
                break
            time.sleep(.5)
        else:
            raise RuntimeError('Supervisor did not reach measured readiness; see launcher.log')
        if session.get('supervisor_pid') != launch.pid:
            raise RuntimeError('Session ownership does not match launched supervisor')
        prefix = runtime_environment(launch.pid, directory, overlay, isolation)
        physics = subprocess.run(['bash', '-c', prefix +
            'exec timeout 15 ign topic -e -t /stats -n 1'], capture_output=True, text=True, timeout=20)
        (directory/'physics.txt').write_text(physics.stdout + physics.stderr)
        if physics.returncode or 'iterations:' not in physics.stdout:
            raise RuntimeError('Gazebo stepping evidence unavailable')
        runner_name = {'stop_probe':'run_stop_probe.py','geometry_oracle':'run_geometry_oracle.py',
                      'corridor_probe':'run_corridor_probe.py',
                      'legacy_corridor_probe':'run_legacy_corridor_probe.py'}.get(case.get('kind'),'run_episode.py')
        runner_args = [str(ROOT/'tools/social_navigation'/runner_name),
                       '--case', str(case_path), '--policy', policy, '--output', str(directory/'episode')]
        if runner_name != 'run_episode.py':
            runner_args += ['--session', str(directory/'stack/session.json')]
        command = prefix + 'exec python3 ' + shlex.join(runner_args)
        with (directory/'episode.log').open('w') as log:
            runner = subprocess.Popen(['bash', '-c', command], cwd=ROOT, env=env,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        print('RUNNING', directory.name, flush=True)
        runner.wait(timeout=case.get('wall_watchdog_s', 2400) + 100)
        summary = read_json(directory/'episode/summary.json')
        result.update(verdict=summary.get('verdict', 'INFRA_FAILURE'), returncode=runner.returncode,
                      failed_checks=summary.get('failed_checks', []))
        if runner.returncode or not summary.get('scenario_passed'):
            if result['verdict'] == 'PASS':
                result['verdict'] = 'INFRA_FAILURE'
        if case.get('kind') in ('corridor_probe', 'legacy_corridor_probe'):
            # Bound the read-only capture; retain fresh grid/scan data alongside
            # endpoint views so a faded wall cannot be classified from pixels alone.
            snapshot = subprocess.run(['bash', '-c', prefix + 'exec python3 ' + shlex.join([
                str(ROOT/'tools/social_navigation/capture_map_snapshot.py'),
                '--output', str(directory/'post_fixture_snapshot'), '--wall-seconds', '20'])],
                env=env, capture_output=True, text=True, timeout=35)
            (directory/'map_snapshot.log').write_text(snapshot.stdout + snapshot.stderr)
            result['post_fixture_snapshot'] = 'RECORDED' if snapshot.returncode == 0 else 'CAPTURE_FAILED'
        if capture_visual:
            capture_script = 'capture_deferred_views.py' if headless else 'capture_views.py'
            capture_prefix = prefix
            if headless and visual_display:
                capture_prefix += 'export DISPLAY=' + shlex.quote(visual_display) + '\nunset XAUTHORITY\n'
            screenshot = run_owned_capture(['bash', '-c', capture_prefix + 'exec python3 ' + shlex.join([
                str(ROOT/'tools/social_navigation'/capture_script), '--output', str(directory),
                '--supervisor', str(launch.pid)])], directory/'screenshots.log', env=env,
                timeout=90 if headless else 40)
            result['visual_review'] = 'PENDING' if screenshot == 0 else 'CAPTURE_FAILED'
        else:
            result['visual_review'] = 'SKIPPED_HEADLESS'
    except KeyboardInterrupt:
        result.update(verdict='INFRA_FAILURE',error='Regression interrupted')
        raise
    except Exception as error:
        result.update(verdict='INFRA_FAILURE', error=str(error))
    finally:
        errors=[]
        for process,timeout in ((runner,35),(launch,45)):
            try:stop_owned(process,timeout)
            except Exception as error:errors.append(str(error))
        if errors:result.update(verdict='INFRA_FAILURE',cleanup_error='; '.join(errors))
        session = read_json(directory/'stack/session.json')
        result['cleanup_complete'] = (launch is not None and launch.poll() is not None and
            session.get('state') in ('stopped','failed') and bool(session.get('ended')) and
            'remaining_owned_pids' in session and not session['remaining_owned_pids'] and
            not session.get('log_capture_errors'))
        if not result['cleanup_complete']:
            result['verdict'] = 'INFRA_FAILURE'
        save(directory/'result.json', result)
    print('RESULT', json.dumps(result), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--overlay', type=Path, required=True)
    parser.add_argument('--camera-preset', type=Path, default=ROOT/DEFAULT_RELATIVE_PRESET,
                        help='Shared six-camera launch preset for the joint development baseline')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cases', nargs='+', choices=CORE + EMPTY_CASES[1:] + ('h2_cross_right', 'h2_stop_ahead', 'h2_head_on') + PROBE_CASES, default=CORE)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--first-repeat', type=int, default=1,
                        help='First repetition number when collecting missing rounds in a new evidence directory')
    parser.add_argument('--startup-retries', type=int, choices=range(3), default=1,
                        help='Bounded retries only before an episode starts, after confirmed owned cleanup; all failed attempts retained')
    parser.add_argument('--collect-all', action='store_true',
                        help='After confirmed cleanup, collect other cold-start cases even if a test failed; never grants stage acceptance')
    parser.add_argument('--headless', action='store_true',
                        help='Run the functional regression without Gazebo/RViz GUI; keep source and safety freshness checks strict')
    parser.add_argument('--skip-visual', action='store_true',
                        help='Do not capture paired Gazebo/RViz screenshots; use only with an explicit follow-up visual run')
    parser.add_argument('--visual-display', help='Independent display for post-episode observers; does not change the physics server display')
    parser.add_argument('--exclusive-performance', action='store_true',
                        help='Require the supervisor host performance lease for each trial; busy resources remain a startup failure, never a measured PASS')
    parser.add_argument('--instance', default='')
    parser.add_argument('--ros-domain-id', type=int, default=25)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    try:
        isolation = SimulationIsolation(args.instance, args.ros_domain_id)
    except ValueError as error:
        parser.error(str(error))
    if not args.overlay.is_file() or args.repeats < 1 or args.first_repeat < 1:
        parser.error('an existing overlay and positive repeat count are required')
    args.overlay = args.overlay.resolve()
    args.output = args.output.resolve()
    jobs = [(name, policy, repeat) for repeat in range(args.first_repeat, args.first_repeat+args.repeats) for name in args.cases
            for policy in ((('off','h2') if repeat%2 else ('h2','off')) if name in EMPTY_CASES
                           else (('off',) if name in PROBE_CASES else ('h2',)))]
    if args.dry_run:
        print(json.dumps(jobs, indent=2)); return 0
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(status='running', stage_acceptance='PENDING', jobs=jobs, startup_retries=args.startup_retries,
                  collect_all=args.collect_all, headless=args.headless, camera_preset=str(args.camera_preset.resolve()),
                  visual_capture=not args.skip_visual, visual_display=args.visual_display,
                  exclusive_performance=args.exclusive_performance,
                  render_acceleration='nvidia_vendor_hint_display_dependent' if args.headless and Path('/usr/share/glvnd/egl_vendor.d/10_nvidia.json').is_file() else ('headless_default' if args.headless else 'display'),
                  isolation=isolation.environment(), results=[])
    def interrupted(signum, frame):
        report['interrupted_signal']=signum
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    try:
        for name, policy, repeat in jobs:
            attempts=[]
            for attempt in range(args.startup_retries+1):
                suffix='' if attempt==0 else f'_startup_retry_{attempt}'
                directory=args.output/f'{name}_{policy}_{repeat}{suffix}'
                result = run_case(ROOT/'tools/social_navigation/cases'/f'{name}.json', policy,
                                  directory, args.overlay, isolation,
                                  headless=args.headless, capture_visual=not args.skip_visual,
                                  exclusive_performance=args.exclusive_performance,
                                  camera_preset=args.camera_preset, visual_display=args.visual_display)
                attempts.append(dict(result))
                retryable=(result['verdict']=='INFRA_FAILURE' and result.get('cleanup_complete') and
                           not (directory/'episode').exists() and not result.get('cleanup_error'))
                if not retryable:break
            result.update(job=dict(case=name,policy=policy,repeat=repeat),startup_attempts=attempts)
            report['results'].append(result)
            save(args.output/'summary.json', report)
            if result['verdict'] != 'PASS' or result.get('visual_review') == 'CAPTURE_FAILED':
                if not args.collect_all or not result.get('cleanup_complete') or result.get('cleanup_error'):
                    break
        if len(report['results']) == len(jobs) and all(r['verdict'] == 'PASS' and
                r.get('visual_review') in ('PENDING', 'SKIPPED_HEADLESS') for r in report['results']):
            report['status'] = 'episodes_passed'
        else:
            report['status'] = 'blocked_on_regression'
        for case in args.cases:
            if case not in EMPTY_CASES:continue
            case_results=[r for r in report['results'] if r['job']['case']==case]
            if len(case_results)!=2*args.repeats or any(r['verdict']!='PASS' for r in case_results):
                continue
            command = [sys.executable, str(ROOT/'tools/social_navigation/compare_empty_baseline.py')]
            for policy, flag in [('off', '--off'), ('h2', '--on')]:
                command += [flag] + [str(Path(r['directory'])/'episode') for r in report['results']
                    if r['job']['case']==case and r['job']['policy']==policy]
            name='empty_comparison' if case=='h2_empty' else case+'_comparison'
            command += ['--output', str(args.output/(name+'.json'))]
            with (args.output/(name+'.log')).open('w') as log:
                comparison = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
            if comparison.returncode:report['status'] = 'blocked_on_regression'
    except KeyboardInterrupt:
        report['status'] = 'interrupted'
        if 'directory' in locals() and (directory/'result.json').is_file() and not any(
                r['directory']==str(directory) for r in report['results']):
            result=read_json(directory/'result.json')
            result['job']=dict(case=name,policy=policy,repeat=repeat)
            report['results'].append(result)
    finally:
        save(args.output/'summary.json', report)
    return 0 if report['status'] == 'episodes_passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
