#!/usr/bin/env python3
"""Own one simulation session; start navigation only after measured data is ready."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time


def process_identity(pid):
    try:
        fields=Path(f'/proc/{pid}/stat').read_text().rsplit(') ',1)[1].split()
        return int(fields[1]),int(fields[19]),fields[0]
    except (FileNotFoundError,ProcessLookupError,PermissionError):
        return None


def owned_descendants(root_pids):
    """Capture parentage and start times before launch processes can orphan children."""
    table={}
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            identity=process_identity(int(entry.name))
            if identity is not None:table[int(entry.name)]=identity
    owned=set(root_pids)&table.keys()
    while True:
        added={pid for pid,identity in table.items() if identity[0] in owned}-owned
        if not added:break
        owned.update(added)
    return {pid:table[pid][1] for pid in owned}


def live_owned(identities):
    return {pid:start for pid,start in identities.items()
            if (current:=process_identity(pid)) is not None and current[1]==start and current[2]!='Z'}


def finish_descendants(identities,term_timeout=3.):
    remaining=live_owned(identities)
    for sig,timeout in ((signal.SIGTERM,term_timeout),(signal.SIGKILL,1.)):
        for pid in remaining:
            # A pidfd binds the signal to the captured process, even if a PID is reused.
            try:
                fd=os.pidfd_open(pid)
                try:
                    if pid in live_owned({pid:identities[pid]}):signal.pidfd_send_signal(fd,sig)
                finally:os.close(fd)
            except ProcessLookupError:pass
        deadline=time.monotonic()+timeout
        while remaining and time.monotonic()<deadline:
            time.sleep(.05);remaining=live_owned(identities)
        if not remaining:break
    return list(remaining)


def arguments():
    p = argparse.ArgumentParser()
    p.add_argument('--mode', choices=['mapping', 'explore', 'localize', 'baseline'], default='mapping')
    p.add_argument('--map', default='')
    p.add_argument('--map-yaml', default='')
    p.add_argument('--navigation-policy', choices=['off', 'p2'], default='off')
    p.add_argument('--tracker', choices=['mppi', 'rpp'], default='mppi')
    p.add_argument('--max-linear-speed', type=float, default=0.35)
    p.add_argument('--scan-source', choices=['slice_scan', 'laserscan'], default='slice_scan')
    p.add_argument('--headless', action='store_true')
    p.add_argument('--no-rviz', action='store_true')
    p.add_argument('--nav-transport', choices=['udp', 'default'], default='udp')
    p.add_argument('--nav-attempts', type=int, choices=range(1,4), default=2)
    p.add_argument('--ready-timeout', type=float, default=120)
    p.add_argument('--log-dir', default='')
    p.add_argument('--dry-run', action='store_true')
    a = p.parse_args()
    if not 0 < a.max_linear_speed <= 1.5 or a.ready_timeout <= 0:
        p.error('speed must be in (0, 1.5] and readiness timeout must be positive')
    if a.mode == 'localize' and not a.map:
        p.error('--mode localize requires --map (serialized SLAM map base path)')
    if a.map_yaml and not Path(a.map_yaml).is_file():
        p.error('--map-yaml does not exist')
    return a


def main():
    a = arguments()
    repo = Path(__file__).resolve().parents[1]
    run = Path(a.log_dir or f'/tmp/astribot_sim_{time.strftime("%Y%m%d_%H%M%S")}').resolve()
    sim = ['ros2', 'launch', 'astribot_s1_navigation', 'nav2_full_bringup.launch.py',
           'env:=sim', 'launch_gazebo:=true', 'launch_navigation:=false',
           f'mode:={"localization" if a.mode == "localize" else "mapping"}',
           f'exploration:={str(a.mode == "explore").lower()}',
           f'scan_source:={a.scan_source}', f'headless:={str(a.headless).lower()}',
           f'use_rviz:={str(not a.no_rviz).lower()}']
    if a.map:
        sim += [f'map_file_name:={a.map}']
    if a.mode == 'baseline':
        sim += ['map_source:=real_file', 'localization:=ground_truth',
                'map_yaml_path:=' + (a.map_yaml or str(repo / 'maps/warehouse_baseline.yaml'))]
    elif a.map_yaml:
        sim += ['map_source:=real_file', f'map_yaml_path:={a.map_yaml}']
    nav = ['ros2', 'launch', 'astribot_s1_navigation', 'navigation.launch.py',
           'use_sim_time:=true', f'controller_plugin:={a.tracker}',
           f'max_linear_speed:={a.max_linear_speed}', f'navigation_policy_stage:={a.navigation_policy}',
           'scan_topic:=' + ('/scan_from_cloud' if a.scan_source=='slice_scan' else '/scan')]
    if a.dry_run:
        print(json.dumps({'simulation': sim, 'navigation': nav, 'logs': str(run), 'nav_transport': a.nav_transport}, indent=2))
        return
    lock = open('/tmp/astribot_sim_domain25.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    # Detect direct executables, without matching shell command text or killing other sessions.
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            args = (proc / 'cmdline').read_bytes().split(b'\0')
            if args and Path(os.fsdecode(args[0])).name in ('controller_server', 'robot_state_publisher'):
                env = (proc / 'environ').read_bytes().split(b'\0')
                if b'ROS_DOMAIN_ID=25' in env:
                    raise RuntimeError(f'existing stack PID {proc.name}; stop its owner before starting another')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    run.mkdir(parents=True, exist_ok=False)
    children, logs = [], []
    owned_processes = {}
    def remember_children():
        owned_processes.update(owned_descendants([c.pid for c in children]))
    manifest = {'supervisor_pid': os.getpid(), 'started': time.time(), 'state': 'starting', 'children': []}
    def save():
        (run / 'session.json').write_text(json.dumps(manifest, indent=2))
    def spawn(name, cmd, env):
        log = open(run / f'{name}.log', 'w'); logs.append(log)
        child = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(child)
        manifest['children'].append({'name': name, 'pid': child.pid, 'command': cmd})
        save()
        return child
    def stopped(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stopped)
    signal.signal(signal.SIGINT, stopped)
    try:
        spawn('simulation', sim, os.environ.copy())
        end = time.monotonic() + a.ready_timeout
        while time.monotonic() < end:
            remember_children()
            result = subprocess.run(['timeout', '4', 'ign', 'topic', '-e', '-t', '/stats', '-n', '1'], capture_output=True, text=True)
            if 'iterations:' in result.stdout and 'iterations: 0' not in result.stdout:
                (run / 'gazebo_stats.txt').write_text(result.stdout)
                break
            if children[0].poll() is not None:
                raise RuntimeError('simulation launch exited')
        else:
            raise RuntimeError('Gazebo physics did not advance; inspect simulation.log')
        def probe(phase, env):
            cmd = ['python3', str(repo/'tools/sim_stack_probe.py'), '--phase', phase,
                   '--timeout', str(a.ready_timeout), '--scan',
                   '/scan_from_cloud' if a.scan_source=='slice_scan' else '/scan']
            if phase=='navigation' and a.navigation_policy!='off':
                cmd+=['--costmap-scan','/navigation_policy/costmap_scan']
            log = open(run/f'probe_{phase}_{time.time_ns()}.log', 'w'); logs.append(log)
            process = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=log, text=True, start_new_session=True)
            children.append(process)
            try:
                probe_end=time.monotonic()+a.ready_timeout+5
                while True:
                    remember_children()
                    remaining=probe_end-time.monotonic()
                    if remaining<=0:raise subprocess.TimeoutExpired(cmd,a.ready_timeout+5)
                    try:
                        stdout,_=process.communicate(timeout=min(.5,remaining))
                        break
                    except subprocess.TimeoutExpired:
                        if time.monotonic()>=probe_end:raise
                result = json.loads(stdout.strip().splitlines()[-1]) if stdout.strip() else {'ready':False}
                log.write(stdout);log.flush()
                return result
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL);process.wait()
                return {'ready':False, 'error':'ROS probe exceeded external wall-clock watchdog'}
            finally:
                if process.poll() is not None:
                    children.remove(process)
        measured = probe('data', os.environ.copy())
        if not measured.get('ready'):
            raise RuntimeError(f'clock/TF/scan/odom not ready: {measured}')
        manifest['pre_navigation'] = measured
        env = os.environ.copy()
        if a.nav_transport == 'udp':
            profile = run / 'nav_udp.xml'
            profile.write_text('''<?xml version="1.0"?><profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><transport_descriptors><transport_descriptor><transport_id>nav_udp</transport_id><type>UDPv4</type></transport_descriptor></transport_descriptors><participant profile_name="nav" is_default_profile="true"><rtps><userTransports><transport_id>nav_udp</transport_id></userTransports><useBuiltinTransports>false</useBuiltinTransports></rtps></participant></profiles>''')
            env['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
        # Queries use the exact environment given to this owned navigation process.
        (run / 'env.sh').write_text('source /opt/ros/humble/setup.bash\nsource ' + shlex.quote(str(repo/'ws_robot/install/setup.bash')) + '\n' + ''.join('export '+k+'='+shlex.quote(v)+'\n' for k,v in env.items() if k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE','IGN_IP','GZ_IP')))
        for attempt in range(1, a.nav_attempts+1):
            child = spawn(f'navigation_{attempt}', nav, env)
            measured = probe('navigation', env)
            states = measured.get('lifecycle', {})
            if measured.get('ready'):
                manifest['navigation_measurements'] = measured
                break
            manifest.setdefault('startup_failures', []).append({'attempt':attempt, 'states':states,
                                                               'measurement':measured})
            save()
            if attempt==a.nav_attempts:
                raise RuntimeError(f'Navigation readiness failed after {attempt} attempts: {measured}')
            print(f'Navigation readiness failed: {measured}; restarting only owned navigation group {child.pid}', flush=True)
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL); child.wait()
            children.remove(child)
            # Let discovery dispose old endpoints before creating replacements.
            time.sleep(2)
        manifest.update(state='ready', lifecycle=states, ready_at=time.time()); save()
        print(f'READY {run} lifecycle={states}', flush=True)
        while all(c.poll() is None for c in children):
            remember_children()
            time.sleep(1)
        raise RuntimeError('a launch exited after readiness')
    except KeyboardInterrupt:
        manifest['state'] = 'stopped'
    except Exception as exc:
        manifest.update(state='failed', error=str(exc)); raise
    finally:
        remember_children()
        descendants=owned_processes.copy()
        for c in reversed(children):
            try:
                os.killpg(c.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for c in reversed(children):
            try:
                c.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(c.pid, signal.SIGTERM)
                try:
                    c.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(c.pid, signal.SIGKILL); c.wait()
        manifest['remaining_owned_pids']=finish_descendants(descendants)
        if manifest['remaining_owned_pids']:
            manifest.update(state='cleanup_failed',error='owned child processes did not exit')
        manifest['ended'] = time.time(); save()
        for log in logs:
            log.close()

if __name__ == '__main__':
    main()
