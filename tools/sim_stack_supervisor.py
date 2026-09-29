#!/usr/bin/env python3
"""Own one simulation session; start navigation only after measured data is ready."""
import logging
import argparse
import fcntl
import json
import os
import re
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time

from astribot_logging import log_directory, log_level
from astribot_logging.output import SessionLog, SessionHandler, ProcessOutput, publish_session_link
from sim_isolation import SimulationIsolation,is_stack_process
from sim_performance_lease import acquire_performance_lease, simulation_servers, PERFORMANCE_LEASE_PATH
from sim_camera_preset import DEFAULT_RELATIVE_PRESET, PATHS, load_camera_preset
from vision.verify_task_environment import verify_manifest_file


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


def control_environment(environment, transport, profile):
    env = environment.copy()
    if transport == 'udp':
        profile.write_text('''<?xml version="1.0"?><profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><transport_descriptors><transport_descriptor><transport_id>nav_udp</transport_id><type>UDPv4</type><interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList></transport_descriptor></transport_descriptors><participant profile_name="nav" is_default_profile="true"><rtps><userTransports><transport_id>nav_udp</transport_id></userTransports><useBuiltinTransports>false</useBuiltinTransports></rtps></participant></profiles>''')
        env.update(FASTRTPS_DEFAULT_PROFILES_FILE=str(profile),
                   RMW_IMPLEMENTATION='rmw_fastrtps_cpp', ROS_LOCALHOST_ONLY='0')
    return env


def arguments():
    p = argparse.ArgumentParser()
    p.add_argument('--instance',default='',help='Opt-in isolated copy of the same canonical world')
    p.add_argument('--ros-domain-id',type=int,default=25)
    p.add_argument('--spawn-x',type=float,default=0.,help='Initial Gazebo X in metres')
    p.add_argument('--spawn-y',type=float,default=0.,help='Initial Gazebo Y in metres')
    p.add_argument('--spawn-yaw',type=float,default=0.,help='Initial Gazebo heading in radians; simulation fixture only')
    p.add_argument('--launch-slam', choices=['true', 'false'], default='true',
                   help='false requires an external SLAM owner; sensor readiness still waits for its clouds')
    p.add_argument('--initial-chassis-pose', default='',
                   help='Measured stationary map chassis x,y,z,qx,qy,qz,qw; never inferred from spawn')
    p.add_argument('--mode', choices=['mapping', 'explore', 'localize', 'baseline'], default='mapping')
    p.add_argument('--exclusive-performance', action='store_true',
                   help='Reserve this host against other cooperating simulation supervisors')
    p.add_argument('--map', default='', help='Voxel session directory for localization')
    p.add_argument('--save-session', default='', help='new Voxel session directory to save')
    p.add_argument('--match-threshold', type=float, default=0.3)
    p.add_argument('--map-yaml', default='')
    p.add_argument('--navigation-geometry-mode', choices=['legacy','fixed_v2'], default='legacy')
    p.add_argument('--payload-source-id', default='', help='explicit physical inventory source; enables read-only ledger in isolated fixed_v2 session')
    p.add_argument('--navigation-policy', choices=['off', 'p2', 'p3', 'p4', 'p5'], default='off')
    p.add_argument('--corridor-file', default='', help='Map-frame corridor annotations for P4')
    p.add_argument('--social-scenario', default='', help='Optional HuNav YAML in the baseline world; behavior is enabled separately')
    p.add_argument('--social-policy', choices=['off','h2'], default='off',
                   help='H2 social waiting and speed selection; simulation truth is explicitly marked')
    p.add_argument('--tracker', choices=['mppi', 'rpp'], default='mppi')
    p.add_argument('--max-linear-speed', type=float, default=0.35)
    p.add_argument('--scan-source', choices=['slice_scan', 'laserscan'], default='slice_scan')
    p.add_argument('--use-lidar', choices=['true', 'false'], default='true',
                   help='simulation sensor fixture: keep the Livox streams enabled by default')
    p.add_argument('--camera-preset', default=str(Path(__file__).resolve().parents[1]/DEFAULT_RELATIVE_PRESET))
    p.add_argument('--use-camera', choices=['true', 'false'], default=None,
                   help='simulation sensor fixture: keep calibrated cameras enabled by default')
    p.add_argument('--use-wrist-cameras', choices=['true', 'false'], default=None)
    p.add_argument('--use-stereo-cameras', choices=['true', 'false'], default=None)
    p.add_argument('--camera-profile', default=None)
    p.add_argument('--torso-camera-profile', default=None)
    p.add_argument('--camera-calibration-dir', default=None)
    p.add_argument('--camera-mounts-profile', default=None)
    p.add_argument('--use-camera-postprocess', choices=['true', 'false'], default=None)
    p.add_argument('--use-camera-pointcloud', choices=['true', 'false'], default=None)
    p.add_argument('--enable-depth-obstacles', choices=['true', 'false'], default=None,
                   help='require RGB-D points in the local costmap; false is an explicit lidar-only regression')
    p.add_argument('--obstacle-layer-plugin',
                   choices=['nav2_costmap_2d::ObstacleLayer', 'nav2_costmap_2d::VoxelLayer'],
                   default='nav2_costmap_2d::ObstacleLayer',
                   help='costmap plugin; VoxelLayer is an explicit 3-D coverage profile')
    p.add_argument('--headless', action='store_true')
    p.add_argument('--real-time-factor', type=float, default=1.0,
                   help='Gazebo simulation/wall time ratio, (0, 1]; preserves the navigation world and sensor timestamps')
    p.add_argument('--no-rviz', action='store_true')
    p.add_argument('--nav-transport', choices=['udp', 'default'], default='udp')
    p.add_argument('--nav-attempts', type=int, choices=range(1,4), default=2)
    p.add_argument('--ready-timeout', type=float, default=120)
    p.add_argument('--log-dir', default='', help='new session directory under the shared log root by default')
    p.add_argument('--log-level', choices=['debug', 'info', 'warn', 'error', 'fatal'],
                   default=log_level(), help='default: ASTRIBOT_LOG_LEVEL or info')
    p.add_argument('--log-max-bytes', type=int,
                   default=os.environ.get('ASTRIBOT_LOG_MAX_BYTES', '10485760'),
                   help='managed output log rotation size in bytes (default: 10 MiB)')
    p.add_argument('--log-backup-count', type=int,
                   default=os.environ.get('ASTRIBOT_LOG_BACKUP_COUNT', '5'),
                   help='managed output log backup count (default: 5)')
    p.add_argument('--runtime-manifest', default='',
                   help='Optional pinned package/artifact/ELF dependency snapshot; reject drift before spawning')
    p.add_argument('--dry-run', action='store_true')
    a = p.parse_args()
    try:
        parameters, a.camera_baseline = load_camera_preset(a.camera_preset)
    except (OSError, ValueError) as error:
        p.error('camera preset: ' + str(error))
    a.camera_baseline['explicit_overrides'] = {}
    for key, value in parameters.items():
        supplied = getattr(a, key)
        if supplied is None or supplied == '':
            setattr(a, key, value)
        elif supplied != value:
            a.camera_baseline['explicit_overrides'][key] = supplied
    if not all(float('-inf') < value < float('inf') for value in (a.spawn_x,a.spawn_y)):
        p.error('--spawn-x and --spawn-y must be finite')
    if not -3.141592653589793<=a.spawn_yaw<=3.141592653589793:
        p.error('--spawn-yaw must be finite and within [-pi,pi]')
    try:SimulationIsolation(a.instance,a.ros_domain_id)
    except ValueError as error:p.error(str(error))
    if a.log_max_bytes <= 0 or a.log_backup_count <= 0:
        p.error('log-max-bytes and log-backup-count must be positive')
    if not 0 < a.max_linear_speed <= 1.5 or a.ready_timeout <= 0:
        p.error('speed must be in (0, 1.5] and readiness timeout must be positive')
    if a.navigation_geometry_mode == 'fixed_v2' and a.navigation_policy not in ('p4','p5'):
        p.error('fixed_v2 requires corridor policy p4 or p5')
    if a.payload_source_id and (not a.instance or a.navigation_geometry_mode != 'fixed_v2' or
                                not re.fullmatch(r'[A-Za-z0-9_.-]{1,128}', a.payload_source_id)):
        p.error('--payload-source-id requires isolated fixed_v2 and a simple bounded source identity')
    if a.navigation_policy == 'p4' and (not a.corridor_file or not Path(a.corridor_file).is_file()):
        p.error('P4 requires --corridor-file')
    if not 0 < a.real_time_factor <= 1.0:
        p.error('--real-time-factor must be in (0,1]')
    if a.mode == 'localize' and not a.map:
        p.error('--mode localize requires --map (Voxel session directory)')
    if not 0 < a.match_threshold < 1:
        p.error('--match-threshold must be in (0, 1)')
    if a.map and not (Path(a.map)/'alidarState.txt').is_file():
        p.error('--map must contain alidarState.txt and kf/')
    if a.save_session and Path(a.save_session).exists():
        p.error('--save-session must be a new directory')
    if a.mode == 'localize' and a.save_session and Path(a.save_session).resolve().parent != Path(a.map).resolve().parent:
        p.error('loaded and saved sessions must share a parent directory')
    if a.mode == 'baseline' and (a.map or a.save_session):
        p.error('baseline uses a static YAML and cannot load/save a Voxel session')
    if a.map and a.save_session and Path(a.save_session).resolve().parent != Path(a.map).resolve().parent:
        p.error('loaded and saved Voxel sessions must have the same parent directory')
    if a.map_yaml and a.mode != 'baseline':
        p.error('--map-yaml is only for --mode baseline; localization requires --map')
    if a.map_yaml and not Path(a.map_yaml).is_file():
        p.error('--map-yaml does not exist')
    if a.social_scenario and (a.mode != 'baseline' or not Path(a.social_scenario).is_file()):
        p.error('--social-scenario requires baseline mode and an existing YAML file')
    if a.social_policy != 'off':
        if not a.social_scenario or a.navigation_policy not in ('off','p2'):
            p.error('H2 requires --social-scenario and P2 (or default off) navigation policy')
        a.navigation_policy='p2'
    return a


def main():
    a = arguments()
    runtime_preflight = None
    if a.runtime_manifest:
        runtime_preflight = verify_manifest_file(a.runtime_manifest)
        if not runtime_preflight['passed']:
            raise RuntimeError('RUNTIME_MANIFEST_REJECTED: ' + json.dumps(runtime_preflight))
    isolation=SimulationIsolation(a.instance,a.ros_domain_id)
    os.environ.update(isolation.environment())
    os.environ['ASTRIBOT_LOG_LEVEL'] = a.log_level
    os.environ['ASTRIBOT_LOG_MAX_BYTES'] = str(a.log_max_bytes)
    os.environ['ASTRIBOT_LOG_BACKUP_COUNT'] = str(a.log_backup_count)
    repo = Path(__file__).resolve().parents[1]
    log_root = log_directory()
    run = Path(a.log_dir or (log_root / f'sim_{time.strftime("%Y%m%d_%H%M%S")}_{os.getpid()}')).resolve()
    sim = ['ros2', 'launch', 'astribot_s1_navigation', 'nav2_full_bringup.launch.py',
           'env:=sim', 'launch_gazebo:=true', 'launch_navigation:=false',
           f'control_loopback_udp:={str(a.nav_transport == "udp").lower()}',
           # This split supervisor waits for clouds before starting navigation;
           # its sensor child owns SLAM, not the later idle mapping_runtime.
           f'launch_slam:={a.launch_slam}',
           f'navigation_geometry_mode:={a.navigation_geometry_mode}',
           f'mode:={"localization" if a.mode == "localize" else "mapping"}',
           f'exploration:={str(a.mode == "explore").lower()}',
           f'scan_source:={a.scan_source}', f'headless:={str(a.headless).lower()}',
           f'use_rviz:={str(not a.no_rviz).lower()}',
           f'use_lidar:={a.use_lidar}', f'use_camera:={a.use_camera}',
           f'use_camera_postprocess:={a.use_camera_postprocess}',
           f'use_camera_pointcloud:={a.use_camera_pointcloud}',
           f'enable_depth_obstacles:={a.enable_depth_obstacles}',
           f'obstacle_layer_plugin:={a.obstacle_layer_plugin}']
    sim += [f'use_wrist_cameras:={a.use_wrist_cameras}',
            f'use_stereo_cameras:={a.use_stereo_cameras}']
    for name in PATHS:
        if value := getattr(a, name):
            sim.append(name+':='+str(Path(value).expanduser().resolve()))
    sim += [f'ros_domain_id:={a.ros_domain_id}', f'spawn_x:={a.spawn_x}',
            f'spawn_y:={a.spawn_y}', f'spawn_yaw:={a.spawn_yaw}']
    if a.initial_chassis_pose:
        sim.append('initial_chassis_pose:=' + a.initial_chassis_pose)
    if a.instance:
        sim += [f'save_path:={run}/slam_sessions/']
    if a.map:
        session = Path(a.map).expanduser().resolve()
        sim += [f'previous_map:={session.name}:{a.match_threshold}', f'save_path:={session.parent}/']
    if a.social_scenario:
        sim += ['social_scenario:=' + str(Path(a.social_scenario).resolve())]
    if a.save_session:
        session = Path(a.save_session).expanduser().resolve()
        sim += [f'save_path:={session.parent}/', f'map_name:={session.name}', 'save_map:=1']
    if a.mode == 'baseline':
        sim += ['slam_backend:=static_map',
                'map_yaml_path:=' + (a.map_yaml or str(repo / 'maps/warehouse_baseline.yaml'))]
    nav = ['ros2', 'launch', 'astribot_s1_navigation', 'navigation.launch.py',
           'use_sim_time:=true', f'controller_plugin:={a.tracker}',
           f'max_linear_speed:={a.max_linear_speed}', f'navigation_policy_stage:={a.navigation_policy}',
           f'navigation_geometry_mode:={a.navigation_geometry_mode}',
           'scan_topic:=' + ('/scan_from_cloud' if a.scan_source=='slice_scan' else '/scan'),
           f'enable_depth_obstacles:={a.enable_depth_obstacles}',
           f'obstacle_layer_plugin:={a.obstacle_layer_plugin}']
    if a.payload_source_id:
        nav += ['enable_payload_ledger:=true', 'payload_environment:=simulation',
                'payload_session_id:='+a.instance, 'payload_source_id:='+a.payload_source_id,
                'payload_journal_path:='+str(run/'payload_ledger.jsonl')]
    if a.mode == 'baseline':
        nav += ['simulation_static_map_yaml:=' + (a.map_yaml or str(repo / 'maps/warehouse_baseline.yaml'))]
        nav += ['arrival_precision_profile:=simulation_precision']
    if a.social_scenario:
        nav += ['obstacle_layer_plugin:=astribot_s1_autonomy::ObservedRayObstacleLayer']
    if a.social_policy != 'off':
        nav += ['social_navigation_stage:='+a.social_policy,'social_allow_simulation_truth:=true']
    if a.corridor_file:nav += ['corridor_file:=' + str(Path(a.corridor_file).resolve())]
    if a.instance:
        nav += ['map_manager_params_file:='+str(run/'map_manager.yaml'),
                'operator_runtime_params_file:='+str(run/'mapping_runtime.yaml'),
                'enable_voxel_adapter:=true',
                'voxel_adapter_params_file:='+str(run/'voxel_session_adapter.yaml')]
    # The split mapping workflow already owns SLAM in the sensor child, but
    # needs its formal C++ session owner for zones and eventual archival.
    # Exploration owns this node in its launch; fixed/localized maps use their
    # map context instead. Never start another SLAM instance to fill this gap.
    mapping_session = (['ros2', 'run', 'astribot_s1_exploration', 'mapping_session_node',
                        '--ros-args', '-p', 'use_sim_time:=true', '-p',
                        'require_navigation_zones:=true'] if a.mode == 'mapping' else None)
    if a.dry_run:
        print(json.dumps({'simulation': sim, 'navigation': nav, 'mapping_session': mapping_session, 'logs': str(run),
                          'isolation':isolation.environment(),'locks':isolation.lock_paths(),
                          'performance_lease': {'path':str(PERFORMANCE_LEASE_PATH),
                                                'exclusive':a.exclusive_performance},
                          'real_time_factor': a.real_time_factor,
                          'nav_transport': a.nav_transport,
                          'camera_baseline': a.camera_baseline,
                          'runtime_preflight': runtime_preflight,
                          'logging': {'layout': 'unified', 'file': str(run / 'session.log'), 'level': a.log_level, 'max_bytes': a.log_max_bytes,
                                      'backup_count': a.log_backup_count}}, indent=2))
        return
    locks=[acquire_performance_lease(a.exclusive_performance)]
    if a.exclusive_performance and (existing_servers := simulation_servers()):
        raise RuntimeError('EXCLUSIVE_PERFORMANCE_REQUIRES_IDLE_HOST: existing Gazebo PIDs '+
                           ','.join(str(item['pid']) for item in existing_servers))
    for path in isolation.lock_paths():
        lock=open(path,'a');fcntl.flock(lock,fcntl.LOCK_EX | fcntl.LOCK_NB);locks.append(lock)
    # Detect direct executables, without matching shell command text or killing other sessions.
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            args = (proc / 'cmdline').read_bytes().split(b'\0')
            if is_stack_process([os.fsdecode(arg) for arg in args]):
                env=dict(item.decode().split('=',1) for item in (proc/'environ').read_bytes().split(b'\0') if b'=' in item)
                conflict=isolation.conflict(env)
                if conflict:
                    raise RuntimeError(f'{conflict}: existing stack PID {proc.name}; select an unused isolated identity')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    run.mkdir(parents=True, exist_ok=False)
    if a.instance:
        # JSON is valid YAML; paths are quoted without shell interpolation.
        (run/'map_manager.yaml').write_text(json.dumps({'map_manager':{'ros__parameters':{
            'storage_root':str(run/'map_catalog'),'import_root':str(run/'slam_sessions')}}}))
        (run/'mapping_runtime.yaml').write_text(json.dumps({'mapping_runtime':{'ros__parameters':{
            'profile':'','save_path':str(run/'slam_sessions')}}}))
        (run/'voxel_session_adapter.yaml').write_text(json.dumps({'map_session_adapter':{'ros__parameters':{
            'profile':'sim', 'allow_navigation_reconfigure':True,
            'asset_root':str(run/'map_catalog/assets'),
            'runtime_root':str(run/'voxel_activation'),
            'map_topic':'/map', 'odom_topic':'/odom',
            'odom_frame':'odom', 'robot_base_frame':'astribot_torso_base'}}}))
    os.environ['ASTRIBOT_LOG_DIR'] = str(run)
    os.environ['ROS_LOG_DIR'] = str(run)
    os.environ['ASTRIBOT_LOG_CAPTURE'] = '1'
    output_path = run / 'session.log'
    session_log = SessionLog(output_path, 'session')
    logger = logging.Logger('astribot.supervisor', getattr(logging, a.log_level.upper()))
    logger.addHandler(SessionHandler(output_path))
    logger.addHandler(logging.StreamHandler())
    # Print before readiness probes, even when the configured severity hides INFO.
    print(f'统一日志（spdlog）：{output_path}', file=sys.stderr, flush=True)
    try:
        # Keep another owner's default latest_sim index stable.
        latest = publish_session_link(log_root / a.instance if a.instance else log_root, run)
        print(f'最新仿真日志索引：{latest / "session.log"}', file=sys.stderr, flush=True)
    except OSError as exc:
        logger.warning('Unable to publish latest_sim log index: %s', exc)
    children, logs = [], []
    owned_processes = {}
    def remember_children():
        owned_processes.update(owned_descendants([c.pid for c in children]))
    manifest = {'supervisor_pid': os.getpid(), 'started': time.time(), 'state': 'starting',
                'camera_baseline': a.camera_baseline,
                'runtime_preflight': runtime_preflight,
                'isolation':isolation.environment(),'locks':isolation.lock_paths(),
                'performance_lease': {'path':str(PERFORMANCE_LEASE_PATH),
                                      'exclusive':a.exclusive_performance},
                'children': [], 'logging_backend': 'spdlog', 'log_layout': 'unified',
                'unified_log': str(output_path), 'log_files': {}}
    def save():
        (run / 'session.json').write_text(json.dumps(manifest, indent=2))
        session_log.write(json.dumps(manifest, ensure_ascii=False))
    def spawn(name, cmd, env):
        child_env = dict(env, ASTRIBOT_LOG_DIR=str(run / name), ROS_LOG_DIR=str(run / name),
                         ASTRIBOT_LOG_CAPTURE='1')
        # Install the console-only launch handler before ros2cli opens launch.log.
        if cmd[:2] == ['ros2', 'launch']:
            cmd = [sys.executable, '-m', 'astribot_logging.launch_entry', *cmd[1:]]
        child = subprocess.Popen(cmd, env=child_env, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        children.append(child)
        logs.append(ProcessOutput(child.stdout, output_path, source=f'{name} pid={child.pid}'))
        manifest['log_files'][name] = {'output': str(output_path), 'source': name}
        manifest['children'].append({'name': name, 'pid': child.pid, 'command': cmd,
                                     'output_log': str(output_path)})
        save()
        return child

    def check_logs():
        for log in logs:
            log.check()
    def stopped(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stopped)
    signal.signal(signal.SIGINT, stopped)
    try:
        env = control_environment(os.environ, a.nav_transport, run/'nav_udp.xml')
        manifest['control_transport'] = {key: env.get(key) for key in (
            'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY', 'FASTRTPS_DEFAULT_PROFILES_FILE')}
        spawn('simulation', sim, env)
        end = time.monotonic() + a.ready_timeout
        while time.monotonic() < end:
            check_logs()
            remember_children()
            result = subprocess.run(['timeout', '4', 'ign', 'topic', '-e', '-t', '/stats', '-n', '1'], capture_output=True, text=True)
            if 'iterations:' in result.stdout and 'iterations: 0' not in result.stdout:
                SessionLog(output_path, 'gazebo_stats').write(result.stdout)
                break
            if children[0].poll() is not None:
                raise RuntimeError('simulation launch exited')
        else:
            raise RuntimeError('Gazebo physics did not advance; inspect session.log [simulation]')
        if a.real_time_factor != 1.0:
            physics = subprocess.run(['ign', 'service', '-s', '/world/default/set_physics',
                '--reqtype', 'ignition.msgs.Physics', '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '3000', '--req', f'real_time_factor: {a.real_time_factor} max_step_size: 0.001'],
                capture_output=True, text=True, timeout=5.)
            if physics.returncode or 'data: true' not in physics.stdout:
                raise RuntimeError('Gazebo rejected the requested real-time factor: ' + physics.stderr)
            manifest['real_time_factor'] = a.real_time_factor
            save()
        def probe(phase, env):
            cmd = ['python3', str(repo/'tools/sim_stack_probe.py'), '--phase', phase,
                   '--timeout', str(a.ready_timeout), '--scan',
                   '/scan_from_cloud' if a.scan_source=='slice_scan' else '/scan']
            if phase=='navigation' and a.navigation_policy!='off':
                cmd+=['--costmap-scan','/navigation_policy/costmap_scan','--require-policy']
            log = SessionLog(output_path, f'probe_{phase}')
            process = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, text=True, start_new_session=True)
            children.append(process)
            try:
                probe_end=time.monotonic()+a.ready_timeout+5
                while True:
                    check_logs()
                    remember_children()
                    remaining=probe_end-time.monotonic()
                    if remaining<=0:raise subprocess.TimeoutExpired(cmd,a.ready_timeout+5)
                    try:
                        stdout,stderr=process.communicate(timeout=min(.5,remaining))
                        break
                    except subprocess.TimeoutExpired:
                        if time.monotonic()>=probe_end:raise
                log.write(stderr.rstrip('\n'))
                log.write(stdout.rstrip('\n'))
                result = json.loads(stdout.strip().splitlines()[-1]) if stdout.strip() else {'ready':False}
                return result
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                stdout,stderr=process.communicate()
                log.write(stderr.rstrip('\n'))
                log.write(stdout.rstrip('\n'))
                return {'ready':False, 'error':'ROS probe exceeded external wall-clock watchdog'}
            finally:
                if process.poll() is not None:
                    children.remove(process)
        measured = probe('data', env)
        if not measured.get('ready'):
            raise RuntimeError(f'clock/TF/scan/odom not ready: {measured}')
        manifest['pre_navigation'] = measured
        # The same local-only transport is used for pre-navigation probes and
        # control nodes; camera bridges retain their dedicated transport.
        # Queries use the exact environment given to this owned navigation process.
        (run / 'env.sh').write_text('source /opt/ros/humble/setup.bash\nsource ' + shlex.quote(str(repo/'ws_robot/install/setup.bash')) + '\n' + ''.join('export '+k+'='+shlex.quote(v)+'\n' for k,v in env.items() if k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE','IGN_IP','GZ_IP','IGN_PARTITION','GZ_PARTITION','IGN_DISCOVERY_MSG_PORT','IGN_DISCOVERY_SRV_PORT','ASTRIBOT_SIM_INSTANCE')))
        if mapping_session:
            spawn('mapping_session', mapping_session, env)
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
            logger.warning(f'Navigation readiness failed: {measured}; restarting only owned navigation group {child.pid}')
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL); child.wait()
            children.remove(child)
            # Let discovery dispose old endpoints before creating replacements.
            time.sleep(2)
        manifest.update(state='ready', lifecycle=states, ready_at=time.time()); save()
        logger.info(f'READY {run} lifecycle={states}')
        while all(c.poll() is None for c in children):
            check_logs()
            remember_children()
            time.sleep(1)
        raise RuntimeError('a launch exited after readiness')
    except KeyboardInterrupt:
        manifest['state'] = 'stopped'
    except Exception as exc:
        manifest.update(state='failed', error=str(exc))
        logger.exception('Session failed')
        raise
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
        capture_errors = []
        for log in logs:
            try:
                log.close()
            except Exception as exc:
                capture_errors.append(str(exc))
        manifest['log_capture_errors'] = capture_errors
        if capture_errors:
            manifest.update(state='logging_failed', error='; '.join(capture_errors))
        manifest['ended'] = time.time(); save()
        if capture_errors:
            raise RuntimeError(manifest['error'])

if __name__ == '__main__':
    main()
