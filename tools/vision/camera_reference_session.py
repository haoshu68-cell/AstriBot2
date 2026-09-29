#!/usr/bin/env python3
"""Own a camera-placement probe in the canonical navigation warehouse.

Launch/validation only. No hardware, navigation goals or arm trajectories.
"""
import fcntl
import argparse
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.sim_isolation import SimulationIsolation, is_stack_process
from tools.vision.owned_process_cleanup import cleanup


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-root', type=Path, default=ROOT / 'runs/camera_reference_20260921')
    parser.add_argument('--instance', default='camera_reference_20260921')
    parser.add_argument('--domain', type=int, default=88)
    parser.add_argument('--overlay', type=Path, action='append', default=[])
    parser.add_argument('--control-dds-profile', type=Path,
                        help='explicit local-only control transport; camera bridge keeps its own profile')
    parser.add_argument('--manifest', type=Path,
                        help='optional frozen installed-artifact gate, checked before Gazebo starts')
    parser.add_argument('--wrist-rate', type=float, choices=[5.0, 10.0], default=10.0,
                        help='10 Hz leaves delivery margin under the unchanged 250 ms health deadline')
    args = parser.parse_args()
    run = args.run_root.resolve()
    session = run / 'session'
    session.mkdir(parents=True, exist_ok=True)
    if (session / 'session.json').exists():
        raise RuntimeError('Session evidence already exists; use a new --run-root')
    iso = SimulationIsolation(args.instance, args.domain)
    locks = []
    for path in iso.lock_paths():
        handle = open(path, 'a+')
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        locks.append(handle)
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            argv = proc.joinpath('cmdline').read_bytes().decode().rstrip('\0').split('\0')
            # Ignition rewrites argv into one process-title string; preserve
            # ownership detection after the original shell/launch has exited.
            if not is_stack_process(argv) and not (argv and argv[0].startswith('ign gazebo ')):
                continue
            env = dict(x.split('=', 1) for x in proc.joinpath('environ').read_bytes().decode().split('\0') if '=' in x)
            if conflict := iso.conflict(env):
                raise RuntimeError(f'{conflict}: owned by existing pid {proc.name}')
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    overlays = [Path('/opt/ros/humble/setup.bash'), ROOT/'ws_robot/install/local_setup.bash',
                ROOT/'install/local_setup.bash',
                ROOT/'runs/grasp_pose_sim_20260921/ros_ws/install/local_setup.bash',
                ROOT/'runs/camera_reference_20260921/ros_ws/install/local_setup.bash',
                *[path.resolve() for path in args.overlay]]
    for path in overlays:
        if not path.is_file():
            raise RuntimeError(f'Missing built overlay: {path}')
    # Retain source calibration separately. Only auxiliary cameras are scaled
    # for this six-camera rendering probe; fixed mounting geometry is unchanged.
    profile_dir = run/'render_profiles'
    profile_dir.mkdir(exist_ok=True)
    import yaml
    source = ROOT/'ws_robot/src/astribot_s1_description/config'
    for camera in ('head_rgbd', 'torso_rgbd', 'left_wrist_rgbd', 'right_wrist_rgbd',
                   'head_stereo_left', 'head_stereo_right'):
        original = yaml.safe_load((source/f'camera_{camera}.yaml').read_text())
        if camera.startswith('head_stereo'):
            width, height, hz = 400, 300, 5
        elif 'wrist' in camera:
            width, height, hz = 640, 320, args.wrist_rate
        else:
            original = yaml.safe_load((source/f'camera_{camera}_nav_sim.yaml').read_text())
            width, height, hz = original['width'], original['height'], original['rate_hz']
        sx, sy = width/original['width'], height/original['height']
        for matrix in original['intrinsics'].values():
            for index in (0, 1, 2):
                matrix[index] *= sx
            for index in (3, 4, 5):
                matrix[index] *= sy
        original.update(width=width, height=height, rate_hz=hz,
                        calibration_status='simulation_scaled_intrinsics_reference_mount_separate')
        (profile_dir/f'camera_{camera}.yaml').write_text(yaml.safe_dump(original, sort_keys=False))
    command = ['ros2', 'launch', 'astribot_s1_gazebo_bringup', 'warehouse_sim.launch.py',
               f'ros_domain_id:={args.domain}', 'headless:=true', 'use_rviz:=false', 'use_lidar:=false',
               'enable_effort_drive:=false', 'use_camera:=true', 'use_wrist_cameras:=true',
               'use_stereo_cameras:=true', 'use_camera_postprocess:=false', 'use_camera_pointcloud:=true',
               f'camera_calibration_dir:={profile_dir}',
               f'camera_profile:={profile_dir}/camera_head_rgbd.yaml',
               f'torso_camera_profile:={profile_dir}/camera_torso_rgbd.yaml']
    setup = ('source '+shlex.quote(str(ROOT/'tools/ros_overlay_env.sh')) +
             '\nastribot_prepare_overlay_environment\n' +
             '\n'.join('source '+shlex.quote(str(path)) for path in overlays))
    if args.manifest:
        check = ['python3', str(ROOT/'tools/vision/verify_task_environment.py'),
                 '--manifest', str(args.manifest.resolve()), '--runtime-only']
        verified = subprocess.run(['bash', '-c', setup+'\n'+shlex.join(check)],
                                  cwd=ROOT, capture_output=True, text=True)
        (session/'environment_check.log').write_text(verified.stdout+verified.stderr)
        if verified.returncode:
            raise RuntimeError('Frozen runtime environment rejected; see environment_check.log')
    env = os.environ.copy()
    session_env = iso.environment()
    if args.control_dds_profile:
        profile = args.control_dds_profile.resolve()
        if not profile.is_file():
            raise RuntimeError('Missing control DDS profile')
        session_env.update(FASTRTPS_DEFAULT_PROFILES_FILE=str(profile),
                           ROS_LOCALHOST_ONLY='0', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
        command += ['localhost_only:=false', 'camera_bridge_dds_profile:='+str(
            ROOT/'ws_robot/src/astribot_s1_gazebo_bringup/config/camera_bridge_fastdds.xml')]
    env.update(session_env)
    env.update(ASTRIBOT_LOG_DIR=str(session), ROS_LOG_DIR=str(session/'ros'), ASTRIBOT_LOG_CAPTURE='1')
    (session/'query_env.sh').write_text(setup+'\n'+''.join('export '+key+'='+shlex.quote(value)+'\n' for key,value in session_env.items()))
    with (session/'session.log').open('w') as log:
        child = subprocess.Popen(['bash', '-c', setup+'\nexec '+shlex.join(command)],
                                 cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        record = {'supervisor_pid': os.getpid(), 'launch_pid': child.pid,
                  'launch_start_ticks': Path(f'/proc/{child.pid}/stat').read_text().split(') ',1)[1].split()[19],
                  'started_wall': time.time(), 'env': session_env, 'command': command,
                  'session_log': str(session/'session.log'),
                  'scope': 'canonical warehouse camera geometry and sensor validation only'}
        target = session/'session.json'
        target.write_text(json.dumps(record, indent=2)+'\n')
        def stop(_signal, _frame):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        result = child.wait()
        cleanup_result = cleanup(child.pid, session, args.instance)
        (session/'owned_cleanup.json').write_text(json.dumps(cleanup_result, indent=2))
        record.update(exit_code=result, stopped_wall=time.time())
        target.write_text(json.dumps(record, indent=2)+'\n')
    return result


if __name__ == '__main__':
    sys.exit(main())
