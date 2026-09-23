#!/usr/bin/env python3
"""Start owned GUI observers after a headless episode, capture, then close them."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from run_regression import descendants
from owned_command import OwnedCommand, run_owned_capture


def validate_environment(environment, isolation):
    for key in ('ROS_DOMAIN_ID', 'IGN_PARTITION', 'GZ_PARTITION'):
        if environment.get(key) != isolation.get(key):
            raise RuntimeError('GUI environment differs from owned episode: ' + key)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--supervisor', type=int, required=True)
    args = parser.parse_args()
    session = json.loads((args.output/'stack/session.json').read_text())
    if session.get('state') != 'ready' or session.get('supervisor_pid') != args.supervisor:
        raise RuntimeError('Matching live episode supervisor required')
    if not Path(f'/proc/{args.supervisor}').exists():
        raise RuntimeError('Episode supervisor already exited')
    validate_environment(os.environ, session['isolation'])
    from ament_index_python.packages import get_package_share_directory
    from Xlib import display, Xatom
    env = dict(os.environ, LIBGL_ALWAYS_SOFTWARE='1')
    for key in ('EGL_PLATFORM', '__GLX_VENDOR_LIBRARY_NAME', '__EGL_VENDOR_LIBRARY_FILENAMES'):
        env.pop(key, None)
    commands = [
        ['ign', 'gazebo', '-g', '--gui-config', str(args.output/'stack/simulation/social_gui.config')],
        ['ros2', 'run', 'rviz2', 'rviz2', '-d', str(Path(get_package_share_directory(
            'astribot_s1_navigation'))/'rviz/nav2_view.rviz'), '--ros-args', '-p',
         'use_sim_time:=true', '-r', '/scan:=/scan_from_cloud'],
    ]
    children = []
    connection = None
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupted)
    try:
        with (args.output/'deferred_gui.log').open('w') as log:
            for command in commands:
                children.append(OwnedCommand(command, env=env, stdout=log,
                    stderr=subprocess.STDOUT))
            connection = display.Display(env['DISPLAY'])
            until = time.monotonic() + 25
            windows = {}
            while time.monotonic() < until:
                for child in children:
                    child.observe()
                owned = descendants(os.getpid())
                prop = connection.screen().root.get_full_property(
                    connection.intern_atom('_NET_CLIENT_LIST'), Xatom.WINDOW)
                for ident in prop.value if prop else []:
                    window = connection.create_resource_object('window', int(ident))
                    pid = window.get_full_property(connection.intern_atom('_NET_WM_PID'), Xatom.CARDINAL)
                    if pid is None or int(pid.value[0]) not in owned:
                        continue
                    name = window.get_wm_name() or ''
                    label = 'gazebo' if name == 'Gazebo' else 'rviz' if name.endswith('RViz') else None
                    if label:
                        windows[label] = window
                if set(windows) == {'gazebo', 'rviz'}:
                    break
                if any(child.poll() is not None for child in children):
                    raise RuntimeError('Owned GUI exited before capture')
                time.sleep(.2)
            if set(windows) != {'gazebo', 'rviz'}:
                raise RuntimeError('Both owned GUI windows did not become available')
            for index, label in enumerate(('gazebo', 'rviz')):
                windows[label].configure(x=20+index*1260, y=30, width=1230, height=1320)
            connection.sync()
            time.sleep(2)
            result = run_owned_capture([sys.executable, str(Path(__file__).with_name('capture_views.py')),
                '--output', str(args.output), '--supervisor', str(os.getpid())],
                args.output/'deferred_capture.log', env=env, timeout=40)
            if result:
                raise RuntimeError('Deferred paired capture failed; see deferred_gui.log')
            (args.output/'visual_scope.json').write_text(json.dumps({
                'scope': 'Post-episode observations only; not continuous route visual coverage',
                'headless_physics': True, 'display': env['DISPLAY'],
                'gui_software_rendering': True, 'commands': commands}, indent=2)+'\n')
    finally:
        if connection is not None:
            connection.close()
        errors = []
        for child in reversed(children):
            try:
                child.stop()
            except Exception as error:
                errors.append(str(error))
        if errors:
            raise RuntimeError('; '.join(errors))


if __name__ == '__main__':
    main()
