"""Default supervised simulation entry with a single spdlog session log.

This launches the owner process, not a second copy of the navigation stack.
The repository tools directory must be present (override repo_dir if necessary).
"""
import os
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from astribot_logging import log_level
from astribot_logging.output import configure_launch_logging


def _repository():
    for parent in Path(__file__).resolve().parents:
        if (parent / 'tools/sim_stack_supervisor.py').is_file():
            return str(parent)
    return ''


def _owner_exited(event, context):
    if event.returncode and not context.is_shutdown:
        raise RuntimeError(f'Simulation supervisor failed with exit code {event.returncode}')
    return []


def _start(context):
    repository = Path(LaunchConfiguration('repo_dir').perform(context)).expanduser().resolve()
    supervisor = repository / 'tools/sim_stack_supervisor.py'
    if not supervisor.is_file():
        raise RuntimeError('Cannot find tools/sim_stack_supervisor.py; set repo_dir to the repository root')
    # The supervisor is the only file owner. The enclosing launch stays on console.
    os.environ['ASTRIBOT_LOG_CAPTURE'] = '1'
    configure_launch_logging()
    command = [sys.executable, '-u', str(supervisor)]
    for name in ('mode', 'map', 'map_yaml', 'navigation_policy', 'corridor_file', 'tracker',
                 'max_linear_speed', 'scan_source', 'nav_transport', 'nav_attempts',
                 'ready_timeout', 'log_dir', 'log_level', 'log_max_bytes', 'log_backup_count'):
        value = LaunchConfiguration(name).perform(context)
        if value:
            command += ['--' + name.replace('_', '-'), value]
    for name in ('headless', 'no_rviz', 'dry_run'):
        value = LaunchConfiguration(name).perform(context).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        if value == 'true':
            command.append('--' + name.replace('_', '-'))
    return [ExecuteProcess(
        cmd=command, name='sim_stack_supervisor', output='screen',
        sigterm_timeout='120', sigkill_timeout='15', on_exit=_owner_exited)]


def generate_launch_description():
    defaults = {
        'repo_dir': (_repository(), 'Repository root containing tools/; required for this workspace launcher'),
        'mode': ('mapping', 'mapping / explore / localize / baseline'),
        'map': ('', 'Serialized SLAM map base path'),
        'map_yaml': ('', 'Optional occupancy map YAML'),
        'navigation_policy': ('off', 'Existing policy stage: off / p2 / p3 / p4 / p5'),
        'corridor_file': ('', 'P4 map-frame corridor annotation JSON'),
        'tracker': ('mppi', 'mppi / rpp'),
        'max_linear_speed': ('0.35', 'Existing simulation speed limit; unchanged'),
        'scan_source': ('slice_scan', 'slice_scan / laserscan'),
        'headless': ('false', 'Disable Gazebo GUI'),
        'no_rviz': ('false', 'Disable RViz'),
        'nav_transport': ('udp', 'udp / default'),
        'nav_attempts': ('2', 'Maximum navigation startup attempts'),
        'ready_timeout': ('120', 'Readiness timeout in seconds'),
        'log_dir': ('', 'New session directory; default sim_<time>_<PID> under the shared log root'),
        'log_level': (log_level(), 'Default logging severity'),
        'log_max_bytes': (os.environ.get('ASTRIBOT_LOG_MAX_BYTES', '10485760'), 'session.log rotation size'),
        'log_backup_count': (os.environ.get('ASTRIBOT_LOG_BACKUP_COUNT', '5'), 'session.log rotation backups'),
        'dry_run': ('false', 'Print resolved startup and unified session.log configuration without starting the stack'),
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value, description=description)
          for name, (value, description) in defaults.items()),
        OpaqueFunction(function=_start),
    ])
