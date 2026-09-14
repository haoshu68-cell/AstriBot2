"""Integration tests for the installed launch/subprocess spdlog capture path."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


class OutputTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def run_code(self, code, **extra):
        env = dict(os.environ, ASTRIBOT_LOG_DIR=str(self.root), ROS_LOG_DIR=str(self.root),
                   ASTRIBOT_LOG_MAX_BYTES='1048576', ASTRIBOT_LOG_BACKUP_COUNT='2')
        env.update(extra)
        result = subprocess.run([sys.executable, '-c', code], env=env, text=True,
                                capture_output=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        return result

    def test_child_stdout_stderr_and_partial_line_persist_after_exit(self):
        self.run_code('''
import subprocess, sys, os
from astribot_logging.output import ProcessOutput
child = subprocess.Popen([sys.executable, '-c',
    "import sys; print('SIM_OUT'); print('NAV_ERR', file=sys.stderr); sys.stdout.write('partial中文')"],
    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
capture = ProcessOutput(child.stdout, os.environ['ROS_LOG_DIR']+'/simulation.log')
assert child.wait(timeout=10) == 0
capture.close()
''')
        content = (self.root / 'simulation.log').read_text()
        for marker in ('SIM_OUT', 'NAV_ERR', 'partial中文'):
            self.assertEqual(content.count(marker), 1)

    def test_file_rotation_and_removed_file_recovery(self):
        self.run_code('''
import os
from pathlib import Path
from astribot_logging.output import FileLog
path=Path(os.environ['ROS_LOG_DIR'])/'navigation.log'
sink=FileLog(path)
for i in range(100): sink.write('line-%d '%i + 'x'*100)
assert len(list(path.parent.glob('navigation*.log'))) == 3
path.unlink()
sink.write('RECOVERED')
''', ASTRIBOT_LOG_MAX_BYTES='1024')
        content = (self.root / 'navigation.log').read_text()
        self.assertIn('reopened', content)
        self.assertIn('RECOVERED', content)
        self.assertTrue(all(p.stat().st_size <= 1024 for p in self.root.glob('navigation*.log')))

    def test_launch_captures_ros_and_non_ros_output_even_when_screen_requested(self):
        self.run_code('''
import sys
from launch import LaunchService, LaunchDescription
from launch.actions import ExecuteProcess
from launch.logging import get_logger
# Exercise replacement of a handler already created by ros2 launch before import.
get_logger('early').info('EARLY_RECORD')
from astribot_logging.launch import Node
code = "import rclpy; rclpy.init(); rclpy.logging.get_logger('nav_test').error('ROS_NODE_MARKER'); rclpy.shutdown()"
ld = LaunchDescription([
    Node(executable=sys.executable, arguments=['-c', code], output='screen'),
    ExecuteProcess(cmd=[sys.executable, '-c',
        "import sys; print('GAZEBO_STDOUT'); print('GAZEBO_STDERR', file=sys.stderr)"], output='screen')])
service=LaunchService()
service.include_launch_description(ld)
assert service.run() == 0
''')
        launches = list(self.root.glob('*/launch.log'))
        self.assertEqual(len(launches), 1)
        content = launches[0].read_text()
        for marker in ('EARLY_RECORD', 'ROS_NODE_MARKER', 'GAZEBO_STDOUT', 'GAZEBO_STDERR'):
            self.assertEqual(content.count(marker), 1, content)
        # Managed ROS nodes no longer leave a second native unbounded logfile.
        self.assertEqual(list(self.root.glob('python*_*.log')), [])

    def test_original_screen_only_output_was_not_persisted(self):
        result = self.run_code('''
import sys
from launch import LaunchService, LaunchDescription
from launch.actions import ExecuteProcess
service=LaunchService()
service.include_launch_description(LaunchDescription([
    ExecuteProcess(cmd=[sys.executable, '-c', "print('OLD_SCREEN_ONLY')"], output='screen')]))
assert service.run() == 0
''', OVERRIDE_LAUNCH_PROCESS_OUTPUT='screen')
        self.assertIn('OLD_SCREEN_ONLY', result.stdout)
        content = '\n'.join(p.read_text() for p in self.root.glob('*/launch.log'))
        self.assertNotIn('OLD_SCREEN_ONLY', content)

    def test_write_failure_is_reported(self):
        self.run_code('''
import os
from pathlib import Path
from astribot_logging.output import FileLog
p=Path(os.environ['ROS_LOG_DIR'])/'bad.log'
p.mkdir()
try: FileLog(p)
except RuntimeError: pass
else: raise AssertionError('sink initialization error was swallowed')
''')

    def test_capture_write_failure_does_not_block_child_pipe(self):
        self.run_code('''
import os, subprocess, sys
from pathlib import Path
from astribot_logging.output import ProcessOutput
path=Path(os.environ['ROS_LOG_DIR'])/'capture.log'
child=subprocess.Popen([sys.executable, '-c',
    "import sys; sys.stdin.readline(); sys.stdout.write('x'*1000000); sys.stdout.flush()"],
    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
capture=ProcessOutput(child.stdout,path)
path.unlink()
path.mkdir()  # Force a write error after the sink has been opened.
child.stdin.write(b'go\\n'); child.stdin.close()
assert child.wait(timeout=10) == 0
try: capture.close()
except RuntimeError as exc: assert 'Failed to capture' in str(exc)
else: raise AssertionError('capture failure was swallowed')
''')

    def test_unified_launch_sdk_ros_and_parallel_sources_without_duplicates(self):
        self.run_code('''
import os, subprocess, sys
from pathlib import Path
from astribot_logging.output import ProcessOutput, SessionLog, SessionHandler
import logging
root=Path(os.environ['ROS_LOG_DIR'])
path=root/'session.log'
logger=logging.Logger('supervisor')
logger.addHandler(SessionHandler(path))
logger.warning('SUPERVISOR_MARKER')
code="""
import sys
from astribot_logging import get_logger
from astribot_logging.output import configure_launch_logging
configure_launch_logging()
get_logger('sdk').info('SDK_MARKER')
from launch import LaunchService, LaunchDescription
from launch.actions import ExecuteProcess
from astribot_logging.launch import Node
ros_code="import rclpy; rclpy.init(); rclpy.logging.get_logger('test').error('ROS_MARKER'); rclpy.shutdown()"
ld=LaunchDescription([
    Node(executable=sys.executable, arguments=['-c',ros_code],output='screen'),
    ExecuteProcess(cmd=[sys.executable,'-c',"print('GAZEBO_MARKER')"],output='log')])
service=LaunchService(); service.include_launch_description(ld)
assert service.run()==0
"""
children=[]
for source in ['simulation','navigation_1']:
    child=subprocess.Popen([sys.executable,'-c',code],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    children.append((child,ProcessOutput(child.stdout,path,source=source)))
for child,output in children:
    assert child.wait(timeout=15)==0
    output.close()
content=path.read_text()
assert content.count('SUPERVISOR_MARKER')==1,content
for marker in ['SDK_MARKER','ROS_MARKER','GAZEBO_MARKER']:
    assert content.count(marker)==2,content
    for source in ['simulation','navigation_1']:
        assert sum(marker in line and '['+source+']' in line for line in content.splitlines())==1
assert list(root.rglob('*.log'))==[path],list(root.rglob('*.log'))
''', ASTRIBOT_LOG_CAPTURE='1')

    def test_console_only_cli_entry_opens_no_duplicate_launch_file(self):
        self.run_code(r'''
import os, subprocess, sys
from pathlib import Path
from astribot_logging.output import ProcessOutput
root=Path(os.environ['ROS_LOG_DIR']);path=root/'session.log'
launch_file=root/'capture.launch.py'
launch_file.write_text("from launch import LaunchDescription\nfrom launch.actions import ExecuteProcess\n"
    "def generate_launch_description():\n"
    "    return LaunchDescription([ExecuteProcess(cmd=['/bin/echo','CLI_MARKER'],output='log')])\n")
child=subprocess.Popen([sys.executable,'-m','astribot_logging.launch_entry','launch',str(launch_file)],
    stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
output=ProcessOutput(child.stdout,path,source='simulation')
assert child.wait(timeout=15)==0
output.close()
assert path.read_text().count('CLI_MARKER')==1,path.read_text()
assert list(root.rglob('*.log'))==[path],list(root.rglob('*.log'))
''', ASTRIBOT_LOG_CAPTURE='1')

    def test_unified_rotation_is_owned_by_one_writer(self):
        self.run_code('''
import os, subprocess, sys
from pathlib import Path
from astribot_logging.output import ProcessOutput
root=Path(os.environ['ROS_LOG_DIR']);path=root/'session.log'
children=[]
for source in ['simulation','navigation_1']:
    child=subprocess.Popen([sys.executable,'-c',"for i in range(12): print('line-%d '%i+'x'*80)"],
        stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    children.append((child,ProcessOutput(child.stdout,path,source=source)))
for child,output in children:
    assert child.wait(timeout=10)==0
    output.close()
files=list(root.glob('session*.log'))
assert len(files)>1
content=''.join(p.read_text() for p in files)
assert content.count('[simulation]')==12
assert content.count('[navigation_1]')==12
assert len(list(root.rglob('*.log')))==len(files)
''', ASTRIBOT_LOG_CAPTURE='1', ASTRIBOT_LOG_MAX_BYTES='1024', ASTRIBOT_LOG_BACKUP_COUNT='10')


if __name__ == '__main__':
    unittest.main()
