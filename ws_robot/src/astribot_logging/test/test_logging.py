"""Behavior tests against the installed native library, without starting a robot."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


class LoggingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def run_python(self, code, **overrides):
        env = dict(os.environ, ASTRIBOT_LOG_DIR=str(self.root),
                   ASTRIBOT_LOG_LEVEL='info', ASTRIBOT_LOG_MAX_BYTES='1048576',
                   ASTRIBOT_LOG_BACKUP_COUNT='2')
        env.update(overrides)
        return subprocess.run([sys.executable, '-c', code], env=env,
                              capture_output=True, text=True, timeout=20)

    def test_filter_unicode_exception_and_single_delivery(self):
        result = self.run_python('''
from astribot_logging import get_logger
log = get_logger('sdk.test')
assert log is get_logger('sdk.test')
log.debug('HIDDEN')
log.info('中文 {} %s', 'VISIBLE')
try: raise ValueError('TRACEBACK_MARKER')
except ValueError: log.exception('caught')
''')
        self.assertEqual(result.returncode, 0, result.stderr)
        files = list(self.root.glob('*.log'))
        self.assertEqual(len(files), 1)
        content = files[0].read_text()
        for output in (content, result.stderr):
            self.assertEqual(output.count('VISIBLE'), 1)
            self.assertNotIn('HIDDEN', output)
            self.assertIn('中文 {}', output)
            self.assertIn('ValueError: TRACEBACK_MARKER', output)
        self.assertEqual(result.stdout, '')

    def test_rotation_is_bounded(self):
        result = self.run_python('''
from astribot_logging import get_logger
log=get_logger('rotation')
for i in range(200): log.info('%d %s', i, 'x'*100)
''', ASTRIBOT_LOG_MAX_BYTES='1024')
        self.assertEqual(result.returncode, 0, result.stderr)
        files = list(self.root.glob('*.log'))
        self.assertEqual(len(files), 3)
        self.assertTrue(all(p.stat().st_size <= 1024 for p in files))
        self.assertIn('199 ', '\n'.join(p.read_text() for p in files))

    def test_processes_get_distinct_files(self):
        result = self.run_python('''
import subprocess, sys
code = "from astribot_logging import get_logger; get_logger('worker').info('done')"
children = [subprocess.Popen([sys.executable, '-c', code]) for _ in range(2)]
assert all(child.wait(timeout=10) == 0 for child in children)
''')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(list(self.root.glob('*.log'))), 2)

    def test_threads_do_not_lose_or_duplicate_messages(self):
        result = self.run_python('''
from concurrent.futures import ThreadPoolExecutor
from astribot_logging import get_logger
def write(i): get_logger('thread.%d' % (i % 4)).info('MARKER-%d-END', i)
with ThreadPoolExecutor(max_workers=8) as pool:
    list(pool.map(write, range(200)))
''')
        self.assertEqual(result.returncode, 0, result.stderr)
        content = '\n'.join(p.read_text() for p in self.root.glob('*.log'))
        for i in range(200):
            self.assertEqual(content.count(f'MARKER-{i}-END'), 1)

    def test_bad_configuration_fails_explicitly(self):
        for override in ({'ASTRIBOT_LOG_LEVEL': 'typo'},
                         {'ASTRIBOT_LOG_MAX_BYTES': '0'},
                         {'ASTRIBOT_LOG_BACKUP_COUNT': '-1'}):
            result = self.run_python(
                "from astribot_logging import get_logger; get_logger().info('x')", **override)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('ValueError', result.stderr)
        blocked = self.root / 'file'
        blocked.write_text('not a directory')
        result = self.run_python(
            "from astribot_logging import get_logger; get_logger().info('x')",
            ASTRIBOT_LOG_DIR=str(blocked))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('RuntimeError', result.stderr)

    def test_vendor_adapter_is_idempotent(self):
        result = self.run_python('''
import logging
from astribot_logging import configure_stdlib_logger
log = logging.getLogger('vendor')
log.addHandler(logging.StreamHandler())
configure_stdlib_logger(log)
configure_stdlib_logger(log)
log.warning('VENDOR_MARKER')
''')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stderr.count('VENDOR_MARKER'), 1)

    def test_launch_level_precedence(self):
        result = self.run_python('''
from launch import LaunchContext
from launch.utilities import perform_substitutions
from astribot_logging.launch import Node, ComposableNodeContainer
context = LaunchContext()
node = Node(executable='/bin/true')
assert 'warn' in [perform_substitutions(context, part) for part in node.cmd]
context.launch_configurations['log_level'] = 'debug'
assert 'debug' in [perform_substitutions(context, part) for part in node.cmd]
node = Node(executable='/bin/true', arguments=['--ros-args', '--log-level', 'error'])
cmd = [perform_substitutions(context, part) for part in node.cmd]
assert cmd.count('--log-level') == 1 and 'error' in cmd
container = ComposableNodeContainer(name='test', namespace='', executable='/bin/true')
context.extend_locals({'ros_specific_arguments': {'name': 'test', 'namespace': '/'}})
assert 'debug' in [perform_substitutions(context, part) for part in container.cmd]
''', ASTRIBOT_LOG_LEVEL='warn')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_ros_backend_and_sdk_coexist(self):
        result = self.run_python('''
import rclpy
from rclpy.context import Context
from rclpy.logging import get_logger as ros_logger
from astribot_logging import get_logger
context = Context()
context.init(args=['--ros-args', '--log-level', 'warn'])
ros_logger('ros.test').info('ROS_HIDDEN')
ros_logger('ros.test').error('ROS_VISIBLE')
get_logger('sdk').info('SDK_BEFORE')
context.shutdown()
get_logger('sdk').info('SDK_AFTER')
''', ROS_LOG_DIR=str(self.root))
        self.assertEqual(result.returncode, 0, result.stderr)
        content = '\n'.join(p.read_text() for p in self.root.glob('*.log'))
        for marker in ('ROS_VISIBLE', 'SDK_BEFORE', 'SDK_AFTER'):
            self.assertEqual(content.count(marker), 1)
        self.assertNotIn('ROS_HIDDEN', content)


if __name__ == '__main__':
    unittest.main()
