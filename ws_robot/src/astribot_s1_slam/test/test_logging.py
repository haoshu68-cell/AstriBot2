"""Real C++ ROS diagnostics captured by the repository's spdlog session sink."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from astribot_logging.output import ProcessOutput


class SlamLoggingTests(unittest.TestCase):
    def capture(self, level, named=None):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            env = dict(os.environ, ROS_DOMAIN_ID='168', ROS_LOCALHOST_ONLY='1',
                       ROS_LOG_DIR=directory, ASTRIBOT_LOG_DIR=directory,
                       ASTRIBOT_LOG_CAPTURE='1', RCUTILS_COLORIZED_OUTPUT='0')
            command = [os.environ['SLAM_LOG_PROBE'], '--ros-args',
                       '--disable-external-lib-logs', '--log-level', level,
                       '-r', '__node:=renamed_slam']
            if named:
                command += ['--log-level', 'renamed_slam:=' + named]
            child = subprocess.Popen(command, env=env, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT)
            capture = ProcessOutput(child.stdout, root / 'session.log', source='slam_probe')
            try:
                self.assertEqual(child.wait(timeout=15), 0)
            finally:
                if child.poll() is None:
                    child.kill()
                    child.wait()
                capture.close()
            self.assertEqual([p.name for p in root.rglob('*.log')], ['session.log'])
            return (root / 'session.log').read_text()

    def test_info_is_captured_once_with_tags_and_tail(self):
        text = self.capture('info')
        for marker in ('INFO_MARK', 'STARTUP_MARK', 'WARN_MARK', 'ERROR_MARK',
                       'DECISION_MARK', 'THREAD_MARK', 'TAIL_MARK'):
            self.assertEqual(text.count(marker), 1, text)
        self.assertEqual(text.count('THROTTLE_MARK'), 3)
        self.assertIn('[renamed_slam]', text)
        self.assertIn('[INIT] INFO_MARK 中文 42', text)
        self.assertIn('debug_evaluated=0', text)
        self.assertNotIn('DEBUG_MARK', text)
        self.assertNotIn('\x1b', text)

    def test_release_debug_and_named_level_override(self):
        text = self.capture('error', named='debug')
        self.assertEqual(text.count('DEBUG_MARK'), 1)
        self.assertIn('debug_evaluated=1', text)
        self.assertIn('STARTUP_MARK', text)

    def test_error_filters_startup_and_decisions_too(self):
        text = self.capture('error')
        self.assertEqual(text.count('ERROR_MARK'), 1)
        for marker in ('INFO_MARK', 'STARTUP_MARK', 'WARN_MARK', 'DECISION_MARK', 'DEBUG_MARK'):
            self.assertNotIn(marker, text)


if __name__ == '__main__':
    unittest.main()
