import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
from sim_performance_lease import acquire_performance_lease, simulation_servers


class PerformanceLease(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / 'performance.lock'

    def tearDown(self):
        self.directory.cleanup()

    def test_normal_sessions_can_share_host(self):
        with acquire_performance_lease(False, self.path), acquire_performance_lease(False, self.path):
            pass

    def test_exclusive_session_blocks_all_new_sessions(self):
        with acquire_performance_lease(True, self.path):
            for exclusive in (False, True):
                with self.assertRaisesRegex(RuntimeError, 'PERFORMANCE_LEASE_BUSY'):
                    acquire_performance_lease(exclusive, self.path)

    def test_existing_normal_session_blocks_exclusive_start(self):
        with acquire_performance_lease(False, self.path):
            with self.assertRaisesRegex(RuntimeError, 'PERFORMANCE_LEASE_BUSY'):
                acquire_performance_lease(True, self.path)

    def test_release_allows_next_owner_and_does_not_unlink_lock(self):
        with acquire_performance_lease(True, self.path):
            inode = self.path.stat().st_ino
        with acquire_performance_lease(True, self.path):
            self.assertEqual(self.path.stat().st_ino, inode)

    def test_recognizes_direct_absolute_ruby_and_process_title_launches(self):
        fixtures = [
            ('ruby3.0', ['ruby', '/usr/bin/ign', 'gazebo', '-s']),
            ('ruby3.0', ['/usr/bin/ign', 'gazebo', '-s']),
            ('ruby3.0', ['ign gazebo -r -s warehouse.sdf']),
            ('gz', ['/usr/bin/gz', 'sim', '-s']),
            ('gzserver', ['/usr/bin/gzserver', 'warehouse.world']),
        ]
        root = Path(self.directory.name)
        for index, (executable, arguments) in enumerate(fixtures, 100):
            process = root / str(index)
            process.mkdir()
            (process / 'exe').symlink_to('/usr/bin/' + executable)
            (process / 'cmdline').write_bytes(b'\0'.join(x.encode() for x in arguments) + b'\0')
        with patch('sim_performance_lease.Path', return_value=root):
            self.assertEqual({r['pid'] for r in simulation_servers()}, set(range(100, 105)))

    def test_incidental_shell_and_observer_text_is_not_a_simulator(self):
        fixtures = [
            ('bash', ['bash', '-c', 'ruby /usr/bin/ign gazebo -s']),
            ('python3', ['python3', 'observe.py', '--pattern', 'ign gazebo']),
            ('ruby3.0', ['ruby', 'other.rb', 'ign', 'gazebo']),
            ('gz', ['/usr/bin/gz', 'topic', '-l']),
        ]
        root = Path(self.directory.name)
        for index, (executable, arguments) in enumerate(fixtures, 100):
            process = root / str(index)
            process.mkdir()
            (process / 'exe').symlink_to('/usr/bin/' + executable)
            (process / 'cmdline').write_bytes(b'\0'.join(x.encode() for x in arguments) + b'\0')
        with patch('sim_performance_lease.Path', return_value=root):
            self.assertEqual(simulation_servers(), [])


if __name__ == '__main__':
    unittest.main()
