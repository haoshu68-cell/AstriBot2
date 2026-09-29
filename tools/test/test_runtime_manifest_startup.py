"""Operations preflight must fail before creating a simulation session."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class RuntimeManifestStartup(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.prefix = self.root/'prefix'
        marker = self.prefix/'share/ament_index/resource_index/packages/startup_test_package'
        marker.parent.mkdir(parents=True)
        marker.touch()
        self.artifact = self.prefix/'library.fixture'
        self.artifact.write_text('pinned C++ library fixture')
        self.manifest = self.root/'manifest.json'
        self.manifest.write_text(json.dumps({
            'prefixes': {'startup_test_package': str(self.prefix)},
            'files': {str(self.artifact): {
                'sha256': hashlib.sha256(self.artifact.read_bytes()).hexdigest(),
                'resolved': str(self.artifact)}}}))

    def run_supervisor(self):
        env = os.environ.copy()
        env['AMENT_PREFIX_PATH'] = str(self.prefix)+os.pathsep+env.get('AMENT_PREFIX_PATH', '')
        result = subprocess.run([
            sys.executable, str(ROOT/'tools/sim_stack_supervisor.py'),
            '--dry-run', '--runtime-manifest', str(self.manifest),
            '--instance', 'runtime_preflight_test', '--ros-domain-id', '94',
            '--log-dir', str(self.root/'session')], env=env,
            capture_output=True, text=True, timeout=15)
        self.assertFalse((self.root/'session').exists())
        return result

    def test_valid_snapshot_reaches_dry_run_and_retains_receipt(self):
        result = self.run_supervisor()
        self.assertEqual(result.returncode, 0, result.stderr)
        receipt = json.loads(result.stdout)['runtime_preflight']
        self.assertTrue(receipt['passed'])
        self.assertEqual(receipt['sha256'], hashlib.sha256(self.manifest.read_bytes()).hexdigest())

    def test_changed_library_rejected_before_session_creation(self):
        self.artifact.write_text('unbounded old library')
        result = self.run_supervisor()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('RUNTIME_MANIFEST_REJECTED', result.stderr)
        self.assertIn('ARTIFACT_CHANGED', result.stderr)

    def test_missing_manifest_rejected_before_session_creation(self):
        self.manifest.unlink()
        result = self.run_supervisor()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('RUNTIME_MANIFEST_REJECTED', result.stderr)


if __name__ == '__main__':
    unittest.main()
