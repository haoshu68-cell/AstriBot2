#!/usr/bin/env python3
"""Regression: a failed estimator cannot reuse a previous successful result."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'tools/vision/sim_pose_evaluate.py'


class EvaluationProcessContractTest(unittest.TestCase):
    def invoke(self, exit_code, new_success=None):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        worker = root / 'bundle/lib/astribot_object_pose_core/object_pose_register'
        worker.parent.mkdir(parents=True)
        (worker.parent.parent / 'libastribot_object_pose_registration.so').write_bytes(b'test-fixture')
        out = root / 'results'
        out.mkdir()
        pose = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
        captures = root / 'captures'
        (captures / 'clear').mkdir(parents=True)
        (captures / 'clear/scene.xyz').write_text('0 0 1 0 0 -1\n')
        (captures / 'clear/truth.json').write_text(json.dumps({'camera_from_object': pose}))
        old = {'success': True, 'camera_from_object': pose, 'reason': 'previous_run'}
        (out / 'clear.json').write_text(json.dumps(old))
        (out / 'summary.json').write_text(json.dumps({'accepted_count': 1, 'previous_run': True}))
        code = '#!/usr/bin/env python3\nimport json,sys,os\n'
        code += 'assert os.environ["LD_LIBRARY_PATH"].split(os.pathsep)[0] == ' + repr(str(root / 'bundle/lib')) + '\n'
        if new_success is not None:
            payload = {'success': new_success, 'camera_from_object': pose if new_success else None, 'reason': 'fixture'}
            code += 'open(sys.argv[sys.argv.index("--output")+1],"w").write(' + repr(json.dumps(payload)) + ')\n'
        code += f'sys.exit({exit_code})\n'
        worker.write_text(code)
        worker.chmod(0o755)
        # Exercise the public dataset selection without replacing globals.
        result = subprocess.run([sys.executable, str(SCRIPT), '--dataset-root', str(captures), 'clear', '--bundle', str(root / 'bundle'),
                                 '--results-dir', str(out)], capture_output=True, text=True)
        return result, out

    def test_old_success_json_cannot_survive_failed_cli(self):
        result, out = self.invoke(64)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((out / 'clear.json').exists())
        self.assertFalse((out / 'summary.json').exists())

    def test_exit_two_cannot_claim_success(self):
        result, out = self.invoke(2, True)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((out / 'summary.json').exists())

    def test_exit_zero_cannot_claim_rejection(self):
        result, out = self.invoke(0, False)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((out / 'summary.json').exists())

    def test_legitimate_rejection_is_scored_as_failure(self):
        result, out = self.invoke(2, False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads((out / 'summary.json').read_text())['accepted_count'], 0)

    def test_legitimate_success_is_scored(self):
        result, out = self.invoke(0, True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads((out / 'summary.json').read_text())['accepted_count'], 1)


if __name__ == '__main__':
    unittest.main()
