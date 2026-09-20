"""Explicit overlays replace inherited ROS paths without dropping external tools."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import unittest


class OverlayEnvironmentTest(unittest.TestCase):
    def test_known_prefixes_are_replaced_and_unrelated_paths_survive(self):
        helper = Path(__file__).resolve().parents[1] / 'ros_overlay_env.sh'
        environment = dict(os.environ, AMENT_PREFIX_PATH='/tmp/old/install/pkg:/opt/ros/humble',
            COLCON_PREFIX_PATH='/tmp/old/install',
            PYTHONPATH='/tmp/old/install/pkg/lib/python3.10/site-packages:/tmp/old/install-extra:/tmp/user modules',
            LD_LIBRARY_PATH='/tmp/sdk/lib:/tmp/old/install/pkg/lib:/usr/lib',
            CMAKE_PREFIX_PATH='/opt/gtsam:/tmp/old/install/pkg',
            IGN_GUI_PLUGIN_PATH='/tmp/old/install/pkg/lib:/tmp/external-gui',
            PATH='/tmp/old/install/pkg/bin:/usr/bin:/bin',
            ROS_DOMAIN_ID='68', IGN_PARTITION='test_partition')
        script = ('source ' + shlex.quote(str(helper)) + '\nastribot_prepare_overlay_environment\n'
                  '/usr/bin/python3 -c "import os,json; print(json.dumps(dict(os.environ)))"')
        run = subprocess.run(['bash', '--noprofile', '--norc', '-c', script], env=environment,
                             check=True, capture_output=True, text=True)
        actual = json.loads(run.stdout)
        self.assertNotIn('AMENT_PREFIX_PATH', actual)
        self.assertNotIn('COLCON_PREFIX_PATH', actual)
        self.assertEqual(actual['PYTHONPATH'], '/tmp/old/install-extra:/tmp/user modules')
        library_paths = actual['LD_LIBRARY_PATH'].split(':')
        self.assertTrue({'/tmp/sdk/lib', '/usr/lib'}.issubset(library_paths))
        self.assertFalse(any(p.startswith('/tmp/old/install/') for p in library_paths))
        self.assertEqual(actual['CMAKE_PREFIX_PATH'], '/opt/gtsam')
        self.assertEqual(actual['IGN_GUI_PLUGIN_PATH'], '/tmp/external-gui')
        self.assertEqual(actual['PATH'], '/usr/bin:/bin')
        self.assertEqual(actual['ROS_DOMAIN_ID'], '68')
        self.assertEqual(actual['IGN_PARTITION'], 'test_partition')


if __name__ == '__main__':
    unittest.main()
