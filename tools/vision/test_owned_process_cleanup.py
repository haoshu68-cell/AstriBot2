#!/usr/bin/env python3
import os, signal, subprocess, sys, tempfile, unittest
from pathlib import Path
from owned_process_cleanup import cleanup, members

class CleanupContract(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.session=Path(self.tmp.name).resolve();self.instance='cleanup_test'
        env=dict(os.environ,ASTRIBOT_LOG_DIR=str(self.session),ASTRIBOT_SIM_INSTANCE=self.instance)
        self.child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'],env=env,start_new_session=True)
        self.addCleanup(self.stop)
    def stop(self):
        if self.child.poll() is None:self.child.kill()
        self.child.wait(timeout=3)
    def test_verified_group_is_stopped(self):
        result=cleanup(self.child.pid,self.session,self.instance)
        self.assertEqual(result['remaining'],[]);self.assertTrue(result['signals']);self.child.wait(timeout=3)
    def test_other_instance_refused_and_preserved(self):
        with self.assertRaisesRegex(RuntimeError,'Unverified'):cleanup(self.child.pid,self.session,'other')
        self.assertIsNone(self.child.poll())
    def test_other_log_directory_refused_and_preserved(self):
        with self.assertRaisesRegex(RuntimeError,'Unverified'):cleanup(self.child.pid,self.session/'other',self.instance)
        self.assertIsNone(self.child.poll())

if __name__=='__main__':unittest.main()
