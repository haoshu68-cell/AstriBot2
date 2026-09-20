from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from sim_isolation import SimulationIsolation,is_stack_process


class IsolationTests(unittest.TestCase):
    def test_bare_simulator_conflict_is_not_missed_or_matched_in_a_shell(self):
        self.assertTrue(is_stack_process(['/usr/bin/ruby','/usr/bin/ign','gazebo','-s']))
        self.assertTrue(is_stack_process(['/usr/bin/ign gazebo server']))
        self.assertFalse(is_stack_process(['/bin/bash','-c','ruby /usr/bin/ign gazebo -s']))
        self.assertFalse(is_stack_process(['/usr/bin/python3','probe.py']))
    def test_legacy_identity_preserved(self):
        value=SimulationIsolation()
        self.assertEqual(value.environment(),{'ROS_DOMAIN_ID':'25','ROS_LOCALHOST_ONLY':'1'})
        self.assertEqual(value.lock_paths(),['/tmp/astribot_sim_domain25.lock'])

    def test_ros_only_override_cannot_claim_isolation(self):
        for instance,domain in [('',74),('test',25),('test',0),('test',213),('../bad',74),('x y',74)]:
            with self.subTest(instance=instance,domain=domain),self.assertRaises(ValueError):
                SimulationIsolation(instance,domain)

    def test_partitions_and_discovery_ports_are_explicit(self):
        value=SimulationIsolation('nonhome_test',74);env=value.environment()
        self.assertEqual(env['IGN_PARTITION'],env['GZ_PARTITION'])
        self.assertNotEqual(env['IGN_DISCOVERY_MSG_PORT'],env['IGN_DISCOVERY_SRV_PORT'])
        self.assertNotEqual(env['IGN_DISCOVERY_MSG_PORT'],'10317')
        self.assertEqual(len(value.lock_paths()),2)
        self.assertEqual(env['IGN_IP'],'127.0.0.1')

    def test_any_shared_control_namespace_is_a_conflict(self):
        value=SimulationIsolation('nonhome_test',74);env=value.environment()
        for key in ['ROS_DOMAIN_ID','IGN_PARTITION','GZ_PARTITION','IGN_DISCOVERY_MSG_PORT','IGN_DISCOVERY_SRV_PORT']:
            with self.subTest(key=key):self.assertTrue(value.conflict({key:env[key]}))
        self.assertFalse(value.conflict(SimulationIsolation('another',73).environment()))


if __name__=='__main__':unittest.main()
