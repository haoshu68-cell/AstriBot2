"""Offline rejection checks; no ROS or simulator is started."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

from prepare_empty_inventory import camera_structure, empty_ready, verify_owner, CAMERAS, CaptureReceipts, ReadbackBarrier, proof_key
from capture_supervisor_owner import capture


class PrepareEmptyTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.proc = Path(self.tmp.name)
        (self.proc/'sys/kernel/random').mkdir(parents=True)
        (self.proc/'sys/kernel/random/boot_id').write_text('new_boot')
        process = self.proc/'123'
        process.mkdir()
        (process/'exe').symlink_to('/usr/bin/python3')
        (process/'stat').write_text('123 (supervisor) ' + ' '.join(['0'] * 19 + ['42']))
        argv = ['/usr/bin/python3', '/repo/tools/sim_stack_supervisor.py', '--instance', 'trial',
                '--ros-domain-id', '89', '--navigation-geometry-mode', 'fixed_v2',
                '--payload-source-id', 'physical', '--navigation-policy', 'p4']
        (process/'cmdline').write_bytes(('\0'.join(argv)+'\0').encode())
        self.owner = {'boot_id': 'new_boot', 'pid': 123, 'start_ticks': 42,
                      'exe': str((process/'exe').resolve()), 'cmdline': argv}
        self.env = {'ROS_DOMAIN_ID': '89', 'IGN_PARTITION': 'astribot_trial', 'ASTRIBOT_SIM_INSTANCE': 'trial'}
        stamp = {'sec': 1, 'nanosec': 0}
        until = {'sec': 1, 'nanosec': 300_000_000}
        observation = dict(environment='simulation', session_id='trial', source_id='physical', source_epoch='boot',
                           clock_epoch=1, revision=2, sequence=3, full_inventory=True, status=1, objects=[],
                           observed_at=stamp, valid_until=until)
        self.latest = dict(source=observation, ledger=dict(observation=copy.deepcopy(observation), confirmed=True,
            attachment_revision='revision', ledger_epoch='ledger', published_at=stamp, valid_until=until),
            geometry=dict(complete=True, attachment_state_confirmed=True, attachment_ids=[],
                          attachment_revision='revision', model_revision='model', header={'stamp': stamp, 'frame_id': 'base'}, valid_until=until),
            diagnostic=dict(policy='static_world_empty_only_v1', reason='EMPTY_INVENTORY_OBSERVED', source_epoch='boot', revision=2,
                            clock_epoch=1, stamp_ns=1_000_000_000))
        self.latest['geometry'].update(source_id='robot', clock_epoch=1)
        self.received = {k: 1. for k in self.latest}

    def valid(self, **kwargs):
        return empty_ready(self.latest, self.received, kwargs.get('wall', 1.1), kwargs.get('ros', 1_100_000_000), 'trial', 'physical')

    def test_owner_valid(self):
        verify_owner(self.owner, 'trial', 'physical', self.env, self.proc)

    def test_explicit_pid_capture_matches_guard(self):
        owner = capture(123, self.proc)
        self.assertEqual(owner['start_ticks'], '42')
        verify_owner(owner, 'trial', 'physical', self.env, self.proc)

    def test_owner_reboot_pid_command_and_domain_rejected(self):
        for field, value in [('boot_id', 'old_boot'), ('start_ticks', 41), ('cmdline', ['other'])]:
            with self.subTest(field=field), self.assertRaises(RuntimeError):
                bad = dict(self.owner); bad[field] = value
                verify_owner(bad, 'trial', 'physical', self.env, self.proc)
        for key, value in [('ROS_DOMAIN_ID', '25'), ('IGN_PARTITION', 'another'), ('ASTRIBOT_SIM_INSTANCE', 'other')]:
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                verify_owner(self.owner, 'trial', 'physical', {**self.env, key: value}, self.proc)

    def test_owner_source_identity_required(self):
        with self.assertRaises(RuntimeError):
            verify_owner(self.owner, 'trial', 'another', self.env, self.proc)

    def test_six_physical_cameras_required(self):
        xml = '<robot>' + ''.join('<link name="'+name+'_camera_link"/>' for name in CAMERAS) + '</robot>'
        self.assertEqual(len(camera_structure(xml)), 6)
        with self.assertRaises(RuntimeError):
            camera_structure(xml.replace('<link name="torso_rgbd_camera_link"/>', ''))

    def test_valid_versioned_empty(self):
        self.assertTrue(self.valid())

    def test_kinematic_policy_must_be_explicit_and_still_empty(self):
        self.latest['diagnostic']['policy']='kinematic_inventory_v1'
        self.assertFalse(self.valid())
        self.assertTrue(empty_ready(self.latest, self.received, 1.1, 1_100_000_000, 'trial', 'physical', 'kinematic_inventory_v1'))
        self.latest['source']['status']=2
        self.assertFalse(empty_ready(self.latest, self.received, 1.1, 1_100_000_000, 'trial', 'physical', 'kinematic_inventory_v1'))

    def test_missing_rejected_latest_age_and_future_accepted(self):
        self.assertFalse(empty_ready({}, {}, 1.1, 1_100_000_000, 'trial', 'physical'))
        self.assertTrue(self.valid(wall=1.31))
        self.assertTrue(self.valid(ros=1_300_000_000))
        self.assertTrue(self.valid(ros=999_999_999))

    def test_different_epochs_and_versions(self):
        for key, field, value in [('source', 'source_epoch', 'old'), ('source', 'revision', 3),
                                  ('geometry', 'attachment_revision', 'old'), ('diagnostic', 'revision', 3)]:
            with self.subTest(key=key, field=field):
                before = copy.deepcopy(self.latest)
                self.latest[key][field] = value
                self.assertFalse(self.valid())
                self.latest = before

    def test_absent_attachments_alone_not_empty(self):
        for field, value in [('full_inventory', False), ('status', 0), ('source_epoch', ''), ('revision', 0)]:
            with self.subTest(field=field):
                before = copy.deepcopy(self.latest)
                self.latest['source'][field] = value
                self.assertFalse(self.valid())
                self.latest = before
        self.latest['ledger']['confirmed'] = False
        self.assertFalse(self.valid())

    def test_nonempty_or_pending_never_ready(self):
        for key, field, value in [('source', 'objects', [{'id': 'load'}]), ('geometry', 'attachment_ids', ['load']),
                                  ('source', 'status', 3), ('geometry', 'complete', False)]:
            with self.subTest(key=key, field=field):
                before = copy.deepcopy(self.latest)
                self.latest[key][field] = value
                self.assertFalse(self.valid())
                self.latest = before

    def test_diagnostic_clock_epoch_and_capture_age(self):
        for field, value in [('clock_epoch', 999), ('stamp_ns', 1), ('stamp_ns', 1_200_000_000)]:
            with self.subTest(field=field, value=value):
                previous = self.latest['diagnostic'][field]
                self.latest['diagnostic'][field] = value
                self.assertEqual(self.valid(),field=='stamp_ns')
                self.latest['diagnostic'][field] = previous

    def test_repeated_capture_does_not_renew_when_ros_clock_freezes(self):
        receipts = CaptureReceipts()
        for wall in (1., 1.1, 100.):
            for key, value in self.latest.items():
                receipts.observe(key, value, wall)
        self.assertTrue(empty_ready(self.latest, receipts.received, 100., 1_100_000_000, 'trial', 'physical'))
        self.assertEqual(receipts.received, {key: 1. for key in self.latest})

    def test_reordered_capture_does_not_renew(self):
        receipts = CaptureReceipts()
        receipts.observe('source', self.latest['source'], 1.)
        self.latest['source']['observed_at'] = {'sec': 0, 'nanosec': 900_000_000}
        receipts.observe('source', self.latest['source'], 1.2)
        self.assertEqual(receipts.received['source'], 1.)

    def test_source_lease_is_not_an_age_gate(self):
        self.latest['source']['valid_until'] = {'sec': 9, 'nanosec': 0}
        self.assertTrue(self.valid())

    def test_readback_rejects_version_change_loss_and_timeout(self):
        key = proof_key(self.latest)
        for ready, other, wall, ros in [(True, key[:-1]+('new_model',), 1.1, 1_100_000_000),
                                        (False, key, 1.1, 1_100_000_000),
                                        (True, key, 1.31, 1_100_000_000)]:
            barrier = ReadbackBarrier(key, 1., 1_000_000_000)
            self.assertFalse(barrier.observe(ready, other, wall, ros))
            self.assertFalse(barrier.observe(True, key, 1.11, 1_110_000_000))
        barrier = ReadbackBarrier(key, 1., 1_000_000_000)
        self.assertTrue(barrier.observe(True, key, 1.1, 1_100_000_000))

    def test_geometry_context_change_invalidates_stable_window(self):
        for field, value in [('source_id', 'restarted_source'), ('clock_epoch', 9)]:
            before = copy.deepcopy(self.latest)
            barrier = ReadbackBarrier(proof_key(self.latest), 1., 1_000_000_000)
            self.latest['geometry'][field] = value
            self.assertFalse(barrier.observe(self.valid(), proof_key(self.latest), 1.1, 1_100_000_000))
            self.latest = before
        barrier = ReadbackBarrier(proof_key(self.latest), 1., 1_000_000_000)
        self.latest['geometry']['header']['frame_id'] = 'another_frame'
        self.assertFalse(barrier.observe(self.valid(), proof_key(self.latest), 1.1, 1_100_000_000))


if __name__ == '__main__':
    unittest.main()
