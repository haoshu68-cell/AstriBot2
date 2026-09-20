import json
from pathlib import Path
import tempfile
import unittest
from astribot_s1_transport.core import Canceled, Ledger, ResourceLease, TaskFailure, TransportTask, validate_scenario

CONFIG = json.loads((Path(__file__).parents[1] / 'config/warehouse_transfer.json').read_text())


class Backend:
    evidence_level = 'offline_contract_only'

    def __init__(self, fail='', cancel=False):
        self.fail, self.cancel = fail, cancel
        self.calls = []
        self.held = False
        self.attached = False
        self.released = False

    def check(self):
        pass

    def invoke(self, name):
        self.calls.append(name)
        if name == self.fail:
            raise Canceled('USER_CANCEL') if self.cancel else TaskFailure(name)

    def admit(self): self.invoke('admit')
    def admit_recovery(self): self.invoke('admit_recovery')
    def hold(self): self.invoke('hold'); self.held = True
    def create_scene(self): self.invoke('scene')
    def locate_object(self): self.invoke('locate')
    def look_pick(self): self.invoke('look')
    def prepare_manipulation(self, operation): self.invoke(operation + '_plan')
    def skill(self, operation, *args): self.invoke(operation)
    def arm_pose(self, station, lift, inward_m=0.): self.invoke(station)
    def attach(self): self.invoke('attach'); self.attached = True
    def transport_envelope(self):
        assert self.attached
        self.invoke('envelope')
    def navigate(self):
        assert self.attached
        self.invoke('navigate')
    def detach(self): self.invoke('detach'); self.attached = False; self.released = True
    def confirm_place(self): self.invoke('place_confirm')
    def verify_place(self):
        assert self.released
        self.invoke('verify')
    def stop_and_hold(self): self.calls.append('stop_and_hold'); self.held = True


class TransactionTest(unittest.TestCase):
    def test_placed_recovery_never_repeats_irreversible_steps(self):
        with tempfile.TemporaryDirectory() as directory:
            ledger = Ledger(directory, CONFIG['object_id'])
            ledger.object.state = 'PLACED'; ledger.object.version = 5
            ledger.emit('FAULT', reason='RETREAT_FAILED')
            ledger = Ledger.resume_placed(directory, CONFIG['object_id'])
            backend = Backend(); backend.released = True
            self.assertTrue(TransportTask(backend, CONFIG, ledger).resume_placed())
            self.assertEqual(ledger.object.version, 5)
            self.assertEqual(ledger.stage, 'SUCCEEDED')
            self.assertFalse(set(backend.calls) & {'scene', 'pick', 'open', 'close', 'attach', 'detach', 'navigate'})
            self.assertEqual(backend.calls[0], 'admit_recovery')

    def test_recovery_refuses_unreleased_and_completed_ledgers(self):
        for state, stage in [('ATTACHED', 'FAULT'), ('RELEASE_PENDING', 'FAULT'), ('PLACED', 'SUCCEEDED')]:
            with self.subTest(state=state, stage=stage), tempfile.TemporaryDirectory() as directory:
                ledger = Ledger(directory, CONFIG['object_id']); ledger.object.state = state; ledger.emit(stage)
                with self.assertRaisesRegex(TaskFailure, 'LEDGER_MISMATCH'):
                    Ledger.resume_placed(directory, CONFIG['object_id'])

    def test_recovery_rejects_snapshot_journal_divergence(self):
        with tempfile.TemporaryDirectory() as directory:
            ledger = Ledger(directory, CONFIG['object_id']); ledger.object.state = 'PLACED'; ledger.emit('FAULT')
            record = json.loads((Path(directory)/'state.json').read_text()); record['sequence'] += 1
            (Path(directory)/'state.json').write_text(json.dumps(record))
            with self.assertRaisesRegex(TaskFailure, 'LEDGER_MISMATCH'):
                Ledger.resume_placed(directory, CONFIG['object_id'])

    def execute(self, backend):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        ledger = Ledger(directory.name, CONFIG['object_id'])
        result = TransportTask(backend, CONFIG, ledger).run()
        return result, ledger, json.loads((Path(directory.name) / 'state.json').read_text())

    def test_complete(self):
        ok, ledger, record = self.execute(Backend())
        self.assertTrue(ok)
        self.assertEqual(ledger.object.state, 'PLACED')
        self.assertEqual(record['stage'], 'SUCCEEDED')
        self.assertEqual(record['evidence_level'], 'offline_contract_only')

    def test_vla_place_failure_preserves_attachment_and_never_releases(self):
        class PolicyFailure(Backend):
            def prepare_manipulation(self, operation):
                if operation == 'PLACE':
                    raise TaskFailure('VLA_INFERENCE_TIMEOUT:infer')
                super().prepare_manipulation(operation)
        backend = PolicyFailure()
        ok, ledger, record = self.execute(backend)
        self.assertFalse(ok)
        self.assertEqual(ledger.object.state, 'ATTACHED')
        self.assertFalse(backend.released)
        self.assertEqual(backend.calls.count('open'), 1)  # Initial empty gripper only.
        self.assertEqual(backend.calls[-1], 'stop_and_hold')

    def test_navigation_failure_retains_payload(self):
        backend = Backend('navigate')
        ok, ledger, record = self.execute(backend)
        self.assertFalse(ok)
        self.assertEqual(ledger.object.state, 'ATTACHED')
        self.assertNotIn('detach', backend.calls)
        self.assertTrue(record['recovery_required'])
        self.assertTrue(backend.held)

    def test_failed_complete_place_plan_never_releases(self):
        backend = Backend('PLACE_plan')
        ok, ledger, _ = self.execute(backend)
        self.assertFalse(ok)
        self.assertEqual(ledger.object.state, 'ATTACHED')
        self.assertEqual(backend.calls.count('open'), 1)  # initial open only
        self.assertNotIn('detach', backend.calls)

    def test_failed_complete_pick_plan_never_closes(self):
        backend = Backend('PICK_plan')
        ok, ledger, _ = self.execute(backend)
        self.assertFalse(ok)
        self.assertEqual(ledger.object.state, 'WORLD')
        self.assertNotIn('close', backend.calls)

    def test_attach_uncertainty_requires_reconciliation(self):
        ok, ledger, record = self.execute(Backend('attach'))
        self.assertFalse(ok)
        self.assertEqual(ledger.object.state, 'ATTACH_PENDING')
        self.assertTrue(record['recovery_required'])

    def test_release_uncertainty_is_not_placed(self):
        ok, ledger, record = self.execute(Backend('detach'))
        self.assertFalse(ok)
        self.assertEqual(ledger.object.state, 'RELEASE_PENDING')

    def test_cancel_preserves_payload(self):
        ok, ledger, record = self.execute(Backend('navigate', True))
        self.assertFalse(ok)
        self.assertEqual(record['stage'], 'CANCELED')
        self.assertEqual(ledger.object.state, 'ATTACHED')

    def test_failures_do_not_report_success(self):
        for failure in ('admit', 'hold', 'scene', 'named', 'open', 'look', 'locate', 'pick', 'close',
                        'attach', 'envelope', 'navigate', 'place', 'place_confirm', 'detach', 'verify'):
            with self.subTest(failure=failure):
                ok, _, record = self.execute(Backend(failure))
                self.assertFalse(ok)
                self.assertEqual(record['stage'], 'FAULT')

    def test_invalid_head_target_rejected_before_execution(self):
        for target in ([float('nan'), 0.], [0.], [1.6, 0.], [0., 1.3]):
            with self.subTest(target=target), self.assertRaisesRegex(TaskFailure, 'INVALID_HEAD_TARGET'):
                validate_scenario(dict(CONFIG, head_pick_joints=target))

    def test_resources_cannot_be_taken_before_release(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'lease'
            with ResourceLease(path):
                with self.assertRaisesRegex(TaskFailure, 'RESOURCES_BUSY'):
                    with ResourceLease(path):
                        pass
            with ResourceLease(path):
                pass

    def test_hardware_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(TaskFailure, 'SIMULATION_ONLY'):
                TransportTask(Backend(), dict(CONFIG, environment='hardware'), Ledger(directory, 'box'))

    def test_invalid_station_configuration_rejected_before_motion(self):
        for override, reason in (({'nav_waypoints': [[0., float('nan'), 0.]]}, 'INVALID_NAV_WAYPOINT'),
                                 ({'place_orientation_xyzw': [0., 0., 0., 0.]}, 'INVALID_PLACE_ORIENTATION')):
            with self.subTest(reason=reason), tempfile.TemporaryDirectory() as directory:
                backend = Backend()
                with self.assertRaisesRegex(TaskFailure, reason):
                    TransportTask(backend, dict(CONFIG, **override), Ledger(directory, 'box'))
                self.assertEqual(backend.calls, [])


if __name__ == '__main__':
    unittest.main()
