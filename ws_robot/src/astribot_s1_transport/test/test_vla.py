import copy
import json
from pathlib import Path
import threading
import time
import unittest

from astribot_s1_transport.core import Canceled, TaskFailure
from astribot_s1_transport.vla_contract import VERSION, UNITS, validate_config, validate_capabilities, validate_proposal
from astribot_s1_transport.vla_policy import ReferencePolicy, HttpPolicy, PolicySession
from astribot_s1_transport.vla_server import make_server


CONFIG = validate_config(json.loads((Path(__file__).parents[1]/'config/vla_reference.json').read_text()))


def request():
    pose = dict(position=[.2, .6, 1.2], quaternion=[0., 0., 0., 1.])
    return dict(schema=VERSION, episode_id='episode', request_id='request', sequence=0, context_id='scene-1',
                operation='PICK', frame_id='base', units=UNITS,
                observation=dict(arm_joint_names=[f'j{i}' for i in range(7)]),
                nominal_action=dict(type='mtc_targets', frame_id='base', group='arm_left',
                    pre_target=copy.deepcopy(pose), target=copy.deepcopy(pose), exit_targets=[copy.deepcopy(pose)]))


class VlaContractTest(unittest.TestCase):
    def setUp(self):
        self.req = request()
        self.cap = ReferencePolicy().call('capabilities', {})
        self.response = ReferencePolicy().call('infer', self.req)

    def validate(self):
        return validate_proposal(self.req, self.response, self.cap, CONFIG)

    def test_nominal_and_bounded_target(self):
        self.validate()
        self.response['action']['target']['position'][0] += .005
        self.assertAlmostEqual(self.validate()['target']['position'][0], .205)

    def test_reject_identity_replay(self):
        for key in ('schema', 'episode_id', 'request_id', 'sequence', 'context_id'):
            old = self.response[key]
            self.response[key] = 'old'
            with self.subTest(key=key), self.assertRaisesRegex(TaskFailure, 'CONTEXT'): self.validate()
            self.response[key] = old
        self.response['sequence'] = False
        with self.assertRaisesRegex(TaskFailure, 'CONTEXT'): self.validate()

    def test_reject_units_frame_group_and_extra_controller_fields(self):
        mutations = [lambda r:r.update(units=dict(UNITS, position='mm')),
                     lambda r:r['action'].update(frame_id='camera'),
                     lambda r:r['action'].update(group='both_arms'),
                     lambda r:r['action'].update(gripper='open')]
        for mutate in mutations:
            self.response = ReferencePolicy().call('infer', self.req)
            mutate(self.response)
            with self.assertRaises(TaskFailure): self.validate()

    def test_reject_nan_nonunit_quaternion_and_task_region(self):
        cases = [('position', [float('nan'), 0., 0.]), ('position', [.3, .6, 1.2]),
                 ('quaternion', [0., 0., 0., 0.]), ('quaternion', [0., 0., 1., 0.]),
                 ('position', ['.2', .6, 1.2])]
        for field, value in cases:
            self.response = ReferencePolicy().call('infer', self.req)
            self.response['action']['target'][field] = value
            with self.subTest(field=field, value=value), self.assertRaises(TaskFailure): self.validate()

    def test_reject_partial_exit_targets(self):
        self.response['action']['exit_targets'] = []
        with self.assertRaisesRegex(TaskFailure, 'COUNT'): self.validate()

    def test_chunk_shadow_contract(self):
        self.cap['action_types'] += ['ee_delta_chunk', 'joint_position_chunk']
        self.response['action'] = dict(type='ee_delta_chunk', frame_id='base', group='arm_left',
            samples=[dict(time_s=.1, position_delta_m=[.001,0.,0.], rotation_vector_rad=[0.,0.,0.], gripper_width_m=.06)])
        with self.assertRaisesRegex(TaskFailure, 'REQUIRES_SHADOW'): self.validate()
        shadow = dict(CONFIG, mode='shadow')
        validate_proposal(self.req, self.response, self.cap, shadow)
        self.response['action']['samples'] *= 2
        with self.assertRaisesRegex(TaskFailure, 'TIME'): validate_proposal(self.req, self.response, self.cap, shadow)
        self.response['action'] = dict(type='joint_position_chunk', frame_id='base', group='arm_left',
            joint_names=self.req['observation']['arm_joint_names'], samples=[dict(time_s=.1, positions_rad=[0.]*7, gripper_width_m=.06)])
        validate_proposal(self.req, self.response, self.cap, shadow)
        self.response['action']['joint_names'] = list(reversed(self.response['action']['joint_names']))
        with self.assertRaisesRegex(TaskFailure, 'JOINT_ORDER'): validate_proposal(self.req, self.response, self.cap, shadow)

    def test_capability_and_config_validation(self):
        for field,value in [('schema','2'), ('robot_model','franka'), ('normalization_id',''), ('units',{})]:
            cap = dict(self.cap, **{field:value})
            with self.subTest(field=field), self.assertRaises(TaskFailure): validate_capabilities(cap, 'mtc')
        for field,value in [('timeout_s',float('nan')), ('mode','raw_control'), ('max_target_delta_m', 1.), ('cameras',[])]:
            with self.subTest(field=field), self.assertRaises(TaskFailure): validate_config(dict(CONFIG, **{field:value}))


class VlaSessionTest(unittest.TestCase):
    def session(self, adapter=None, **overrides):
        session = PolicySession(dict(CONFIG, **overrides), 'episode', lambda *args:None, adapter)
        self.addCleanup(lambda: session.close(dict(status='test_done')))
        return session

    def test_discovery_reset_infer_and_replay(self):
        session = self.session()
        session.start(lambda:None)
        self.assertEqual(session.infer(request(), lambda:None)['type'], 'mtc_targets')
        with self.assertRaisesRegex(TaskFailure, 'REPLAY'): session.infer(request(), lambda:None)

    def test_local_adapter_cannot_mutate_reference_bounds(self):
        class Mutating(ReferencePolicy):
            def call(self, endpoint, packet):
                if endpoint == 'infer':
                    packet['nominal_action']['target']['position'][0] = 10.
                return super().call(endpoint, packet)
        session = self.session(Mutating())
        session.start(lambda:None)
        req = request()
        with self.assertRaisesRegex(TaskFailure,'OUTSIDE_TASK_REGION'): session.infer(req,lambda:None)
        self.assertEqual(req['nominal_action']['target']['position'][0], .2)

    def test_failed_reset_cannot_infer(self):
        class ResetFailure(ReferencePolicy):
            def call(self, endpoint, packet):
                return {} if endpoint == 'reset' else super().call(endpoint, packet)
        session = self.session(ResetFailure())
        with self.assertRaisesRegex(TaskFailure,'RESET_NOT_ACKNOWLEDGED'): session.start(lambda:None)
        with self.assertRaisesRegex(TaskFailure,'SESSION_NOT_READY'): session.infer(request(),lambda:None)

    def test_deadline_and_late_result_never_reused(self):
        class Slow(ReferencePolicy):
            def call(self, endpoint, packet):
                if endpoint == 'infer': time.sleep(.12)
                return super().call(endpoint, packet)
        session = self.session(Slow(), timeout_s=.04)
        session.start(lambda:None)
        with self.assertRaisesRegex(TaskFailure, 'TIMEOUT'): session.infer(request(), lambda:None)
        time.sleep(.13)
        self.assertTrue(session.closed)
        with self.assertRaisesRegex(TaskFailure, 'CLOSED'): session.invoke('infer', request(), lambda:None)

    def test_cancel_does_not_wait_for_remote_inference(self):
        gate = threading.Event()
        class Blocked(ReferencePolicy):
            def call(self, endpoint, packet):
                if endpoint == 'infer': gate.wait(1.)
                return super().call(endpoint, packet)
        session = self.session(Blocked())
        session.start(lambda:None)
        start = time.monotonic()
        def check():
            if time.monotonic()-start > .04: raise Canceled('USER_CANCEL')
        try:
            with self.assertRaises(Canceled): session.infer(request(), check)
            self.assertLess(time.monotonic()-start, .2)
            self.assertTrue(session.closed)
        finally: gate.set()

    def test_feedback_does_not_block_task(self):
        gate = threading.Event()
        class Blocked(ReferencePolicy):
            def call(self, endpoint, packet):
                if endpoint == 'feedback': gate.wait(1.)
                return super().call(endpoint, packet)
        session = self.session(Blocked())
        start = time.monotonic()
        session.enqueue('feedback', {'stage':'ATTACH_CONFIRM'})
        self.assertLess(time.monotonic()-start, .1)
        gate.set()

    def test_real_http_roundtrip_and_protocol_error(self):
        server = make_server('127.0.0.1', 0, ReferencePolicy())
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            adapter = HttpPolicy('http://127.0.0.1:' + str(server.server_port), 1.)
            session = self.session(adapter)
            session.start(lambda:None)
            self.assertEqual(session.infer(request(), lambda:None), request()['nominal_action'])
            with self.assertRaisesRegex(TaskFailure, 'HTTP_STATUS_400'): adapter.call('infer', {'schema':'wrong'})
            session.close({'status':'done'})
        finally:
            server.shutdown(); server.server_close(); thread.join(1.)


if __name__ == '__main__':
    unittest.main()
