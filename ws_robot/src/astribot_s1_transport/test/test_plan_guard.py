import unittest
from types import SimpleNamespace
from astribot_s1_transport.core import TaskFailure
from astribot_s1_transport.plan_guard import PlanGuard, ORDERS


class GuardTest(unittest.TestCase):
    def guard(self, operation='PLACE'):
        stages = [SimpleNamespace(stage_id=n + (':1' if n == 'RETREAT' else ''), kind=k)
                  for n,k in ORDERS[operation]]
        return PlanGuard(operation, stages, 10., 'ctx', 'ctx')

    def test_rejects_partial_plan_and_wrong_context(self):
        with self.assertRaisesRegex(TaskFailure, 'INCOMPLETE'):
            PlanGuard('PLACE', self.guard().stages[:-1], 10., 'ctx', 'ctx')
        with self.assertRaisesRegex(TaskFailure, 'CONTEXT'):
            PlanGuard('PLACE', self.guard().stages, 10., 'ctx', 'old')

    def test_rejects_stale_state_attachment_and_time(self):
        for now, actual, attachment, reason in [(131., .1, {'box'}, 'EXPIRED'),
                 (11., .2, {'box'}, 'START_STATE'), (11., float('nan'), {'box'}, 'START_STATE'),
                 (11., .1, set(), 'ATTACHMENT')]:
            with self.subTest(reason=reason), self.assertRaisesRegex(TaskFailure, reason):
                self.guard().check('PREPLACE', 'ARM', now, {'a':actual}, {'a':.1}, attachment, {'box'})

    def test_acknowledgement_required_and_cannot_reuse(self):
        guard = self.guard()
        for name, kind in ORDERS['PLACE']:
            guard.check(name, kind, 11., {'a':.1}, {'a':.1}, set(), set())
            guard.acknowledge()
        self.assertTrue(guard.complete)
        with self.assertRaisesRegex(TaskFailure, 'CONSUMED'):
            guard.check('PREPLACE', 'ARM', 11., {'a':.1}, {'a':.1}, set(), set())

    def test_cannot_skip_to_release(self):
        with self.assertRaisesRegex(TaskFailure, 'STAGE_ORDER'):
            self.guard().check('RELEASE', 'GRIPPER', 11., {'a':.1}, {'a':.1}, set(), set())
