"""Single-use ordered plan contract; deliberately independent of ROS."""
import math
from .core import TaskFailure

ORDERS = {
    'PICK': [('PREGRASP', 'ARM'), ('GRASP_APPROACH', 'ARM'), ('GRASP_CONFIRM', 'GRIPPER'),
             ('ATTACH_CONFIRM', 'ATTACH'), ('LIFT', 'ARM'), ('TRANSPORT_POSTURE', 'ARM')],
    'PLACE': [('PREPLACE', 'ARM'), ('PLACE_APPROACH', 'ARM'), ('RELEASE', 'GRIPPER'),
              ('DETACH_CONFIRM', 'DETACH'), ('RETREAT', 'ARM'), ('STOW', 'ARM')],
}


def stage_name(name):
    return name.split(':', 1)[0]


class PlanGuard:
    def __init__(self, operation, stages, created, context, returned_context):
        if not context or context != returned_context:
            raise TaskFailure('MTC_CONTEXT_MISMATCH')
        if [(stage_name(s.stage_id), s.kind) for s in stages] != ORDERS.get(operation):
            raise TaskFailure('MTC_INCOMPLETE_SEQUENCE')
        self.stages, self.created, self.index = stages, created, 0

    def check(self, name, kind, now, actual, expected, attached, expected_attached):
        if self.index >= len(self.stages):
            raise TaskFailure('MTC_PLAN_ALREADY_CONSUMED')
        stage = self.stages[self.index]
        if (stage_name(stage.stage_id), stage.kind) != (name, kind):
            raise TaskFailure('MTC_STAGE_ORDER')
        if not 0 <= now - self.created <= 120.:
            raise TaskFailure('MTC_PLAN_EXPIRED')
        if not expected or any(not math.isfinite(v) or not math.isfinite(actual.get(j, math.nan)) or
                               abs(actual[j] - v) > .025 for j, v in expected.items()):
            raise TaskFailure('MTC_START_STATE_CHANGED')
        if attached != expected_attached:
            raise TaskFailure('MTC_ATTACHMENT_CHANGED')
        return stage

    def acknowledge(self):
        self.index += 1

    @property
    def complete(self):
        return self.index == len(self.stages)
