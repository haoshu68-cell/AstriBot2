"""Offline route-state regression using generated ROS service messages.

No ROS graph, motion, or world-model fixture is started; this tests the adapter,
not DDS transport.
"""
import copy
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import Mock

from geometry_msgs.msg import PoseStamped
from astribot_navigation_msgs.srv import ResolveRoute
from astribot_s1_navigation_policy.behavior import Selection
from astribot_s1_navigation_policy.contracts import Planning, Trigger, Version
from astribot_s1_navigation_policy.planning_session import PlanningBudgetExhausted
from astribot_s1_navigation_policy.profile import Profile
from astribot_s1_navigation_policy import route_coordinator as route_module


ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'ws_robot/src/astribot_s1_navigation_policy/config/simulation.json'


Request = ResolveRoute.Request
Response = ResolveRoute.Response


class RecoveryRouteTests(unittest.TestCase):
    def setUp(self):
        self.node = N(
            profile=Profile.load(PROFILE), boot=1, epoch=0,
            create_client=Mock(return_value=Mock()), create_service=Mock(),
            execution=N(version=Version('goal', 0, 0, 0)),
            get_clock=lambda: N(now=lambda: N(
                nanoseconds=10**9, to_msg=lambda: PoseStamped().header.stamp)),
            retire_path_risk=Mock(), path=((0., 0.), (1., 0.)),
            last_robot=N(x=0., y=0., yaw=0., vx=0., vy=0., wz=0.),
            last_world=N(observation_seq=1))
        self.route = route_module.RouteCoordinator(self.node)
        self.request = Request()
        self.request.session_id = 'same-navigation-execution'
        self.request.allow_detour = True
        self.request.reference_path.header.frame_id = 'map'
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.pose.orientation.w = 1.
        goal = copy.deepcopy(start)
        goal.pose.position.x = 1.
        self.request.reference_path.poses = [start, goal]
        self.request.goal = goal
        self.assertEqual(self.call().disposition, Response.KEEP)
        self.node.active_path_key = self.route.key

    def call(self, mode=Request.NORMAL, request=None):
        request = copy.deepcopy(request or self.request)
        request.mode = mode
        return self.route.evaluate_route(request, Response())

    def block(self):
        self.route.failure = 'PERCEPTION_BLOCKED: recovery required'
        self.route.failure_code = Response.OBSTRUCTION_DEADLINE
        self.route.blocked_since = time.monotonic()

    def test_generated_interface_has_recovery_modes(self):
        self.assertEqual((Request.RECOVER, Request.RESUME,
                          Response.RECOVERY_COMMITTED, Response.RESUMED), (3, 4, 5, 6))

    def test_first_valid_held_risk_requests_recovery_without_waiting(self):
        for kind in ('blocked', 'immediate', 'uncertain'):
            with self.subTest(kind=kind):
                self.route.failure = None
                self.route.failure_code = Response.NONE
                self.route.blocked_since = None
                risk = N(blocked=False, immediate=False, uncertain=False, moving=False)
                setattr(risk, kind, True)
                selection = self.route.advance(Selection('HOLD', 0., 'YIELD', episode=1),
                                               risk, True)
                self.assertEqual(selection.motion, 'HOLD')
                response = self.call()
                self.assertEqual(response.disposition, Response.BLOCKED)
                self.assertEqual(response.reason_code, Response.OBSTRUCTION_DEADLINE)
                self.assertEqual(response.reason, 'PERCEPTION_BLOCKED: recovery required')
        self.node.create_client.return_value.call_async.assert_not_called()

    def test_invalid_input_or_safe_progress_does_not_request_recovery(self):
        risk = N(blocked=True, immediate=False, uncertain=False, moving=False)
        self.route.advance(Selection('HOLD', 0., 'INPUT_UNAVAILABLE'), risk, False)
        self.assertIsNone(self.route.failure)
        self.assertEqual(self.route.failure_code, Response.NONE)
        self.route.advance(Selection('SLOW', .1, 'PREDICTED_CONFLICT'), risk, True)
        self.assertIsNone(self.route.failure)
        self.assertEqual(self.route.failure_code, Response.NONE)

    def test_recover_requires_existing_typed_obstruction(self):
        self.assertEqual(self.call(Request.RECOVER).disposition, Response.BLOCKED)
        self.route.failure = 'INPUT_UNAVAILABLE: input deadline'
        self.route.failure_code = Response.NONE
        response = self.call(Request.RECOVER)
        self.assertEqual(response.disposition, Response.BLOCKED)
        self.assertEqual(self.route.failure, 'INPUT_UNAVAILABLE: input deadline')
        self.assertEqual(self.route.execution_mode, Request.NORMAL)

    def test_foreign_recovery_context_cannot_change_existing_execution(self):
        self.block()
        for field in ('session', 'path', 'goal'):
            with self.subTest(field=field):
                request = copy.deepcopy(self.request)
                if field == 'session':
                    request.session_id = 'foreign'
                elif field == 'path':
                    request.reference_path.poses[0].pose.position.y = .1
                else:
                    request.goal.pose.position.x = 2.
                    request.reference_path.poses[-1] = copy.deepcopy(request.goal)
                self.assertEqual(self.call(Request.RECOVER, request).disposition,
                                 Response.BLOCKED)
                self.assertEqual(self.route.context, self.request.session_id)
                self.assertEqual(self.route.reference, self.request.reference_path)
                self.assertEqual(self.route.execution_mode, Request.NORMAL)
                self.assertEqual(self.route.failure_code, Response.OBSTRUCTION_DEADLINE)

    def test_recover_cancels_old_candidate_and_holds_until_resume(self):
        self.block()
        future = Mock()
        self.route.future = future
        self.route.request = object()
        self.route.ready = copy.deepcopy(self.request.reference_path)
        self.route.holding = True
        self.assertEqual(self.call(Request.RECOVER).disposition, Response.RECOVERY_COMMITTED)
        future.cancel.assert_called_once()
        self.node.create_client.return_value.remove_pending_request.assert_called_once_with(future)
        self.assertIsNone(self.route.future)
        self.assertIsNone(self.route.request)
        self.assertIsNone(self.route.ready)
        clear = N(blocked=False, immediate=False, uncertain=False, moving=False)
        selection = self.route.advance(Selection('CONTINUE', .35, 'CLEAR'), clear, True)
        self.assertEqual((selection.motion, selection.speed), ('HOLD', 0.))
        self.assertEqual(self.route.execution_mode, Request.RECOVER)

    def test_resume_requires_recovery_and_original_binding(self):
        self.block()
        self.assertEqual(self.call(Request.RESUME).disposition, Response.BLOCKED)
        self.assertEqual(self.call(Request.RECOVER).disposition, Response.RECOVERY_COMMITTED)
        for field in ('session', 'path', 'goal'):
            with self.subTest(field=field):
                request = copy.deepcopy(self.request)
                if field == 'session':
                    request.session_id = 'foreign'
                elif field == 'path':
                    request.reference_path.poses[0].pose.position.y = -.2
                else:
                    request.goal.pose.position.x = 2.
                    request.reference_path.poses[-1] = copy.deepcopy(request.goal)
                self.assertEqual(self.call(Request.RESUME, request).disposition, Response.BLOCKED)
                self.assertEqual(self.route.execution_mode, Request.RECOVER)

    def test_resume_clears_obstruction_without_new_session_or_goal_budget(self):
        self.block()
        session = self.route.session
        version = self.route.version
        for _ in range(2):
            pending = session.request(self.route.version, Trigger.PATH_RISK,
                                      frozenset({Planning.LOCAL}), 1, self.route.steady())
            session.retire(pending)
        self.assertIsNotNone(session._blocked_at)
        self.assertEqual(self.call(Request.RECOVER).disposition, Response.RECOVERY_COMMITTED)
        self.node.retire_path_risk.reset_mock()
        response = self.call(Request.RESUME)
        self.assertEqual(response.disposition, Response.RESUMED)
        self.assertEqual(self.route.execution_mode, Request.NORMAL)
        self.assertIsNone(self.route.failure)
        self.assertEqual(self.route.failure_code, Response.NONE)
        self.assertIsNone(self.route.blocked_since)
        self.node.retire_path_risk.assert_called_once()
        self.assertIs(self.route.session, session)
        self.assertEqual(self.route.version, version)
        self.assertEqual(session._attempts, 2)
        self.assertIsNone(session._blocked_at)
        self.assertEqual(self.route.context, self.request.session_id)
        self.assertEqual(self.route.goal, self.request.goal)
        self.assertEqual(self.call().disposition, Response.KEEP)
        replanned = copy.deepcopy(self.request)
        replanned.reference_path.poses[0].pose.position.y = -.2
        self.assertEqual(self.call(request=replanned).disposition, Response.KEEP)
        self.assertEqual(self.route.context, self.request.session_id)
        self.assertEqual(self.route.reference, replanned.reference_path)
        self.assertEqual(self.route.version.path_revision, version.path_revision + 1)
        self.assertEqual(session._attempts, 2)
        self.assertFalse(self.route.recovery_resumed)

    def test_resume_repeated_poll_is_idempotent(self):
        self.block()
        self.assertEqual(self.call(Request.RECOVER).disposition, Response.RECOVERY_COMMITTED)
        self.assertEqual(self.call(Request.RESUME).disposition, Response.RESUMED)
        session, version = self.route.session, self.route.version
        for _ in range(3):
            response = self.call(Request.RESUME)
            self.assertEqual((response.disposition, response.reason),
                             (Response.RESUMED, 'RECOVERY_REPLAN_READY'))
            self.assertEqual(self.route.execution_mode, Request.NORMAL)
            self.assertIs(self.route.session, session)
            self.assertEqual(self.route.version, version)
            self.assertTrue(self.route.recovery_resumed)
            self.assertIsNone(self.route.failure)
            self.assertIsNone(self.route.blocked_since)

    def test_resume_ack_survives_clear_policy_tick(self):
        self.block()
        self.assertEqual(self.call(Request.RECOVER).disposition, Response.RECOVERY_COMMITTED)
        request=copy.deepcopy(self.request)
        request.mode=Request.RESUME
        self.assertEqual(self.route.resolve(request,Response()).reason,'ROUTE_EVALUATION_PENDING')
        self.route.process_route()
        clear=N(blocked=False,immediate=False,uncertain=False,moving=False)
        # Same order as PolicyNode.tick: evaluate the mailbox, then advance.
        self.route.advance(Selection('CONTINUE',.35,'CLEAR'),clear,True)
        self.assertEqual(self.route.resolve(request,Response()).disposition,Response.RESUMED)
        # Ordinary path/candidate evidence must still be invalidated by cancel.
        self.route.resolve(self.request,Response())
        self.route.process_route()
        self.route.cancel()
        self.assertIsNone(self.route.route_response)

    def test_resume_and_replanned_path_do_not_restore_exhausted_goal_budget(self):
        session = self.route.session
        for _ in range(session.budget.max_requests_per_goal):
            pending = session.request(self.route.version, Trigger.PATH_RISK,
                                      frozenset({Planning.LOCAL}), 1, self.route.steady())
            session.retire(pending)
        self.block()
        self.assertEqual(self.call(Request.RECOVER).disposition, Response.RECOVERY_COMMITTED)
        self.assertEqual(self.call(Request.RESUME).disposition, Response.RESUMED)
        replanned = copy.deepcopy(self.request)
        replanned.reference_path.poses[0].pose.position.y = -.2
        self.assertEqual(self.call(request=replanned).disposition, Response.KEEP)
        self.assertIs(self.route.session, session)
        self.assertEqual(self.route.context, self.request.session_id)
        self.assertEqual(self.route.goal, self.request.goal)
        with self.assertRaisesRegex(PlanningBudgetExhausted, 'per-goal planning attempts'):
            session.request(self.route.version, Trigger.PATH_RISK,
                            frozenset({Planning.LOCAL}), 2, self.route.steady())
        self.assertEqual(self.call(Request.RESUME, replanned).disposition, Response.BLOCKED)

    def test_mailbox_does_not_treat_pending_recovery_as_committed(self):
        self.block()
        self.route.route_response = (self.route.request_key(self.request),
                                     time.monotonic(), self.call())
        request = copy.deepcopy(self.request)
        request.mode = Request.RECOVER
        response = self.route.resolve(request, Response())
        self.assertEqual((response.disposition, response.reason),
                         (Response.KEEP, 'ROUTE_EVALUATION_PENDING'))
        self.route.process_route()
        self.assertEqual(self.route.resolve(request, Response()).disposition,
                         Response.RECOVERY_COMMITTED)
        request.mode = Request.RESUME
        self.assertEqual(self.route.resolve(request, Response()).disposition, Response.KEEP)
        self.route.process_route()
        self.assertEqual(self.route.resolve(request, Response()).disposition, Response.RESUMED)
        # A repeated transport poll may already have queued the same command;
        # its processing must still acknowledge the completed handover.
        for _ in range(3):
            self.route.process_route()
            self.assertEqual(self.route.resolve(request, Response()).disposition, Response.RESUMED)


if __name__ == '__main__':
    unittest.main()
