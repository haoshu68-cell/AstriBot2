"""Exercise the real route adapter and wire protocol without a ROS graph."""
import copy
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import Mock, patch

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from astribot_navigation_msgs.srv import ResolveRoute
from astribot_s1_navigation_policy.behavior import Selection
from astribot_s1_navigation_policy.contracts import Version, Stamp
from astribot_s1_navigation_policy.observer_node import PolicyObserver
from astribot_s1_navigation_policy.policy_node import PolicyNode
from astribot_s1_navigation_policy.profile import Profile
from astribot_s1_navigation_policy.route_coordinator import RouteCoordinator

PROFILE=Path(__file__).resolve().parents[2]/'ws_robot/src/astribot_s1_navigation_policy/config/simulation.json'


class WorkstationRouteTests(unittest.TestCase):
    def setUp(self):
        self.node=N(profile=Profile.load(PROFILE),boot=1,epoch=0,
                    create_client=Mock(return_value=Mock()),create_service=Mock(),
                    execution=N(version=Version('goal',0,0,0)),
                    get_clock=lambda:N(now=lambda:N(to_msg=lambda:PoseStamped().header.stamp)),
                    retire_path_risk=Mock())
        self.route=RouteCoordinator(self.node)
        self.request=ResolveRoute.Request();self.request.session_id='session';self.request.allow_detour=True
        self.request.reference_path.header.frame_id='map'
        start=PoseStamped();start.header.frame_id='map';start.pose.orientation.w=1.
        goal=copy.deepcopy(start);goal.pose.position.x=1.
        self.request.reference_path.poses=[start,goal];self.request.goal=goal
        self.assertEqual(self.call().disposition,ResolveRoute.Response.KEEP)
        self.node.active_path_key=self.route.key

    def call(self,mode=ResolveRoute.Request.NORMAL,request=None):
        req=copy.deepcopy(request or self.request);req.mode=mode
        return self.route.evaluate_route(req,ResolveRoute.Response())

    def obstruct(self,valid=True):
        self.route.blocked_since=time.monotonic()-self.node.profile.planning_budget['episode_timeout_s']-1.
        selection=Selection('HOLD',0.,'YIELD',episode=1)
        risk=N(blocked=True,immediate=True,uncertain=False)
        self.route.advance(selection,risk,valid)

    def test_typed_deadline_is_only_obstruction(self):
        self.obstruct();response=self.call()
        self.assertEqual((response.disposition,response.reason_code),(response.BLOCKED,response.OBSTRUCTION_DEADLINE))
        self.route.failure=None;self.obstruct(valid=False);response=self.call()
        self.assertEqual((response.disposition,response.reason_code),(response.BLOCKED,response.NONE))
        self.assertIn('INPUT_UNAVAILABLE',response.reason)

    def test_only_original_obstructed_context_can_commit(self):
        self.assertEqual(self.call(self.request.WORKSTATION_ALIGN).reason,'WORKSTATION_REQUIRES_OBSTRUCTION_DEADLINE')
        self.obstruct()
        for mutation in ('session','path','goal'):
            req=copy.deepcopy(self.request)
            if mutation=='session':req.session_id='another'
            if mutation=='path':req.reference_path.poses[0].pose.position.y=.1
            if mutation=='goal':
                req.goal.pose.position.x=2.;req.reference_path.poses[-1]=copy.deepcopy(req.goal)
            self.assertEqual(self.call(req.WORKSTATION_ALIGN,req).reason,'WORKSTATION_CONTEXT_MISMATCH')
            self.assertEqual(self.route.failure_code,ResolveRoute.Response.OBSTRUCTION_DEADLINE)
        self.node.retire_path_risk.assert_not_called()
        # A normal poll must not relabel the failed route before mode admission.
        changed=copy.deepcopy(self.request);changed.goal.pose.position.x=2.
        changed.reference_path.poses[-1]=copy.deepcopy(changed.goal)
        self.assertEqual(self.call(request=changed).reason,'WORKSTATION_CONTEXT_MISMATCH')
        self.assertEqual(self.route.goal.pose,self.request.goal.pose)

    def test_commit_heartbeat_normal_replay_and_finish(self):
        self.obstruct();response=self.call(self.request.WORKSTATION_ALIGN)
        self.assertEqual(response.disposition,response.ALIGNMENT_COMMITTED)
        self.assertTrue(self.route.workstation_alignment);self.assertIsNone(self.route.failure)
        self.assertEqual(self.route.failure_code,response.NONE)
        self.node.retire_path_risk.assert_called_once()
        self.node.create_client.return_value.call_async.assert_not_called()
        self.assertEqual(self.call(self.request.WORKSTATION_ALIGN).disposition,response.ALIGNMENT_COMMITTED)
        self.node.retire_path_risk.assert_called_once()
        self.assertEqual(self.call().reason,'WORKSTATION_MODE_ALREADY_COMMITTED')
        for _ in range(2):self.assertEqual(self.call(self.request.FINISH).disposition,response.FINISHED)
        self.assertFalse(self.route.workstation_alignment)
        self.assertEqual(self.call(self.request.WORKSTATION_ALIGN).reason,'ROUTE_FINISHED')
        self.assertEqual(self.call().reason,'ROUTE_FINISHED')
        req=copy.deepcopy(self.request);req.session_id='next'
        self.assertEqual(self.call(request=req).disposition,response.KEEP)
        self.assertFalse(self.route.finished);self.assertFalse(self.route.alignment_used)

    def test_input_failure_epoch_and_context_expiration_cannot_be_cleared(self):
        self.obstruct(valid=False)
        self.assertEqual(self.call(self.request.WORKSTATION_ALIGN).reason,'WORKSTATION_REQUIRES_OBSTRUCTION_DEADLINE')
        self.assertIn('INPUT_UNAVAILABLE',self.route.failure)
        self.route.failure=None;self.obstruct()
        self.route.last_poll=time.monotonic()-self.route.takeover['context_timeout_s']-1.
        self.assertEqual(self.call(self.request.WORKSTATION_ALIGN).reason,'WORKSTATION_CONTEXT_UNAVAILABLE')
        self.assertEqual(self.route.failure_code,ResolveRoute.Response.OBSTRUCTION_DEADLINE)
        self.call();self.node.epoch+=1
        self.assertEqual(self.call(self.request.WORKSTATION_ALIGN).reason,'WORKSTATION_CONTEXT_UNAVAILABLE')

    def test_mailbox_never_mistakes_pending_or_normal_cache_for_commit(self):
        self.obstruct();self.route.route_response=(self.route.request_key(self.request),time.monotonic(),self.call())
        req=copy.deepcopy(self.request);req.mode=req.WORKSTATION_ALIGN
        pending=self.route.resolve(req,ResolveRoute.Response())
        self.assertEqual((pending.disposition,pending.reason),(pending.KEEP,'ROUTE_EVALUATION_PENDING'))
        self.route.process_route()
        committed=self.route.resolve(req,ResolveRoute.Response())
        self.assertEqual(committed.disposition,committed.ALIGNMENT_COMMITTED)
        req.mode=req.FINISH
        pending=self.route.resolve(req,ResolveRoute.Response())
        self.assertEqual(pending.disposition,pending.KEEP)
        self.route.process_route()
        self.assertEqual(self.route.resolve(req,ResolveRoute.Response()).disposition,pending.FINISHED)

    def test_retired_old_path_reports_do_not_reassert_hold(self):
        self.obstruct();self.call(self.request.WORKSTATION_ALIGN)
        node=N(coordinator=self.route,path_blocked=True,path_evidence=object(),report_order=(0,1),
               pending_path_report=(object(),1),path_report_lock=threading.Lock())
        PolicyNode.retire_path_risk(node)
        PolicyNode.path_risk(node,Bool(data=True))
        node.pending_path_report=(object(),1)
        PolicyNode.process_path_report(node)
        self.assertFalse(node.path_blocked);self.assertIsNone(node.path_evidence)
        self.assertIsNone(node.pending_path_report);self.assertIsNone(node.report_order)

    def test_actual_policy_tick_maps_mode_and_preserves_input_and_coverage_stop(self):
        self.obstruct();self.call(self.request.WORKSTATION_ALIGN)
        n=PolicyNode.__new__(PolicyNode);n.__dict__.update(self.node.__dict__)
        n.coordinator=self.route;n.process_path_report=lambda:None
        n.stamp=lambda:Stamp(10**9,'sim',0);n.path=();n.last_inputs_valid=True;n.last_evaluation_epoch=0
        n.last_risk=None;n.path_evidence=None;n.active_path_key=None;n.path_blocked=True
        n.last_robot=N(vx=0.,vy=0.,wz=0.);n.last_world=N(version=Version('goal',0,0,0))
        n.health_registry=N(health=lambda now:[],allows_motion=Mock(return_value=True))
        n.selector=N(select=Mock(side_effect=AssertionError('ordinary selector used')))
        n.corridor=N(advance=Mock(side_effect=AssertionError('old corridor used')),evidence={},last_error='')
        n.sequence=0;n.scan_timing=None;n.constraint=Mock();n.typed_state=Mock();n.state=Mock()
        with patch.object(PolicyObserver,'tick',lambda self:None):
            for valid,coverage,hold in ((True,True,False),(False,True,True),(True,False,True)):
                n.last_inputs_valid=valid;n.health_registry.allows_motion.return_value=coverage
                PolicyNode.tick(n)
                msg=n.constraint.publish.call_args.args[0]
                self.assertTrue(msg.workstation_alignment);self.assertEqual(msg.hold,hold)
                self.assertEqual(msg.max_linear_speed,0. if hold else n.profile.max_speed_m_s)
        n.selector.select.assert_not_called();n.corridor.advance.assert_not_called()


if __name__=='__main__':unittest.main()
