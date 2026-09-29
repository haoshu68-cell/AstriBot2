import math
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
from astribot_s1_navigation_policy.profile import Profile
from astribot_s1_navigation_policy.robot_envelope import FixedEnvelopeProfile
from astribot_s1_navigation_policy.corridor import Corridor
from astribot_s1_navigation_policy.fixed_corridor import FixedCorridorPolicy
from astribot_s1_navigation_policy.behavior import Selection

class CorridorTests(unittest.TestCase):
    def setUp(self):
        self.p=FixedEnvelopeProfile(Profile.load(Path(__file__).parents[1]/'config/simulation.json'))
        self.p.footprint_xy=np.array([[-.32,-.32],[.32,-.32],[.32,.32],[-.32,.32]])
        self.c=Corridor('test',(0.,0.),(3.,0.),.85,('unused_label',),.01,.01)
        self.policy=FixedCorridorPolicy(self.p,[self.c])
        self.v=SimpleNamespace(goal_id='goal',path_revision=1,map_epoch=1,localization_epoch=1,envelope_epoch=1,clock_epoch=1)
        self.r=SimpleNamespace(x=-1.,y=0.,yaw=0.,vx=.1,vy=0.,wz=0.)
    def evaluate(self,path=((-1.,0.),(4.,0.)),clear=True):
        return self.policy.evaluate(Selection('PROCEED',.2,'CLEAR'),self.r,path,self.v,'any_posture',True,clear,1.,True,True)
    def test_aligned_85cm_no_stop(self):
        self.assertEqual(self.evaluate().state,'TRANSIT')
    def test_centering_finishes_at_target_not_reservation_edge(self):
        self.c=Corridor('wide',(0.,0.),(3.,0.),1.5,('unused',))
        self.policy=FixedCorridorPolicy(self.p,[self.c])
        for side in (.15,.10,.010001):
            self.r.y=side
            self.assertEqual(self.evaluate().state,'CENTER')
        for side in (.01,.009999,0.):
            self.r.y=side
            self.assertEqual(self.evaluate().state,'TRANSIT')

    def test_asymmetric_center_target_and_exit_are_retained(self):
        self.c=Corridor('narrow',(0.,0.),(1.2,0.),1.25,('unused',))
        self.p.footprint_xy=np.array([[-.4,-.368],[.4,-.368],[.4,.513],[-.4,.513]])
        self.policy=FixedCorridorPolicy(self.p,[self.c])
        target=-.0725
        for side in (.15,-.045):
            self.r.y=side
            result=self.evaluate(((-1.,target),(2.,target)))
            self.assertEqual(result.state,'CENTER')
            self.assertAlmostEqual(result.centering_target[1],target)
        self.r.x=1.3;self.r.y=target
        self.assertEqual(self.evaluate(((-1.,target),(2.,target))).state,'TRANSIT')
        self.r.x=1.2+.4+self.policy.margin(self.c)+.0001
        self.assertEqual(self.evaluate(((-1.,target),(2.,target))).state,'NORMAL')

    def test_far_transit_obstacle_does_not_block_entrance_alignment(self):
        self.r.yaw=.1745
        self.assertEqual(self.evaluate(clear=False).state,'ALIGN')
        self.r.yaw=0.;self.r.y=.02
        self.assertEqual(self.evaluate(clear=False).state,'CENTER')

    def test_concatenated_waypoint_endpoints_are_not_reverse_segments(self):
        path=((-1.,0.),(0.,0.),(0.,0.),(1.,0.),(1.,0.),(3.,0.),(4.,0.))
        self.assertEqual(self.evaluate(path).state,'TRANSIT')
        self.assertFalse(self.policy.route_fits(self.c,((-1.,0.),(1.,0.),(.9,0.),(4.,0.))))
        self.assertFalse(self.policy.route_fits(self.c,((-1.,0.),(1.,.3),(1.,.3),(4.,0.))))
    def test_inside_aligned_restart(self):
        self.r.x=1.;self.assertEqual(self.evaluate().state,'TRANSIT')
    def test_inside_misaligned_no_rotation(self):
        self.r.x=1.;self.r.yaw=.2
        self.assertEqual(self.evaluate().selection.reason,'ROTATION_FORBIDDEN_IN_NARROW_PASSAGE')
    def test_asymmetric_offset(self):
        self.p.footprint_xy=np.array([[-.32,-.2],[.32,-.2],[.32,.44],[-.32,.44]])
        self.assertAlmostEqual(self.policy.target_offset(self.c),-.12)
        self.r.y=-.12
        self.assertEqual(self.evaluate(((-1.,-.12),(4.,-.12))).state,'TRANSIT')
        self.assertEqual(self.evaluate().selection.reason,'CORRIDOR_OFFSET_ROUTE_REQUIRED')
    def test_front_extension_changes_rotation_not_straight_width(self):
        self.p.footprint_xy=np.array([[-.32,-.32],[1.,-.32],[1.,.32],[-.32,.32]])
        self.r.x=-1.5;self.assertEqual(self.evaluate(((-1.5,0.),(4.,0.))).state,'TRANSIT')
        self.assertGreater(self.policy.half_projection(.2),.32)
    def test_uncertainty_and_obstacle_preserved(self):
        self.assertEqual(self.evaluate(clear=False).selection.motion,'HOLD')
        self.c=Corridor('test',(0.,0.),(3.,0.),.85,('irrelevant',),.05,.05)
        self.policy=FixedCorridorPolicy(self.p,[self.c])
        self.assertIn('INSUFFICIENT_WIDTH',self.evaluate().selection.reason)
    def test_stationary_goal_keeps_corridor_rotation_rule(self):
        self.r.x=1.;self.r.yaw=.2
        self.assertEqual(self.evaluate(((1.,0.),)).selection.reason,'ROTATION_FORBIDDEN_IN_NARROW_PASSAGE')
        self.r.yaw=0.
        self.assertEqual(self.evaluate(((1.,0.),)).failure,'CORRIDOR_GOAL_BEFORE_FULL_EXIT')
    def test_side_extension_rejected(self):
        self.p.footprint_xy=np.array([[-.32,-.32],[.32,-.32],[.32,.6],[-.32,.6]])
        self.assertIn('INSUFFICIENT_WIDTH',self.evaluate().selection.reason)

    def test_completed_entry_replaces_unused_approach_corner(self):
        self.p.footprint_xy=np.array([[-.32,-.2],[1.,-.2],[1.,.44],[-.32,.44]])
        self.r.x=-1.5;self.r.y=-.12;self.r.yaw=0.
        route=((-1.5,.15),(-.65,-.12),(1.,-.12),(4.,-.12))
        self.assertFalse(self.policy.route_fits(self.c,route))
        self.assertEqual(self.evaluate(route).state,'TRANSIT')
        # The same ownership does not permit a turn before the rear clears.
        self.p.footprint_xy=np.array([[-1.,-.2],[.32,-.2],[.32,.44],[-1.,.44]])
        bad_exit=((-1.5,.15),(-.65,-.12),(3.1,-.12),(3.1,1.))
        self.assertEqual(self.evaluate(bad_exit).failure,'CORRIDOR_ROUTE_TURN_UNREACHABLE')

    def test_rear_arm_must_leave_before_exit_turn(self):
        self.p.footprint_xy=np.array([[-1.,-.32],[.32,-.32],[.32,.32],[-1.,.32]])
        self.assertFalse(self.policy.route_fits(self.c,((-1.5,0.),(3.1,0.),(3.1,1.))))
        self.assertTrue(self.policy.route_fits(self.c,((-1.5,0.),(4.2,0.),(4.2,1.))))

    def test_impossible_arm_turn_reports_route_failure_without_waiting(self):
        self.p.footprint_xy=np.array([[-1.,-.32],[.32,-.32],[.32,.32],[-1.,.32]])
        self.r.x=-1.5
        result=self.evaluate(((-1.5,0.),(3.1,0.),(3.1,1.)))
        self.assertEqual(result.selection.motion,'HOLD')
        self.assertEqual(result.failure,'CORRIDOR_ROUTE_TURN_UNREACHABLE')

    def test_turn_sweep_not_just_endpoint_postures(self):
        self.p.footprint_xy=np.array([[-1.,-.32],[.32,-.32],[.32,.32],[-1.,.32]])
        margin=self.policy.margin(self.c)
        # The rear corner extends farther at an intermediate rotation than
        # at either the incoming or outgoing heading.
        turn_x=3.+margin+1.02
        self.assertFalse(self.policy.route_fits(self.c,((-1.5,0.),(turn_x,0.),(turn_x,-1.))))

    def test_front_arm_alignment_finishes_before_entry(self):
        self.p.footprint_xy=np.array([[-.32,-.32],[1.,-.32],[1.,.32],[-.32,.32]])
        self.assertFalse(self.policy.route_fits(self.c,((-.5,-1.),(-.5,0.),(4.,0.))))
        self.assertTrue(self.policy.route_fits(self.c,((-1.5,-1.),(-1.5,0.),(4.,0.))))

    def test_repeated_corner_endpoint_keeps_arm_clearance(self):
        self.p.footprint_xy=np.array([[-1.,-.32],[.32,-.32],[.32,.32],[-1.,.32]])
        self.assertFalse(self.policy.route_fits(self.c,((-1.5,0.),(3.5,0.),(3.5,0.),(3.5,1.))))

    def test_remote_turn_and_small_tracking_bend_still_allowed(self):
        self.assertTrue(self.policy.route_fits(self.c,((-2.,-2.),(-2.,0.),(4.,0.),(4.,1.))))
        wide=Corridor('wide',(0.,0.),(3.,0.),1.3,('unused_label',),.01,.01)
        self.assertTrue(self.policy.route_fits(wide,((-1.,0.),(1.,.005),(4.,0.))))

    def test_diagonal_remote_turn_is_not_a_swept_box_collision(self):
        self.p.footprint_xy=np.array([[-.32,-.32],[1.,-.32],[1.,.32],[-.32,.32]])
        self.assertTrue(self.policy.route_fits(self.c,
            ((-1.5,0.),(4.5,0.),(4.5,-1.3),(3.95,-1.3),(3.95,-1.2))))

class AdapterEntryTests(unittest.TestCase):
    """Execute the production advance body without starting ROS nodes."""
    def setUp(self):
        import ast
        from dataclasses import replace
        from astribot_s1_navigation_policy.corridor import CorridorPolicy,Passage
        source=Path(__file__).parents[1]/'astribot_s1_navigation_policy/corridor_adapter.py'
        tree=ast.parse(source.read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='CorridorAdapter')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='advance')
        namespace=dict(Time=lambda:None,replace=replace,Passage=Passage,np=np,math=math,
            yaw=lambda q:2*math.atan2(q.z,q.w))
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
        self.advance=namespace['advance'];self.legacy_class=CorridorPolicy

    def entry(self,side,*,fixed=True,mirror=False,sweep_clear=True,held_target=None,valid=True,repeats=1,
              translation=False,immediate=False,uncertain=False,previous_failure=False):
        import sys
        from unittest.mock import Mock,patch
        from astribot_s1_navigation_policy.robot_envelope import FIELDS
        baseline=Profile.load(Path(__file__).parents[1]/'config/simulation.json')
        profile=FixedEnvelopeProfile(baseline)
        limits={name:getattr(baseline,name) for name in FIELDS}
        profile.envelope=SimpleNamespace(**limits,posture_id='simulation_transport')
        profile.footprint_xy=np.array([[-.32,-.32],[.32,-.32],[.32,.95],[-.32,.95]])
        if mirror:profile.footprint_xy[:,1]*=-1
        corridor=Corridor('test',(0.,0.),(3.,0.),1.75,('simulation_transport',))
        version=SimpleNamespace(goal_id='goal',path_revision=1,map_epoch=1,
            localization_epoch=1,envelope_epoch=1,clock_epoch=1)
        policy=(FixedCorridorPolicy if fixed else self.legacy_class)(profile,[corridor])
        policy.active=corridor;policy.direction_allowed=True;policy.task=('goal',1)
        if previous_failure:policy.task=('previous',1);policy.failure='OLD_TASK_FAILURE';policy.wait_at=0.
        policy.entry_stopping=True;policy.centering_target=held_target
        target=policy.target_offset(corridor) if fixed else 0.
        node=SimpleNamespace(profile=profile,execution=SimpleNamespace(version=version),
            active_path_message=None,
            last_risk=SimpleNamespace(immediate=immediate,uncertain=uncertain),
            tf=SimpleNamespace(lookup_transform=lambda *args:None),
            point=lambda xyz,transform:xyz,path=((-1.,target),(4.,target)),
            last_robot=SimpleNamespace(x=-.8,y=side,yaw=0.,vx=0.,vy=0.,wz=0.),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:None)))
        if translation:
            points=((1.,target,0.),(.5,target,0.),(-1.2,target,0.),(-1.2,target,math.pi),(-1.5,target,math.pi))
            node.active_path_message=SimpleNamespace(header=SimpleNamespace(frame_id='map'),poses=[
                SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x,y=y),
                    orientation=SimpleNamespace(z=math.sin(h/2),w=math.cos(h/2)))) for x,y,h in points])
            node.last_robot.x=1.;node.last_robot.y=target
            node.path=tuple((x,y) for x,y,h in points)
            node.tf.lookup_transform=lambda *args:SimpleNamespace(transform=SimpleNamespace(
                rotation=SimpleNamespace(z=0.,w=1.)))
        rotation=Mock(side_effect=lambda c,t,target=None:True if target is None else sweep_clear)
        adapter=SimpleNamespace(node=node,clock_epoch=1,automatic=False,fixed=fixed,
            annotations=(corridor,),policy=policy,evidence={},
            geometry_clear=lambda *args:True,rotation_clear=rotation,
            fixed_sweep_clear=lambda *args:sweep_clear,
            assessment_pub=SimpleNamespace(publish=lambda msg:None))
        message_module=SimpleNamespace(PassageAssessment=lambda:SimpleNamespace(header=SimpleNamespace(),
            entry=SimpleNamespace(),exit=SimpleNamespace()))
        with patch.dict(sys.modules,{'astribot_navigation_msgs.msg':message_module}):
            for _ in range(repeats):
                result=self.advance(adapter,Selection('HOLD',0.,'YIELD') if translation else
                    Selection('PROCEED',.2,'CLEAR'),valid,1.)
        self.last_evidence=adapter.evidence
        return result,rotation

    def test_translation_owns_the_admitted_heading_instead_of_tangent_forecast(self):
        result,_=self.entry(0.,translation=True)
        self.assertEqual(result.state,'TRANSLATE')
        self.assertGreater(result.selection.speed,0.)
        self.assertEqual(result.tracking_heading,0.)

    def test_translation_keeps_live_sweep_current_collision_and_uncertainty_gates(self):
        for failure in ({'sweep_clear':False},{'immediate':True},{'uncertain':True},{'valid':False}):
            with self.subTest(failure=failure):
                result,_=self.entry(0.,translation=True,**failure)
                self.assertEqual(result.selection.motion,'HOLD')

    def test_new_translation_task_does_not_inherit_startup_failure(self):
        result,_=self.entry(0.,translation=True,previous_failure=True,valid=False)
        self.assertEqual(result.selection.reason,'CORRIDOR_INPUT_UNAVAILABLE')
        self.assertEqual(result.failure,'')

    def test_missing_input_heartbeat_evidence_stays_bounded(self):
        import json
        result,_=self.entry(-.42,valid=False,repeats=100)
        self.assertEqual(result.selection.reason,'CORRIDOR_INPUT_UNAVAILABLE')
        self.assertLess(len(json.dumps(self.last_evidence)),1000)
        self.assertEqual(self.last_evidence['transit'],{})
        self.assertEqual(self.last_evidence['alignment'],{})
        self.assertEqual(self.last_evidence['centering'],{})

    def test_asymmetric_entry_moves_105mm_left_and_right(self):
        for mirror in (False,True):
            with self.subTest(mirror=mirror):
                sign=-1 if mirror else 1
                result,rotation=self.entry(-.42*sign,mirror=mirror)
                self.assertEqual(result.state,'CENTER')
                self.assertAlmostEqual(result.centering_target[1],-.315*sign)
                self.assertAlmostEqual(abs(result.centering_target[1]+.42*sign),.105)
                self.assertEqual(rotation.call_count,2)

    def test_asymmetric_entry_over_300mm_is_rejected(self):
        for mirror in (False,True):
            with self.subTest(mirror=mirror):
                result,rotation=self.entry(.7 if mirror else -.7,mirror=mirror)
                self.assertEqual(result.selection.reason,'CORRIDOR_OFFSET_SWEEP_BLOCKED')
                self.assertEqual(rotation.call_count,1)

    def test_asymmetric_entry_failed_sweep_still_rejects(self):
        result,rotation=self.entry(-.42,sweep_clear=False)
        self.assertEqual(result.selection.reason,'CORRIDOR_OFFSET_SWEEP_BLOCKED')
        self.assertEqual(rotation.call_count,2)

    def test_retained_target_controls_requested_displacement(self):
        # A target already being followed must not be replaced by nominal offset.
        result,rotation=self.entry(-.42,held_target=(-.8,-.8))
        self.assertEqual(result.selection.reason,'CORRIDOR_OFFSET_SWEEP_BLOCKED')
        self.assertEqual(rotation.call_count,1)

    def test_legacy_center_uses_zero_target_and_same_limit(self):
        for side,allowed in ((.1,True),(-.1,True),(.4,False),(-.4,False)):
            with self.subTest(side=side):
                result,rotation=self.entry(side,fixed=False)
                self.assertEqual(result.state=='CENTER',allowed)
                self.assertEqual(rotation.call_count,2 if allowed else 1)
                if allowed:self.assertEqual(result.centering_target[1],0.)


if __name__=='__main__':unittest.main()
