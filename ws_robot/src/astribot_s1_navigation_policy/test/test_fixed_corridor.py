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
        self.assertEqual(self.evaluate(((1.,0.),)).state,'TRANSIT')
    def test_side_extension_rejected(self):
        self.p.footprint_xy=np.array([[-.32,-.32],[.32,-.32],[.32,.6],[-.32,.6]])
        self.assertIn('INSUFFICIENT_WIDTH',self.evaluate().selection.reason)

if __name__=='__main__':unittest.main()
