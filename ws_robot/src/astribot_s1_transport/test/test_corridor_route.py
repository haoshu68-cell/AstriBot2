import math
import unittest
import xml.etree.ElementTree as ET
from astribot_s1_transport.corridor_route import CorridorRoute,TraversalWitness


class CorridorTests(unittest.TestCase):
    def route(self):return CorridorRoute('fixture','map',(.6,0.),(-.6,0.),1.3)

    def test_intent_retains_ordered_inside_goals_and_final_outside_goal(self):
        route=self.route();goals=route.via_poses()
        self.assertEqual(goals[0][0],.6)
        self.assertAlmostEqual(goals[-2][0],-.6)
        self.assertAlmostEqual(goals[-1][0],-1.4)
        self.assertTrue(all(a[0]>b[0] for a,b in zip(goals,goals[1:])))
        tree=ET.fromstring(route.behavior_tree())
        nodes=[n.tag for n in tree.find('.//ReactiveSequence')]
        self.assertEqual(nodes,['ReactiveFallback','RequireCorridorRoute','RemovePassedGoals','FollowPath'])
        self.assertEqual(tree.find('.//RemovePassedGoals').attrib['radius'],'0.10')
        self.assertIsNotNone(tree.find('.//PolicyExecution'))

    def test_actual_crossing_requires_both_gates_and_contiguous_source_samples(self):
        route=self.route();witness=TraversalWitness(route)
        for i in range(141):witness.observe(1+i*.1,1.4-i*.02,0.)
        result=witness.result()
        self.assertTrue(result['entered'] and result['exited'])
        self.assertEqual(result['samples'],141)

    def test_arrival_without_crossing_is_not_a_pass(self):
        witness=TraversalWitness(self.route())
        witness.observe(1.,1.4,0.)
        with self.assertRaisesRegex(ValueError,'NOT_WITNESSED'):witness.result()
        with self.assertRaisesRegex(ValueError,'POSE_JUMP'):witness.observe(1.1,-1.4,0.)
        with self.assertRaisesRegex(ValueError,'MISSING_ENTRY'):TraversalWitness(self.route()).observe(1.,0.,0.)

    def test_side_detour_is_rejected(self):
        witness=TraversalWitness(self.route())
        witness.observe(1.,.62,.8)
        with self.assertRaisesRegex(ValueError,'BYPASSES_CORRIDOR'):witness.observe(1.1,.60,.8)

    def test_repeated_stamp_cannot_hide_dropout_or_clock_rollback(self):
        for following in (.9,1.31):
            witness=TraversalWitness(self.route())
            witness.observe(1.,1.4,0.);witness.observe(1.,1.4,0.)
            self.assertEqual(witness.samples,1)
            with self.assertRaisesRegex(ValueError,'TIME_GAP'):witness.observe(following,1.4,0.)

    def test_bad_intent_and_pose_rejected(self):
        for width in (0.,-1.,float('nan')):
            with self.assertRaisesRegex(ValueError,'INVALID_CORRIDOR_INTENT'):
                CorridorRoute('test','map',(0.,0.),(1.,0.),width)
        with self.assertRaisesRegex(ValueError,'INVALID'):
            TraversalWitness(self.route()).observe(1.,float('nan'),0.)

    def test_same_source_stamp_cannot_describe_two_poses(self):
        witness=TraversalWitness(self.route());witness.observe(1.,1.4,0.)
        with self.assertRaisesRegex(ValueError,'CONFLICTING_SAMPLE'):witness.observe(1.,1.39,0.)

    def test_between_sample_side_entry_is_rejected(self):
        witness=TraversalWitness(self.route());witness.observe(1.,.61,.66)
        with self.assertRaisesRegex(ValueError,'BYPASSES_CORRIDOR'):witness.observe(1.1,.58,.64)


if __name__=='__main__':unittest.main()
