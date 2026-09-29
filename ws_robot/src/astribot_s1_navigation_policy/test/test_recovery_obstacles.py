import unittest
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
from astribot_navigation_msgs.srv import GetRecoveryObstacles
from astribot_s1_navigation_policy.policy_node import PolicyNode
from astribot_s1_navigation_policy.world_geometry import prediction_rows
from test_prediction_rows import sample_world

class RecoveryObstaclesTests(unittest.TestCase):
    def query(self, world, valid=True, epoch=1):
        node=SimpleNamespace(last_world=world,last_inputs_valid=valid,
                             last_evaluation_epoch=1,epoch=epoch)
        return PolicyNode.recovery_obstacles(node,GetRecoveryObstacles.Request(),GetRecoveryObstacles.Response())
    def test_static_explicit_and_parametric_predictions_are_enclosed(self):
        world=sample_world();result=self.query(world)
        self.assertTrue(result.valid,result.reason)
        self.assertEqual(result.header.frame_id,'odom')
        self.assertEqual(result.header.stamp.sec,1)
        self.assertEqual(len(result.obstacles),len(world.tracks))
        rows=prediction_rows(world,include_current=True,swept=True)
        for owner, polygon in enumerate(result.obstacles):
            points=np.array([(p.x,p.y) for p in polygon.points])
            select=rows.owners==owner
            np.testing.assert_allclose(points.min(axis=0),rows.lower[select].min(axis=0),atol=1e-6)
            np.testing.assert_allclose(points.max(axis=0),rows.upper[select].max(axis=0),atol=1e-6)
    def test_valid_empty_world_is_clear(self):
        result=self.query(replace(sample_world(),tracks=()))
        self.assertTrue(result.valid);self.assertEqual(result.obstacles,[])
    def test_missing_invalid_epoch_and_nonmetric_cannot_certify_clear(self):
        for world,valid,epoch in ((None,True,1),(sample_world(),False,1),(sample_world(),True,2)):
            self.assertFalse(self.query(world,valid,epoch).valid)
        world=SimpleNamespace(unassociated=(object(),))
        result=self.query(world)
        self.assertFalse(result.valid);self.assertEqual(result.reason,'RECOVERY_NONMETRIC_OBSTACLES')

if __name__=='__main__':unittest.main()
