"""A completed FK job must retain its source lease and input identity."""
from pathlib import Path as _ReferencePath
import sys as _reference_sys
_reference_sys.path.insert(0, str(_ReferencePath(__file__).resolve().parents[4] / "tools/migration/python_reference"))
from reference_bootstrap import enable as _enable_references
_enable_references()

from concurrent.futures import Future
from types import SimpleNamespace as N
import unittest
from rclpy.time import Time
from astribot_s1_robot_geometry.node import GeometryNode, stamp
from astribot_navigation_msgs.msg import RobotGeometryState


class AsyncSourceTests(unittest.TestCase):
    def node(self,now=1_150_000_000,generation=0):
        msg=RobotGeometryState(height_m=1.7)
        msg.header.stamp=stamp(1_000_000_000);msg.valid_until=stamp(1_300_000_000)
        work=Future();work.set_result(msg);model=object();self.sent=[]
        n=N(get_clock=lambda:N(now=lambda:Time(nanoseconds=now)),refresh_inputs=lambda _:None,
            work=work,work_context=(0,model,'a',0),input_generation=generation,model=model,
            attachments=[],attachment_revision='a',samples=N(epoch=0),coverage_max=2.2,
            coverage_at=1_000_000_000,pub=N(publish=self.sent.append),next_sample=float('inf'),
            frame='base',sequence=1,source='test',filter_revision='a',filter_at=float('inf'))
        n.new_message=lambda now:GeometryNode.new_message(n,now)
        return n

    def test_fresh_result_does_not_restamp_measurement_or_extend_lease(self):
        n=self.node();GeometryNode.tick(n)
        self.assertTrue(self.sent[-1].complete)
        self.assertEqual(self.sent[-1].header.stamp,stamp(1_000_000_000))
        self.assertEqual(self.sent[-1].valid_until,stamp(1_300_000_000))
        self.assertEqual(self.sent[-1].published_at,stamp(1_150_000_000))

    def test_delayed_result_and_clock_rollback_are_incomplete(self):
        for now in (1_300_000_000,900_000_000):
            n=self.node(now);GeometryNode.tick(n)
            self.assertFalse(self.sent[-1].complete)
            self.assertIn('SOURCE_LEASE',self.sent[-1].reason)

    def test_invalidated_input_cannot_publish_complete_old_job(self):
        n=self.node(generation=1);GeometryNode.tick(n)
        self.assertFalse(self.sent[-1].complete)
        self.assertEqual(self.sent[-1].reason,'GEOMETRY_INPUT_CHANGED_DURING_COMPUTE')

    def test_lost_read_response_is_bounded_without_refreshing_attachment_evidence(self):
        future=Future();removed=[]
        client=N(service_is_ready=lambda:False,remove_pending_request=removed.append)
        n=N(params_future=None,params=client,last_params=-1.,scene_future=future,
            scene=client,last_scene=-1.,coverage_future=None,coverage=client,
            last_coverage=-1.,attachment_stamp=123,attachment_revision='unchanged')
        GeometryNode.refresh_inputs(n,2_000_000_000)
        self.assertTrue(future.cancelled());self.assertEqual(removed,[future])
        self.assertIsNone(n.scene_future)
        self.assertEqual(n.attachment_stamp,123)
        self.assertEqual(n.attachment_revision,'unchanged')


if __name__=='__main__':unittest.main()
