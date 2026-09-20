import unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch
from rclpy.time import Time
from astribot_navigation_msgs.msg import StartManeuverRequest,StartManeuver
from astribot_s1_navigation_policy.start_maneuver_adapter import StartManeuverAdapter
from astribot_s1_navigation_policy.profile import Profile
from astribot_s1_navigation_policy.behavior import Selection


class RequestBoundaryTests(unittest.TestCase):
    def test_new_request_wait_does_not_inherit_old_timeout(self):
        sent=[]
        profile=Profile.load(Path(__file__).parents[1]/'config/simulation.json')
        node=N(profile=profile,active_path_key='old-path',
            stamp=lambda:N(ns=1000_000_000_000),
            get_clock=lambda:N(now=lambda:Time(seconds=1000.)),
            create_subscription=lambda *a:None,
            create_publisher=lambda *a:N(publish=sent.append))
        adapter=StartManeuverAdapter(node)
        adapter.request_id=1;adapter.policy.started=1.
        adapter.policy.fail('MANEUVER_TIMEOUT')
        request=StartManeuverRequest(request_id=2)
        request.stamp=Time(seconds=1000.).to_msg()
        adapter.receive(request)
        with patch('astribot_s1_navigation_policy.start_maneuver_adapter.path_identity',return_value='new-path'):
            adapter.advance(Selection('PROCEED',.2,'CLEAR'),True)
            self.assertEqual(sent[-1].mode,StartManeuver.WAIT)
            self.assertEqual(sent[-1].reason,'START_MANEUVER_ACTIVE_PATH_PENDING')
            self.assertAlmostEqual(adapter.policy.started,1000.)
            # Repeating the same request cannot clear its own terminal fault.
            adapter.policy.fail('REVERSE_EXIT_NO_PROGRESS')
            adapter.advance(Selection('PROCEED',.2,'CLEAR'),True)
            self.assertEqual(sent[-1].mode,StartManeuver.FAILED)
            self.assertIn('REVERSE_EXIT_NO_PROGRESS',sent[-1].reason)


if __name__=='__main__':unittest.main()
