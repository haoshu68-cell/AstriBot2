"""Offline wiring guards; no ROS process or simulated time is started.

Behavioral deadline/rounding cases live in publication_lease_test.cpp. These
checks keep navigation constraints bounded by source evidence after expensive
sweep calculations and ensure this node has no command publishing authority.
"""
from pathlib import Path
import unittest

SOURCE = Path(__file__).resolve().parents[1] / 'src/navigation_constraint_node.cpp'

class NavigationConstraintWiring(unittest.TestCase):
    def test_constraint_uses_actual_anchor_and_remaining_lease(self):
        source = SOURCE.read_text()
        self.assertIn('const auto publication_anchor = now();', source)
        self.assertIn('c.stamp = publication_anchor;', source)
        self.assertIn('stop ? hold_lease_s : publication.lease_s', source)
        self.assertNotIn('c.stamp = ros;', source)
        self.assertLess(source.index('protection_swept_collision(points_'),
                        source.index('const auto publication_anchor = now();'))
        self.assertLess(source.index('const auto publication_anchor = now();'),
                        source.index('constraint_->publish(c)'))

    def test_expired_publication_stops_same_tick_and_revokes_all_motion_flags(self):
        source = SOURCE.read_text()
        self.assertIn('if (!publication.allowed)', source)
        block = source.split('if (!publication.allowed)', 1)[1].split('\n    }', 1)[0]
        self.assertIn('stop = true;', block)
        self.assertIn('clear_at_.reset();', block)
        for field in ('planning', 'alignment_required', 'centering_required', 'corridor_tracking_required'):
            line = next(line for line in source.splitlines() if 'c.' + field + ' =' in line)
            self.assertIn('!stop &&', line)

    def test_wire_diagnostic_identifies_upstream_and_effective_publication(self):
        source = SOURCE.read_text()
        for key in ('proposal_epoch', 'proposal_sequence', 'proposal_stamp_ns',
                    'proposal_deadline_ns', 'publication_stamp_ns', 'effective_lease_s',
                    'publication_lease_reason', 'constraint_publish_start_ns'):
            self.assertIn('"' + key + '"', source)
        self.assertIn('if (timing_diagnostics_ || sequence_ % 5 == 0)', source)

    def test_node_only_publishes_constraints_diagnostics_and_optional_zone_ack(self):
        source = SOURCE.read_text()
        self.assertNotIn('create_publisher<Twist>', source)
        self.assertNotIn('Publisher<Twist>', source)
        self.assertNotIn('CommandRestriction', source)
        self.assertNotIn('EnvelopeApplyStatus', source)
        self.assertNotIn('/navigation/envelope_applied', source)
        self.assertNotIn('"output_speed_m_s"', source)
        self.assertNotIn('"output_publish_start_ns"', source)
        self.assertIn('"requested_command_speed_m_s"', source)
        self.assertIn('"command_topic","/cmd_vel_nav_body_raw"', source)
        self.assertIn('command_descriptor.read_only=true', source)
        self.assertIn('"/navigation_policy/constraint_state"', source)
        self.assertIn('acknowledgement(*snapshot,"navigation_constraint"', source)

if __name__ == '__main__':
    unittest.main()
