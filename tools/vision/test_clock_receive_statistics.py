import unittest
from clock_receive_statistics import ClockReceiveStatistics


class ClockReceiveStatisticsTest(unittest.TestCase):
    def test_empty_does_not_claim_clock_progress(self):
        result = ClockReceiveStatistics().summary()
        self.assertEqual(result['samples'], 0)
        self.assertEqual(result['advanced_sec'], 0.)

    def test_wall_stall_is_visible_even_when_ros_time_barely_moves(self):
        clock = ClockReceiveStatistics()
        clock.observe(100_000_000_000, 10.)
        clock.observe(100_001_000_000, 10.001)
        clock.observe(100_002_000_000, 10.401)
        result = clock.summary()
        self.assertEqual(result['samples'], 3)
        self.assertAlmostEqual(result['advanced_sec'], .002)
        self.assertAlmostEqual(result['max_receive_interval_wall_sec'], .4)
        self.assertEqual(result['receive_gaps_over_250ms'], 1)
        event = result['gap_events'][0]
        self.assertEqual(event['previous_source_ns'], 100_001_000_000)
        self.assertEqual(event['source_ns'], 100_002_000_000)

    def test_duplicates_and_clock_rollback_are_separate(self):
        clock = ClockReceiveStatistics()
        for stamp, wall in [(100, 1.), (100, 1.1), (50, 1.2), (60, 1.3)]:
            clock.observe(stamp, wall)
        result = clock.summary()
        self.assertEqual(result['duplicate_source_intervals'], 1)
        self.assertEqual(result['source_regressions'], 1)
        self.assertLess(result['advanced_sec'], 0.)

    def test_gap_details_are_bounded_without_losing_total_count(self):
        clock = ClockReceiveStatistics()
        for index in range(200):
            clock.observe(index * 1_000_000, index * .3)
        result = clock.summary()
        self.assertEqual(result['receive_gaps_over_250ms'], 199)
        self.assertEqual(len(result['gap_events']), 128)
        self.assertEqual(result['gap_events_omitted'], 71)

    def test_terminal_silence_is_reported_without_a_recovery_message(self):
        clock = ClockReceiveStatistics()
        self.assertIsNone(clock.summary(at_monotonic=10.)['tail_silence_wall_sec'])
        clock.observe(1_000_000_000, 10.)
        result = clock.summary(at_monotonic=10.4)
        self.assertAlmostEqual(result['tail_silence_wall_sec'], .4)
        self.assertEqual(result['receive_gaps_over_250ms'], 0)


if __name__ == '__main__':
    unittest.main()
