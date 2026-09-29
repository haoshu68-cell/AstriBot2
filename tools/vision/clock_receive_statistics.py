"""Bounded clock observations for validation; never adjusts sensor timestamps."""
from collections import deque


class ClockReceiveStatistics:
    def __init__(self):
        self.samples = 0
        self.first_source_ns = self.last_source_ns = None
        self.last_receive = None
        self.max_receive_gap = 0.
        self.gap_count = self.duplicates = self.regressions = 0
        self.gap_events = deque(maxlen=128)

    def observe(self, source_ns, receive_monotonic):
        if self.samples:
            gap = receive_monotonic - self.last_receive
            self.max_receive_gap = max(self.max_receive_gap, gap)
            self.duplicates += source_ns == self.last_source_ns
            self.regressions += source_ns < self.last_source_ns
            if gap > .25:
                self.gap_count += 1
                self.gap_events.append({
                    'previous_receive_monotonic_sec': self.last_receive,
                    'receive_monotonic_sec': receive_monotonic,
                    'interval_wall_sec': gap,
                    'previous_source_ns': self.last_source_ns,
                    'source_ns': source_ns,
                })
        else:
            self.first_source_ns = source_ns
        self.last_source_ns = source_ns
        self.last_receive = receive_monotonic
        self.samples += 1

    def summary(self, at_monotonic=None):
        return {
            'scope': 'Observer clock receive gaps, not Gazebo server execution timing',
            'samples': self.samples,
            'advanced_sec': ((self.last_source_ns - self.first_source_ns) * 1e-9
                             if self.samples else 0.),
            'first_source_ns': self.first_source_ns,
            'last_source_ns': self.last_source_ns,
            'tail_silence_wall_sec': (max(0., at_monotonic - self.last_receive)
                                      if at_monotonic is not None and self.samples else None),
            'max_receive_interval_wall_sec': self.max_receive_gap,
            'receive_gaps_over_250ms': self.gap_count,
            'duplicate_source_intervals': self.duplicates,
            'source_regressions': self.regressions,
            'gap_events': list(self.gap_events),
            'gap_events_omitted': self.gap_count - len(self.gap_events),
        }
