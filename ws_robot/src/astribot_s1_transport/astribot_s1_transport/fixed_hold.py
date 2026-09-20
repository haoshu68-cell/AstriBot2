"""Measured settling before committing a fixed navigation posture."""
from collections import deque
import math


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


class StableGeometryReference:
    def __init__(self, after_ns, duration_ns=500_000_000):
        self.after_ns = after_ns
        self.duration_ns = duration_ns
        self.samples = deque()
        self.identity = None

    def observe(self, state, now_ns):
        if (state is None or not state.complete or not state.attachment_state_confirmed or
                not self.after_ns < ns(state.header.stamp) <= now_ns < ns(state.valid_until)):
            self.samples.clear()
            return False
        names = tuple(state.joints.name)
        identity = (state.source_id, state.clock_epoch, state.model_revision,
                    state.attachment_revision, names, tuple(state.joint_position_error_bounds))
        values = tuple(state.joints.position)
        if (not names or len(values) != len(names) or len(identity[-1]) != len(names) or
                not all(math.isfinite(q) for q in values) or
                not all(math.isfinite(e) and e > 0 for e in identity[-1])):
            self.samples.clear()
            return False
        if identity != self.identity:
            self.samples.clear()
            self.identity = identity
        stamp = ns(state.header.stamp)
        if self.samples and stamp <= self.samples[-1][0]:
            return False
        self.samples.append((stamp, values))
        while len(self.samples) > 2 and stamp - self.samples[1][0] >= self.duration_ns:
            self.samples.popleft()
        if len(self.samples) < 3 or stamp - self.samples[0][0] < self.duration_ns:
            return False
        # Settling consumes at most a quarter of the existing hold error budget;
        # it does not enlarge that budget or replace the ongoing drift check.
        return all(max(q[i] for _, q in self.samples) - min(q[i] for _, q in self.samples)
                   <= error / 4 for i, error in enumerate(identity[-1]))
