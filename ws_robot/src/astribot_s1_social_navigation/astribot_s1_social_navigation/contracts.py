"""Validation independent of ROS executors and transport. SI units throughout."""
import math
import numpy as np


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def covariance_valid(values):
    matrix = np.asarray(values, dtype=float).reshape(6, 6)
    return (np.isfinite(matrix).all() and np.allclose(matrix, matrix.T, atol=1e-9)
            and np.linalg.eigvalsh(matrix).min() >= -1e-9)


def validate_sample(message, now_ns, timeout_s, allow_truth=False):
    if not message.valid:
        return message.reason or 'SOURCE_INVALID'
    if not message.header.frame_id or not message.source_id:
        return 'MISSING_FRAME_OR_SOURCE'
    if message.provenance not in (0, 1) or (message.provenance == 1 and not allow_truth):
        return 'TRUTH_NOT_ALLOWED'
    measured = stamp_ns(message.header.stamp)
    age = (now_ns - measured) * 1e-9
    if measured <= 0 or age < 0 or age > timeout_s:
        return 'SAMPLE_TIME_INVALID'
    ids = set()
    for agent in message.agents:
        if not agent.track_id or agent.track_id in ids:
            return 'DUPLICATE_OR_EMPTY_ID'
        ids.add(agent.track_id)
        observed = stamp_ns(agent.last_observed)
        if observed <= 0 or observed > measured:
            return 'OBSERVATION_TIME_INVALID'
        if agent.tracking_state not in (0, 1, 2):
            return 'TRACK_STATE_INVALID'
        if agent.tracking_state != 2 and (now_ns - observed) * 1e-9 > timeout_s:
            return 'TRACK_EXPIRED'
        p, q, v, w = agent.pose.pose.position, agent.pose.pose.orientation, agent.velocity.twist.linear, agent.velocity.twist.angular
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w, agent.radius,
                  agent.person_confidence, agent.heading_confidence, agent.group_confidence)
        if not all(math.isfinite(x) for x in values) or agent.radius <= 0:
            return 'GEOMETRY_INVALID'
        if any(not 0 <= x <= 1 for x in (agent.person_confidence, agent.heading_confidence, agent.group_confidence)):
            return 'CONFIDENCE_INVALID'
        if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 1e-3:
            return 'ORIENTATION_INVALID'
        if not covariance_valid(agent.pose.covariance):
            return 'POSE_COVARIANCE_INVALID'
        if agent.velocity_valid and (not all(math.isfinite(x) for x in (v.x, v.y, v.z, w.x, w.y, w.z))
                                     or not covariance_valid(agent.velocity.covariance)):
            return 'VELOCITY_INVALID'
    return ''


class SampleOrder:
    """A source restart requires a monotonically increasing source epoch."""
    def __init__(self):
        self.previous = None

    def accept(self, message):
        key = (message.source_id, message.source_epoch, message.sequence,
               stamp_ns(message.header.stamp))
        previous = self.previous
        if previous is not None:
            if key[0] != previous[0]:
                return 'SOURCE_CHANGED'
            if key[1] < previous[1]:
                return 'SOURCE_EPOCH_REGRESSED'
            if key[1] == previous[1] and (key[2] <= previous[2] or key[3] <= previous[3]):
                return 'DUPLICATE_OR_OUT_OF_ORDER'
        self.previous = key
        return ''
