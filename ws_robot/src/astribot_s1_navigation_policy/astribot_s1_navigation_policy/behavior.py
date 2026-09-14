"""Episode-based behavior selection, independent of ROS and controller internals."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class Selection:
    motion: str
    speed: float
    reason: str
    planning: int = 0
    episode: int = 0

def requires_stop(risk, profile):
    stop_time=profile.reaction_time_s+profile.max_speed_m_s/profile.brake_deceleration_m_s2
    return (risk.immediate or risk.uncertain or
            (risk.blocked and not risk.conflict_time_s>stop_time+.5))

class YieldPolicy:
    def __init__(self, profile):
        self.profile=profile
        self.blocked_at=None;self.clear_at=None;self.last_time=None
        self.held=True;self.episode=0;self.in_episode=False

    def select(self, risk, valid, now):
        p=self.profile
        if not math.isfinite(now):raise ValueError('finite policy time required')
        if self.last_time is not None and now<self.last_time:
            self.blocked_at=None;self.clear_at=None;self.held=True;self.in_episode=False
        self.last_time=now
        if not valid or risk is None:
            self.held=True;self.clear_at=None
            return Selection('HOLD',0.,'INPUT_UNAVAILABLE',episode=self.episode)
        obstructed=risk.immediate or risk.blocked or risk.uncertain
        if obstructed:
            if not self.in_episode:self.episode+=1;self.in_episode=True
            # A confirmed distant conflict permits cautious recovery. Near or
            # unresolved risk restarts the same clear-confirmation interval.
            must_stop=requires_stop(risk,p)
            if must_stop:
                if self.blocked_at is None:self.blocked_at=now
                self.clear_at=None;self.held=True
                reason='UNKNOWN_GEOMETRY' if risk.uncertain else ('IMMEDIATE_RISK' if risk.immediate else 'YIELD')
                if now-self.blocked_at>p.wait_budget_s:reason='BLOCKED_CAPABILITY_NOT_ENABLED'
                return Selection('HOLD',0.,reason,episode=self.episode)
        if self.held:
            if self.clear_at is None:self.clear_at=now
            if now-self.clear_at<p.clear_hold_s:return Selection('HOLD',0.,'CLEAR_CONFIRMATION',episode=self.episode)
        self.held=False;self.clear_at=None
        if obstructed:
            self.blocked_at=None
            return Selection('SLOW',p.narrow_speed_m_s,'PREDICTED_CONFLICT',episode=self.episode)
        self.blocked_at=None;self.in_episode=False
        return Selection('CONTINUE',p.max_speed_m_s,'CLEAR',episode=self.episode)
