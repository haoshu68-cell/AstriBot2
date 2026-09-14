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

class YieldPolicy:
    def __init__(self, profile):
        self.profile=profile
        self.blocked_at=None;self.clear_at=None;self.last_time=None
        self.held=True;self.episode=0

    def select(self, risk, valid, now):
        p=self.profile
        if not math.isfinite(now):raise ValueError('finite policy time required')
        if self.last_time is not None and now<self.last_time:
            self.blocked_at=None;self.clear_at=None;self.held=True
        self.last_time=now
        if not valid or risk is None:
            self.held=True;self.clear_at=None
            return Selection('HOLD',0.,'INPUT_UNAVAILABLE',episode=self.episode)
        if risk.immediate or risk.blocked:
            self.clear_at=None
            if self.blocked_at is None:self.blocked_at=now;self.episode+=1
            stop_time=p.reaction_time_s+p.max_speed_m_s/p.brake_deceleration_m_s2
            if not self.held and not risk.immediate and not risk.uncertain and risk.conflict_time_s>stop_time+.5:
                return Selection('SLOW',p.narrow_speed_m_s,'PREDICTED_CONFLICT',episode=self.episode)
            self.held=True
            reason='UNKNOWN_GEOMETRY' if risk.uncertain else ('IMMEDIATE_RISK' if risk.immediate else 'YIELD')
            if now-self.blocked_at>p.wait_budget_s:reason='BLOCKED_CAPABILITY_NOT_ENABLED'
            return Selection('HOLD',0.,reason,episode=self.episode)
        if self.held:
            if self.clear_at is None:self.clear_at=now
            if now-self.clear_at<p.clear_hold_s:return Selection('HOLD',0.,'CLEAR_CONFIRMATION',episode=self.episode)
        self.held=False;self.blocked_at=None
        return Selection('CONTINUE',p.max_speed_m_s,'CLEAR',episode=self.episode)
