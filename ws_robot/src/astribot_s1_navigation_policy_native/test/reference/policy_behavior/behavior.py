"""Episode-based behavior selection, independent of ROS and controller internals."""
from dataclasses import dataclass
import math
import os


_native = None
if os.environ.get('ASTRIBOT_NAV_NATIVE_KERNELS', '').lower() in ('1', 'true', 'yes'):
    try:
        from astribot_s1_navigation_policy_native import _navigation_math_native as _native
    except ImportError:  # pragma: no cover - Python-only overlays remain supported
        _native = None

@dataclass(frozen=True)
class Selection:
    motion: str
    speed: float
    reason: str
    planning: int = 0
    episode: int = 0

def requires_stop(risk, profile):
    if (_native is not None and risk is not None and
            all(hasattr(risk, name) for name in
                ('immediate', 'uncertain', 'blocked', 'conflict_time_s'))):
        return bool(_native.yield_requires_stop(
            bool(risk.immediate), bool(risk.uncertain), bool(risk.blocked),
            float(risk.conflict_time_s), float(profile.max_speed_m_s),
            float(profile.reaction_time_s),
            float(profile.brake_deceleration_m_s2),
            float(getattr(profile, 'linear_stop_delay_s', 0.0))))
    from .motion_geometry import stopping_horizon
    stop_time=stopping_horizon((profile.max_speed_m_s,0.,0.),profile)
    return (risk.immediate or risk.uncertain or
            (risk.blocked and not risk.conflict_time_s>stop_time+.5))

class YieldPolicy:
    def __init__(self, profile):
        self.profile=profile
        self.blocked_at=None;self.clear_at=None;self.last_time=None
        self.held=True;self.episode=0;self.in_episode=False
        try:
            self._native_policy = (_native.YieldPolicy(
                float(profile.max_speed_m_s), float(profile.narrow_speed_m_s),
                float(profile.reaction_time_s),
                float(profile.brake_deceleration_m_s2),
                float(getattr(profile, 'linear_stop_delay_s', 0.0)),
                float(profile.wait_budget_s), float(profile.clear_hold_s))
                if _native is not None else None)
        except (AttributeError, TypeError, ValueError):
            self._native_policy = None

    def select(self, risk, valid, now):
        p=self.profile
        if not math.isfinite(now):raise ValueError('finite policy time required')
        if (self._native_policy is not None and
                (risk is None or all(hasattr(risk, name) for name in
                                     ('immediate', 'blocked', 'uncertain',
                                      'conflict_time_s')))):
            immediate = bool(getattr(risk, 'immediate', False)) if risk is not None else False
            blocked = bool(getattr(risk, 'blocked', False)) if risk is not None else False
            uncertain = bool(getattr(risk, 'uncertain', False)) if risk is not None else False
            conflict = (float(getattr(risk, 'conflict_time_s', 0.0))
                        if risk is not None else 0.0)
            result = self._native_policy.select(
                immediate, blocked, uncertain, conflict,
                bool(valid and risk is not None), float(now))
            self._sync_native_state()
            return Selection(result.motion, float(result.speed), result.reason,
                             episode=int(result.episode))
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

    def _sync_native_state(self):
        blocked, blocked_at, clear, clear_at, last, last_time, held, episode, in_episode = \
            self._native_policy.state()
        self.blocked_at = float(blocked_at) if blocked else None
        self.clear_at = float(clear_at) if clear else None
        self.last_time = float(last_time) if last else None
        self.held = bool(held)
        self.episode = int(episode)
        self.in_episode = bool(in_episode)
