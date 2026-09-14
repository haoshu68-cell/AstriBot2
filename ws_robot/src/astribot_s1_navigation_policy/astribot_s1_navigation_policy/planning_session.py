"""P3 request ownership and budgets. This module grants no motion authority."""
from dataclasses import dataclass
from threading import RLock

from .contracts import Planning, Stamp, Trigger, Version, finite, label, require


@dataclass(frozen=True)
class PlanningBudget:
    request_timeout_s: float
    episode_timeout_s: float
    max_requests_per_goal: int

    def __post_init__(self):
        for key in ('request_timeout_s', 'episode_timeout_s'):
            finite(getattr(self, key), key, 0)
            require(getattr(self, key) >= 1e-9, key)
        require(self.request_timeout_s <= self.episode_timeout_s, 'planning.timeout_order')
        require(type(self.max_requests_per_goal) is int and self.max_requests_per_goal > 0,
                'planning.max_requests_per_goal')


@dataclass(frozen=True)
class PlanningRequest:
    request_id: str
    version: Version
    episode: int
    observation_seq: int
    kinds: frozenset[Planning]
    trigger: Trigger
    issued_at: Stamp
    valid_until: Stamp

    def __post_init__(self):
        label(self.request_id, 'planning.request_id')
        require(isinstance(self.version, Version), 'planning.version')
        require(type(self.episode) is int and self.episode > 0, 'planning.episode')
        require(type(self.observation_seq) is int and self.observation_seq >= 0,
                'planning.observation_seq')
        require(type(self.kinds) is frozenset and bool(self.kinds) and
                all(isinstance(k, Planning) and k != Planning.NONE for k in self.kinds),
                'planning.kinds')
        require(isinstance(self.trigger, Trigger) and self.trigger != Trigger.NONE,
                'planning.event_required')
        require(isinstance(self.issued_at, Stamp) and isinstance(self.valid_until, Stamp),
                'planning.stamps')
        require(self.issued_at.clock == 'steady' and self.valid_until.since(self.issued_at) > 0,
                'planning.lease')


class PlanningBudgetExhausted(RuntimeError):
    pass


class PlanningSession:
    """One pending request per active goal; map refresh alone cannot request a plan.

    The adapter supplies a fresh, authoritative goal context. Matching a response
    only permits candidate inspection: geometry, time-space risk and takeover
    validation must still run before any path is committed.
    """
    def __init__(self, session_id: str, budget: PlanningBudget):
        label(session_id, 'planning.session_id')
        require(isinstance(budget, PlanningBudget), 'planning.budget')
        self.session_id = session_id
        self.budget = budget
        self._lock = RLock()
        self._version = None
        self._pending = None
        self._last_time = None
        self._blocked_at = None
        self._clock_fault = False
        self._serial = 0
        self._episode = 0
        self._attempts = 0

    def _advance(self, now):
        require(isinstance(now, Stamp) and now.clock == 'steady', 'planning.steady_clock')
        previous = self._last_time
        self._last_time = now
        if previous is not None and (now.epoch != previous.epoch or now.ns < previous.ns):
            self._pending = None
            self._clock_fault = True
        return not self._clock_fault

    def activate(self, version: Version, now: Stamp):
        require(isinstance(version, Version), 'planning.version')
        require(isinstance(now, Stamp) and now.clock == 'steady', 'planning.steady_clock')
        with self._lock:
            if self._version is None or version.goal_id != self._version.goal_id:
                self._pending = None
                self._blocked_at = None
                self._attempts = 0
                self._clock_fault = False
                self._last_time = now
            else:
                self._advance(now)
                if version != self._version:
                    self._pending = None
            self._version = version

    def finish(self, goal_id: str):
        with self._lock:
            if self._version is not None and self._version.goal_id == goal_id:
                self._pending = None
                self._version = None
                self._blocked_at = None

    def clear_blockage(self, now: Stamp):
        with self._lock:
            if self._advance(now):
                self._pending = None
                self._blocked_at = None
            # Repeated clear/reblock events never reset the per-goal attempt budget.

    def request(self, version: Version, trigger: Trigger, kinds: frozenset[Planning],
                observation_seq: int, now: Stamp):
        require(isinstance(version, Version), 'planning.version')
        require(isinstance(trigger, Trigger) and trigger != Trigger.NONE, 'planning.event_required')
        require(type(kinds) is frozenset and bool(kinds) and
                all(isinstance(k, Planning) and k != Planning.NONE for k in kinds), 'planning.kinds')
        require(type(observation_seq) is int and observation_seq >= 0, 'planning.observation_seq')
        with self._lock:
            require(version == self._version, 'planning.active_version')
            require(self._advance(now), 'planning.clock_reset_requires_new_goal')
            if self._blocked_at is None:
                self._blocked_at = now
                self._episode += 1
            episode_end = self._blocked_at.ns + int(self.budget.episode_timeout_s * 1e9)
            if now.ns >= episode_end:
                self._pending = None
                raise PlanningBudgetExhausted('TEMPORARILY_BLOCKED: episode deadline')
            if self._pending is not None and now.ns < self._pending.valid_until.ns:
                return self._pending
            self._pending = None
            if self._attempts >= self.budget.max_requests_per_goal:
                raise PlanningBudgetExhausted('TEMPORARILY_BLOCKED: per-goal planning attempts')
            request = PlanningRequest(
                f'{self.session_id}:{self._serial + 1}', version, self._episode,
                observation_seq, kinds, trigger, now,
                Stamp(min(episode_end, now.ns + int(self.budget.request_timeout_s * 1e9)),
                      now.clock, now.epoch))
            self._serial += 1
            self._attempts += 1
            self._pending = request
            return request

    def response_current(self, request: PlanningRequest, version: Version, now: Stamp):
        if not isinstance(request, PlanningRequest) or not isinstance(version, Version):
            return False
        with self._lock:
            return (self._advance(now) and self._version is not None and
                    request == self._pending and request.version == version == self._version and
                    now.epoch == request.issued_at.epoch and
                    request.issued_at.ns <= now.ns < request.valid_until.ns)

    def failure_reason(self, now: Stamp):
        with self._lock:
            if not self._advance(now):
                return 'CLOCK_MISMATCH'
            if self._version is None or self._blocked_at is None:
                return None
            if now.ns - self._blocked_at.ns >= int(self.budget.episode_timeout_s * 1e9):
                self._pending = None
                return 'TEMPORARILY_BLOCKED: episode deadline'
            if (self._attempts >= self.budget.max_requests_per_goal and
                    (self._pending is None or now.ns >= self._pending.valid_until.ns)):
                self._pending = None
                return 'TEMPORARILY_BLOCKED: per-goal planning attempts'
            return None

    def retire(self, request: PlanningRequest):
        with self._lock:
            if self._pending == request:
                self._pending = None
