"""Physical-time freshness with an independent simulation-clock watchdog."""
from dataclasses import dataclass
import math
import os


_native_time = None
if os.environ.get('ASTRIBOT_NAV_NATIVE_KERNELS', '').lower() in ('1', 'true', 'yes'):
    try:
        from astribot_s1_navigation_policy_native import _navigation_math_native as _native_time
    except ImportError:  # pragma: no cover - Python-only overlays remain supported
        _native_time = None


@dataclass(frozen=True)
class TimeStep:
    now: float
    dt: float
    wall_dt: float
    running: bool
    reset: bool
    stop_commands: bool


class ControlTime:
    def __init__(self, simulated, stall_timeout_s):
        if not math.isfinite(stall_timeout_s) or stall_timeout_s <= 0:
            raise ValueError('invalid clock watchdog timeout')
        self.simulated = simulated
        self.stall_timeout_s = stall_timeout_s
        self.last_ros = self.last_wall = self.progress_wall = None
        self.stalled = False
        self.capture_floor = -math.inf
        self.command_floor = -math.inf
        self._native_clock = (_native_time.ControlTime(
            bool(simulated), float(stall_timeout_s))
            if _native_time is not None else None)

    def advance(self, ros, wall):
        if not all(math.isfinite(v) for v in (ros, wall)):
            raise ValueError('nonfinite clock')
        if self._native_clock is not None:
            result = self._native_clock.advance(float(ros), float(wall))
            self._sync_native_state()
            return TimeStep(*result)
        wall_dt = 0. if self.last_wall is None else wall-self.last_wall
        ros_dt = 0. if self.last_ros is None else ros-self.last_ros
        backward = self.last_ros is not None and ros_dt < 0
        if self.progress_wall is None or ros_dt > 0 or backward:
            self.progress_wall = wall
        stalled = self.simulated and (ros <= 0 or wall-self.progress_wall > self.stall_timeout_s)
        reset = backward
        stop_commands = stalled and not self.stalled
        if backward:
            # Only a time discontinuity invalidates the observation timeline.
            # Ordinary simulation pauses preserve physical acquisition ages.
            self.capture_floor = ros
            self.command_floor = -math.inf
        if stop_commands:
            self.command_floor = ros
        self.last_ros, self.last_wall, self.stalled = ros, wall, stalled
        return TimeStep(ros if self.simulated else wall,
                        max(0., ros_dt) if self.simulated else wall_dt,
                        wall_dt, not stalled, reset, stop_commands)

    def _sync_native_state(self):
        has_last, last_ros, last_wall, has_progress, progress_wall, stalled, \
            capture_floor, command_floor = self._native_clock.state()
        self.last_ros = float(last_ros) if has_last else None
        self.last_wall = float(last_wall) if has_last else None
        self.progress_wall = float(progress_wall) if has_progress else None
        self.stalled = bool(stalled)
        self.capture_floor = float(capture_floor)
        self.command_floor = float(command_floor)

    def accepts(self, capture):
        if self._native_clock is not None:
            return bool(self._native_clock.accepts(float(capture)))
        return math.isfinite(capture) and (not self.simulated or capture > self.capture_floor)

    def fresh(self, capture, received_wall, ttl, ros, wall):
        if self._native_clock is not None:
            return bool(self._native_clock.fresh(
                float(capture), float(received_wall), float(ttl),
                float(ros), float(wall)))
        if not self.accepts(capture) or not 0 <= ros-capture <= ttl:
            return False
        return self.simulated or 0 <= wall-received_wall <= ttl

    def command_fresh(self, received_ros, received_wall, ttl, ros, wall):
        if self._native_clock is not None:
            return bool(self._native_clock.command_fresh(
                float(received_ros), float(received_wall), float(ttl),
                float(ros), float(wall)))
        if self.simulated:
            return (self.accepts(received_ros) and received_ros > self.command_floor and
                    0 <= ros-received_ros <= ttl)
        return 0 <= wall-received_wall <= ttl
