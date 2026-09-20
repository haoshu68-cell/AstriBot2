"""Physical-time freshness with an independent simulation-clock watchdog."""
from dataclasses import dataclass
import math


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

    def advance(self, ros, wall):
        if not all(math.isfinite(v) for v in (ros, wall)):
            raise ValueError('nonfinite clock')
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

    def accepts(self, capture):
        return math.isfinite(capture) and (not self.simulated or capture > self.capture_floor)

    def fresh(self, capture, received_wall, ttl, ros, wall):
        if not self.accepts(capture) or not 0 <= ros-capture <= ttl:
            return False
        return self.simulated or 0 <= wall-received_wall <= ttl

    def command_fresh(self, received_ros, received_wall, ttl, ros, wall):
        if self.simulated:
            return (self.accepts(received_ros) and received_ros > self.command_floor and
                    0 <= ros-received_ros <= ttl)
        return 0 <= wall-received_wall <= ttl
