#!/usr/bin/env python3
"""Measured zero-command envelope; no ROS/SDK writes and no production auto-loading."""
import bisect
import math


class MeasuredStopModel:
    def __init__(self, profile):
        self.axes=profile['axes']
        for direction,curve in self.axes.items():
            speed=curve['speed'];upper=curve['upper']
            if not speed or len(speed)!=len(upper):raise ValueError('invalid stopping curve '+direction)
            if any(not math.isfinite(x) or x<=0 for x in speed):raise ValueError('invalid speed')
            if any(not math.isfinite(x) or x<0 for x in upper):raise ValueError('invalid distance')
            if any(a>=b for a,b in zip(speed,speed[1:])):raise ValueError('speed must increase')
            if any(a>b for a,b in zip(upper,upper[1:])):raise ValueError('upper distance must not decrease')

    def zero_command_upper(self,direction,speed):
        """Conservative step envelope within measured range; low speed uses first bin.

        This is an empirical engineering envelope, not a probabilistic guarantee.
        Using the next measured bin avoids underestimating a curved stopping relation
        by linearly interpolating between sparse samples.
        """
        if not math.isfinite(speed) or speed<0:raise ValueError('invalid speed')
        if speed==0:return 0.
        curve=self.axes[direction]
        if speed>curve['speed'][-1]:raise ValueError('speed outside characterized range')
        i=bisect.bisect_left(curve['speed'],speed)
        return curve['upper'][i]

    def planning_distance(self,direction,speed,pose_age,command_delay,smoother_deceleration):
        """Upper planning budget = measured tail + unobserved travel + smoother ramp.

        a is the positive configured deceleration; the ramp term is a planning
        approximation. An end-to-end smoothed stop still needs hardware validation.
        """
        if min(pose_age,command_delay)<0 or smoother_deceleration<=0:
            raise ValueError('invalid latency or deceleration')
        if not all(math.isfinite(v) for v in [pose_age,command_delay,smoother_deceleration]):
            raise ValueError('nonfinite planning input')
        return self.zero_command_upper(direction,speed)+speed*(pose_age+command_delay)+speed*speed/(2*smoother_deceleration)
