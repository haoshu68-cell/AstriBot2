"""Final planar command restriction; a restriction cannot create motion."""
import math
import numpy as np
from .motion_geometry import stopping_horizon, sampling_margin
from .continuous_sweep import motion_clearance

def scan_usable(ranges, range_min, range_max, angle_min, angle_increment, minimum_fraction):
    if (not ranges or not all(math.isfinite(v) for v in (range_min,range_max,angle_min,angle_increment))
            or not 0<=range_min<range_max or angle_increment<=0):return False
    valid=sum(r==math.inf or (math.isfinite(r) and range_min<=r<=range_max) for r in ranges)
    return valid/len(ranges)>=minimum_fraction

def costmap_clearing_ranges(ranges, range_max, max_marking_range):
    endpoint=range_max-max(1e-4,abs(range_max)*1e-6)
    if (not math.isfinite(endpoint) or not math.isfinite(max_marking_range)
            or max_marking_range<0 or endpoint<=max_marking_range):
        raise ValueError('clearing endpoint must lie beyond the costmap marking range')
    return [endpoint if r==range_max or r==math.inf else r for r in ranges]

def swept_point_collision(points, command, profile):
    vx,vy,wz=command
    if not all(math.isfinite(v) for v in command):return True
    duration=stopping_horizon(command,profile)
    error=sampling_margin(command,profile,.05)
    radius=(math.hypot(vx,vy)*duration+math.hypot(profile.half_length_m,profile.half_width_m)+
            profile.clearance_margin_m+profile.payload_extra_margin_m+error)
    points=tuple((x,y) for x,y in points if math.hypot(x,y)<=radius+1e-9)
    if not points:return False
    t=np.minimum(duration,np.arange(max(1,int(math.ceil(duration/.05)))+1)*.05)
    obstacles=np.asarray(points)[None,:,:]
    gaps=motion_clearance(command,np.r_[0.,t[:-1]][:,None],t[:,None],
                          obstacles,obstacles,profile)
    return bool(np.any(gaps<=0))

class CommandRestriction:
    def __init__(self, profile):
        self.profile=profile;self.output=(0.,0.,0.);self.recovering=True

    def apply(self, command, cap, angular_cap, stop, dt, *, allow_zero_dt=False):
        if stop or not all(math.isfinite(v) for v in (*command,cap,angular_cap,dt)):
            self.output=(0.,0.,0.);self.recovering=True;return self.output
        if cap<0 or angular_cap<0 or dt<0 or (dt==0 and not allow_zero_dt):
            self.output=(0.,0.,0.);self.recovering=True;return self.output
        speed=math.hypot(*command[:2]);w=abs(command[2])
        ratio=min(1.,cap/speed if speed>0 else 1.,angular_cap/w if w>0 else 1.)
        target=tuple(v*ratio for v in command)
        # Clamp every excess, but floating-point norm roundoff is not a new restriction episode.
        limited=ratio<1.-1e-12
        if self.recovering or limited:
            step=self.profile.max_acceleration_m_s2*min(dt,.05)
            old_speed=math.hypot(*self.output[:2]);target_speed=math.hypot(*target[:2])
            gain=min(1.,(old_speed+step)/max(target_speed,1e-12))
            angular_gain=min(1.,(abs(self.output[2])+self.profile.max_angular_acceleration_rad_s2*min(dt,.05))/max(abs(target[2]),1e-12))
            if allow_zero_dt and dt==0:
                if target_speed==0:gain=1.
                if target[2]==0:angular_gain=1.
            gain=min(gain,angular_gain);target=tuple(v*gain for v in target)
            self.recovering=gain<1.-1e-12 or limited
        self.output=target
        return target
