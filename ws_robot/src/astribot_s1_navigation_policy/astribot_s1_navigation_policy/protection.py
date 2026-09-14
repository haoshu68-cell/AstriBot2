"""Final planar command restriction; a restriction cannot create motion."""
import math

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
    duration=profile.reaction_time_s+math.hypot(vx,vy)/profile.brake_deceleration_m_s2
    duration=max(duration,profile.reaction_time_s+abs(wz)/profile.angular_brake_deceleration_rad_s2)
    hx=profile.half_length_m+profile.clearance_margin_m+profile.payload_extra_margin_m
    hy=profile.half_width_m+profile.clearance_margin_m+profile.payload_extra_margin_m
    radius=math.hypot(vx,vy)*duration+math.hypot(hx,hy)
    points=tuple((x,y) for x,y in points if math.hypot(x,y)<=radius+1e-9)
    if not points:return False
    for i in range(max(1,int(math.ceil(duration/.05)))+1):
        t=min(duration,i*.05);angle=wz*t
        if abs(wz)<1e-6:x,y=vx*t,vy*t
        else:
            x=(vx*math.sin(angle)+vy*(math.cos(angle)-1))/wz
            y=(vx*(1-math.cos(angle))+vy*math.sin(angle))/wz
        c,s=math.cos(angle),math.sin(angle)
        for px,py in points:
            dx,dy=px-x,py-y
            if abs(c*dx+s*dy)<=hx and abs(-s*dx+c*dy)<=hy:return True
    return False

class CommandRestriction:
    def __init__(self, profile):
        self.profile=profile;self.output=(0.,0.,0.);self.recovering=True

    def apply(self, command, cap, angular_cap, stop, dt):
        if stop or not all(math.isfinite(v) for v in (*command,cap,angular_cap,dt)):
            self.output=(0.,0.,0.);self.recovering=True;return self.output
        if cap<0 or angular_cap<0 or dt<=0:
            self.output=(0.,0.,0.);self.recovering=True;return self.output
        speed=math.hypot(*command[:2]);w=abs(command[2])
        ratio=min(1.,cap/max(speed,1e-12),angular_cap/max(w,1e-12))
        target=tuple(v*ratio for v in command)
        # Clamp every excess, but floating-point norm roundoff is not a new restriction episode.
        limited=ratio<1.-1e-12
        if self.recovering or limited:
            step=self.profile.max_acceleration_m_s2*min(dt,.05)
            old_speed=math.hypot(*self.output[:2]);target_speed=math.hypot(*target[:2])
            gain=min(1.,(old_speed+step)/max(target_speed,1e-12))
            angular_gain=min(1.,(abs(self.output[2])+self.profile.max_angular_acceleration_rad_s2*min(dt,.05))/max(abs(target[2]),1e-12))
            gain=min(gain,angular_gain);target=tuple(v*gain for v in target)
            self.recovering=gain<1.-1e-12 or limited
        self.output=target
        return target
