"""Asymmetric admission for an actually held posture. No implicit home fallback."""
import math
import numpy as np
from dataclasses import replace
from astribot_s1_robot_geometry.polygon import projection
from astribot_s1_robot_geometry._geometry_native import corridor_turns_outside
from .corridor import CorridorPolicy,Passage,angle

class FixedCorridorPolicy(CorridorPolicy):
    def support_bounds(self,theta=0.):
        return projection(self.profile.footprint_xy,theta)
    def lateral_interval(self,c,theta=0.):
        _,_,right,left=self.support_bounds(theta);m=self.margin(c)
        return -c.width_m/2-right+m,c.width_m/2-left-m
    def target_offset(self,c):
        low,high=self.lateral_interval(c)
        return (low+high)/2
    def half_projection(self,theta):
        # Compatibility broad phase; exact admission below uses each side.
        _,_,lo,hi=self.support_bounds(theta)
        return max(abs(lo),abs(hi))
    def choose(self,robot,path):
        # Starting/ending at the current position still needs the corridor's
        # rotation rule, even when there is no translational path segment.
        for c in self.corridors:
            along,side=c.coordinates(robot.x,robot.y)
            if 0<=along<=c.length and abs(side)<=c.width_m/2:
                if abs(angle(robot.yaw-c.heading))<=math.pi/2:return c,True
                return c.reverse(),c.bidirectional
        return super().choose(robot,path)
    def route_fits(self,c,path):
        self.route_failure=''
        if not corridor_turns_outside(self.profile.footprint_xy,
                np.asarray([c.coordinates(*point) for point in path],dtype=float).reshape(-1,2),
                c.length,c.width_m,self.margin(c),self.profile.narrow_heading_limit_rad):
            self.route_failure='CORRIDOR_ROUTE_TURN_UNREACHABLE'
            return False
        if path and all(math.dist(path[0],point)<1e-6 for point in path):
            along,side=c.coordinates(*path[0]);low,high=self.lateral_interval(c)
            return 0<=along<=c.length and low<=side<=high
        seen=False
        for a,b in zip(path,path[1:]):
            sa,la=c.coordinates(*a);sb,lb=c.coordinates(*b)
            if max(sa,sb)<0 or min(sa,sb)>c.length:continue
            # ComputePathThroughPoses concatenates segment endpoints. A
            # repeated position has no travel direction; still validate its
            # lateral reservation instead of interpreting atan2(0,0) as a turn.
            if math.dist(a,b)<=1e-9:
                low,high=self.lateral_interval(c)
                if not low<=la<=high:return False
                continue
            if sb<=sa:return False
            theta=angle(math.atan2(b[1]-a[1],b[0]-a[0])-c.heading)
            if abs(theta)>self.profile.narrow_heading_limit_rad:return False
            low,high=self.lateral_interval(c,theta)
            lo=max(0.,(0-sa)/(sb-sa));hi=min(1.,(c.length-sa)/(sb-sa))
            if lo>hi:continue
            seen=True
            if min(la+(lb-la)*lo,la+(lb-la)*hi)<low or max(la+(lb-la)*lo,la+(lb-la)*hi)>high:return False
        return seen
    def evaluate(self,selection,robot,path,version,posture,valid,geometry_clear,now,rotation_clear=False,centering_clear=False):
        task=(version.goal_id,version.clock_epoch)
        if task!=self.task or (self.last_time is not None and now<self.last_time):
            self.active=None;self.permit=();self.failure='';self.wait_at=None;self.centering_target=None;self.task=task
        self.last_time=now
        if robot is None:return Passage(selection)
        if self.active is None:self.active,self.direction_allowed=self.choose(robot,path)
        if self.active is None:return Passage(selection)
        c=self.active;p=self.profile;s,lateral=c.coordinates(robot.x,robot.y)
        theta=angle(robot.yaw-c.heading)
        front,rear=0.,0.
        if getattr(p,'footprint_xy',None) is None:return self.hold('GEOMETRY_UNAVAILABLE',now,selection)
        xmin,xmax,_,_=self.support_bounds(theta);front=xmax;rear=-xmin
        if s-rear>c.length+self.margin(c):
            self.active=None;self.permit=();self.wait_at=None;self.state='NORMAL';return Passage(selection)
        outside=s+front+self.margin(c)<0
        low,high=self.lateral_interval(c,theta)
        target=self.target_offset(c)
        self.assessment=dict(width=c.width_m,lateral_min=low,lateral_max=high,target_offset=target,
            left_clearance=c.width_m/2-lateral-self.support_bounds(theta)[3],
            right_clearance=c.width_m/2+lateral+self.support_bounds(theta)[2],
            in_place_rotation_allowed=outside and rotation_clear)
        if not valid:return self.hold('CORRIDOR_INPUT_UNAVAILABLE',now,selection)
        if not self.direction_allowed:return self.hold('CORRIDOR_DIRECTION_FORBIDDEN',now,selection)
        if selection.motion=='HOLD':return self.hold(selection.reason,now,selection)
        nominal_low,nominal_high=self.lateral_interval(c)
        if nominal_low>nominal_high:return self.hold('CORRIDOR_INSUFFICIENT_WIDTH: CHANGE_POSTURE_OUTSIDE_OR_REROUTE',now,selection)
        if abs(theta)>p.narrow_heading_limit_rad:
            if not outside:return self.hold('ROTATION_FORBIDDEN_IN_NARROW_PASSAGE',now,selection)
            if not rotation_clear:return self.hold('CORRIDOR_ALIGNMENT_SWEEP_BLOCKED',now,selection)
            self.state='ALIGN'
            return Passage(replace(selection,motion='ALIGN',speed=p.narrow_speed_m_s,reason='CORRIDOR_ALIGN_OUTSIDE'),
                'ALIGN',c.corridor_id,p.narrow_angular_speed_rad_s,alignment_heading=c.heading)
        if outside and abs(target-lateral)>p.narrow_centering_tolerance_m:
            if abs(target-lateral)>p.narrow_centering_max_offset_m or not centering_clear:return self.hold('CORRIDOR_OFFSET_SWEEP_BLOCKED',now,selection)
            self.centering_target=self.centering_target or c.point(s,target);self.state='CENTER'
            return Passage(replace(selection,motion='CENTER',speed=p.narrow_centering_speed_m_s,reason='CORRIDOR_ASYMMETRIC_OFFSET'),
                'CENTER',c.corridor_id,0.,centering_target=self.centering_target)
        if not outside and not low<=lateral<=high:
            return self.hold('CORRIDOR_RESERVATION_VIOLATED',now,selection)
        # Entrance alignment/centering have replaced the original approach.
        # Validate the still-executed passage and exit, not a discarded corner
        # in the global path before entry. Keep all later points in order.
        cursor=min(s,c.length)  # The rear may still occupy the passage after the base crosses its exit.
        first=next((i for i,point in enumerate(path) if c.coordinates(*point)[0]>=max(0.,cursor)),len(path))
        passage_path=(c.point(cursor,target),*path[first:])
        if not self.route_fits(c,passage_path):
            if self.route_failure:
                self.state='HOLD';self.permit=()
                return Passage(replace(selection,motion='HOLD',speed=0.,reason=self.route_failure),
                    'HOLD',c.corridor_id,failure=self.route_failure)
            return self.hold('CORRIDOR_OFFSET_ROUTE_REQUIRED',now,selection)
        if path and c.coordinates(*path[-1])[0] <= c.length-self.support_bounds()[0]+self.margin(c):
            reason='CORRIDOR_GOAL_BEFORE_FULL_EXIT'
            return Passage(replace(selection,motion='HOLD',speed=0.,reason=reason),
                'HOLD',c.corridor_id,failure=reason)
        if not geometry_clear:return self.hold('CORRIDOR_GEOMETRY_UNAVAILABLE',now,selection)
        # Direct admission includes a restart inside an aligned passage. The
        # current world/path/geometry have just been revalidated; no width tier
        # or nominal pose label adds an unconditional stop.
        self.permit=(c.corridor_id,c.entry,version.goal_id,version.path_revision,version.map_epoch,
                     version.localization_epoch,version.envelope_epoch,version.clock_epoch)
        self.state='TRANSIT';self.wait_at=None;self.centering_target=None
        return Passage(replace(selection,motion='SLOW',speed=min(selection.speed,p.narrow_speed_m_s),reason='CORRIDOR_POLYGON_TRANSIT'),
            'TRANSIT',c.corridor_id,p.narrow_angular_speed_rad_s,permit=self.permit,tracking_heading=c.heading)
