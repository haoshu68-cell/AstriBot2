"""Swept planar-envelope risk for conservative simulation transport profiles."""
from dataclasses import dataclass
import math

from .contracts import finite


@dataclass(frozen=True)
class RobotState:
    x: float
    y: float
    yaw: float
    vx: float = 0.
    vy: float = 0.
    wz: float = 0.

    def __post_init__(self):
        for name in ('x', 'y', 'yaw', 'vx', 'vy', 'wz'):
            finite(getattr(self, name), name)


@dataclass(frozen=True)
class Risk:
    blocked: bool
    immediate: bool
    clearance_m: float
    conflict_time_s: float
    obstacle_ids: tuple[str,...]
    moving: bool
    uncertain: bool


def path_position(path, distance, fallback):
    if not path:
        return fallback.x,fallback.y,fallback.yaw
    for a,b in zip(path,path[1:]):
        length=math.hypot(b[0]-a[0],b[1]-a[1])
        if length>1e-9 and distance<=length:
            ratio=distance/length
            return a[0]+ratio*(b[0]-a[0]),a[1]+ratio*(b[1]-a[1]),math.atan2(b[1]-a[1],b[0]-a[0])
        distance-=length
    a=path[-1]
    return a[0],a[1],fallback.yaw


def remaining_path(path, robot):
    if not path:return ()
    best=None
    for i,(a,b) in enumerate(zip(path,path[1:])):
        dx,dy=b[0]-a[0],b[1]-a[1];length=dx*dx+dy*dy
        t=max(0.,min(1.,((robot.x-a[0])*dx+(robot.y-a[1])*dy)/length)) if length else 0.
        x,y=a[0]+t*dx,a[1]+t*dy;distance=(x-robot.x)**2+(y-robot.y)**2
        if best is None or distance<best[0]:best=(distance,i,(x,y))
    return ((robot.x,robot.y),best[2],*path[best[1]+1:]) if best else ((robot.x,robot.y),*path)


def box_clearance(x,y,yaw,box,profile):
    # Bounding rectangles remain conservative for a rotated rectangular robot.
    hx=abs(math.cos(yaw))*profile.half_length_m+abs(math.sin(yaw))*profile.half_width_m
    hy=abs(math.sin(yaw))*profile.half_length_m+abs(math.cos(yaw))*profile.half_width_m
    margin=profile.clearance_margin_m+profile.payload_extra_margin_m
    uncertainty_x=2*math.sqrt(max(0.,box.position_covariance_m2.values[0]))
    uncertainty_y=2*math.sqrt(max(0.,box.position_covariance_m2.values[4]))
    dx=abs(x-box.center_m.x)-hx-box.size_m.x/2-margin-uncertainty_x
    dy=abs(y-box.center_m.y)-hy-box.size_m.y/2-margin-uncertainty_y
    return math.hypot(max(0.,dx),max(0.,dy)) if dx>0 or dy>0 else max(dx,dy)


def evaluate_risk(world, robot, path, profile):
    route=remaining_path(path,robot)
    minimum=float('inf');first=float('inf');blocked=[];moving=False;immediate=False
    poses={}
    c,s=math.cos(robot.yaw),math.sin(robot.yaw)
    stop_time=profile.reaction_time_s+math.hypot(robot.vx,robot.vy)/profile.brake_deceleration_m_s2
    for track in world.tracks:
        current=box_clearance(robot.x,robot.y,robot.yaw,track.geometry,profile)
        minimum=min(minimum,current)
        # Real motion sweep must also be checked when it differs from the planned heading.
        if current<=0:immediate=True
        hit=False
        for sample in track.predictions:
            t=sample.offset_ns*1e-9
            if sample.offset_ns not in poses:
                poses[sample.offset_ns]=path_position(route,profile.max_speed_m_s*t,robot)
            x,y,yaw=poses[sample.offset_ns]
            gap=box_clearance(x,y,yaw,sample.geometry,profile)
            if gap<=0 and route:hit=True;first=min(first,t)
            if t<=stop_time:
                actual=box_clearance(robot.x+(c*robot.vx-s*robot.vy)*t,
                                     robot.y+(s*robot.vx+c*robot.vy)*t,
                                     robot.yaw+robot.wz*t,sample.geometry,profile)
                if actual<=0:immediate=True
        if hit or current<=0:
            blocked.append(track.fused_track_id)
            if track.predictions:
                end=track.predictions[-1].geometry.center_m;start=track.geometry.center_m
                moving |= math.hypot(end.x-start.x,end.y-start.y)>.1
    uncertain=bool(world.unassociated)
    # Until camera rays are calibrated into the route frame, unresolved vision must not be labelled clear.
    return Risk(bool(blocked) or uncertain,immediate,minimum,first,tuple(blocked),moving,uncertain)
