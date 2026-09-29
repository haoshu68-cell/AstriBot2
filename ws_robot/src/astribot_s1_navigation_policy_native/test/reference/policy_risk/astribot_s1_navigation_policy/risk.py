"""Swept planar-envelope risk for conservative simulation transport profiles."""
from dataclasses import dataclass
import math
import os

import numpy as np

from .contracts import finite
from .motion_geometry import stopping_horizon
from .continuous_sweep import motion_clearance

try:
    if os.environ.get('ASTRIBOT_NAV_NATIVE_KERNELS', '').lower() in ('1', 'true', 'yes'):
        from astribot_s1_navigation_policy_native import _navigation_math_native as _native
    else:
        _native = None
except ImportError:
    _native = None


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
    immediate_obstacle_ids: tuple[str,...] = ()


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
    # Reaching the end of the forecast must not replace the final path tangent
    # with the robot's earlier approach heading.
    for before,after in reversed(tuple(zip(path,path[1:]))):
        if math.dist(before,after)>1e-9:
            return a[0],a[1],math.atan2(after[1]-before[1],after[0]-before[0])
    return a[0],a[1],fallback.yaw


def _path_position_batch(path, distances, fallback):
    """Batch route interpolation while retaining Python's endpoint tangent rule."""
    if _native is not None:
        try:
            path_xy = [value for point in path for value in point[:2]]
            values = _native.path_position_batch(
                path_xy, [float(value) for value in distances],
                [fallback.x, fallback.y, fallback.yaw])
            return np.asarray(values, dtype=float).reshape((-1, 3))
        except (TypeError, ValueError, IndexError):
            # Preserve the Python route/shape oracle for malformed inputs.
            pass
    return np.asarray([path_position(path, distance, fallback)
                       for distance in distances], dtype=float)


def remaining_path(path, robot):
    if _native is not None:
        try:
            path_xy = [value for point in path for value in point[:2]]
            values = _native.remaining_path(path_xy, float(robot.x), float(robot.y))
            return tuple((values[i], values[i + 1])
                         for i in range(0, len(values), 2))
        except (TypeError, ValueError, IndexError):
            # Preserve the Python route/shape oracle for malformed inputs.
            pass
    if not path:return ()
    best=None
    for i,(a,b) in enumerate(zip(path,path[1:])):
        dx,dy=b[0]-a[0],b[1]-a[1];length=dx*dx+dy*dy
        t=max(0.,min(1.,((robot.x-a[0])*dx+(robot.y-a[1])*dy)/length)) if length else 0.
        x,y=a[0]+t*dx,a[1]+t*dy;distance=(x-robot.x)**2+(y-robot.y)**2
        if best is None or distance<best[0]:best=(distance,i,(x,y))
    return ((robot.x,robot.y),best[2],*path[best[1]+1:]) if best else ((robot.x,robot.y),*path)


def box_clearance(x,y,yaw,box,profile):
    from .swept_geometry import obstacle_bounds, footprint_clearance
    lo,hi=obstacle_bounds(box)
    return footprint_clearance(x,y,yaw,lo,hi,profile)


def evaluate_risk(world, robot, path, profile, speed_limit=None):
    speed=profile.max_speed_m_s if speed_limit is None else speed_limit
    if not math.isfinite(speed) or not 0<speed<=profile.max_speed_m_s:
        raise ValueError('risk forecast speed outside profile')
    # A proposed cap cannot erase motion already present in the measured state.
    speed=max(speed,math.hypot(robot.vx,robot.vy))
    route=remaining_path(path,robot)
    from .swept_geometry import bounds_many, clearance_many
    tracks=world.tracks
    if not tracks:
        uncertain=bool(world.unassociated)
        return Risk(uncertain,False,float('inf'),float('inf'),(),False,uncertain)
    lower,upper=bounds_many([track.geometry for track in tracks])
    current=clearance_many(robot.x,robot.y,robot.yaw,lower,upper,profile)
    minimum=float(np.min(current));immediate=bool(np.any(current<=0))
    blocked=set(np.flatnonzero(current<=0).tolist());first=float('inf')
    immediate_owners=set(blocked)
    from .world_geometry import prediction_rows
    batch=prediction_rows(world)
    if batch.owners.size:
        offsets=batch.offsets_ns*1e-9;owners=batch.owners
        lower,upper=batch.lower,batch.upper
        unique,inverse=np.unique(batch.offsets_ns,return_inverse=True)
        poses=_path_position_batch(
            route, [speed * (int(offset) * 1e-9) for offset in unique], robot)
        positions=poses[inverse]
        gaps=clearance_many(positions[:,0],positions[:,1],positions[:,2],lower,upper,profile)
        hits=(gaps<=0) if route else np.zeros(len(batch.owners),dtype=bool)
        if np.any(hits):
            first=float(np.min(offsets[hits]));blocked.update(owners[hits].tolist())
        command=(robot.vx,robot.vy,robot.wz)
        stop_time=stopping_horizon(command,profile)
        previous=np.r_[0.,offsets[:-1]]
        previous[np.r_[True,owners[1:]!=owners[:-1]]]=0.
        active=previous<stop_time
        swept=prediction_rows(world,swept=True)
        actual=motion_clearance(command,previous[active],np.minimum(offsets[active],stop_time),
                                swept.lower[active],swept.upper[active],profile,
                                (robot.x,robot.y,robot.yaw))
        immediate=immediate or stop_time>profile.prediction_horizon_s or bool(np.any(actual<=0))
        immediate_owners.update(swept.owners[active][actual<=0].tolist())
    selected=[tracks[i] for i in sorted(blocked)]
    def moving_track(track):
        model=track.prediction_model
        if model is not None and model.steps:
            t=model.steps[-1][1];center=track.geometry.center_m
            return math.hypot((center.x+model.velocity.x*t)-center.x,
                              (center.y+model.velocity.y*t)-center.y)>.1
        return bool(track.predictions and
               math.hypot(track.predictions[-1].geometry.center_m.x-track.geometry.center_m.x,
                          track.predictions[-1].geometry.center_m.y-track.geometry.center_m.y)>.1)
    moving=any(moving_track(track) for track in selected)
    uncertain=bool(world.unassociated)
    return Risk(bool(blocked) or uncertain,immediate,minimum,first,
                tuple(t.fused_track_id for t in selected),moving,uncertain,
                tuple(tracks[i].fused_track_id for i in sorted(immediate_owners)))
