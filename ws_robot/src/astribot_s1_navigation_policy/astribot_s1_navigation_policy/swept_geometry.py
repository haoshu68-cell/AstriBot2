"""Shared footprint separation from uncertainty-expanded swept obstacle boxes."""
import math
from functools import lru_cache
import numpy as np


def bounds_many(boxes, previous=None):
    """Batch the same uncertainty-expanded boxes; retain every observation."""
    def values(items):
        return np.asarray([(b.center_m.x, b.center_m.y, b.size_m.x/2, b.size_m.y/2,
                            b.position_covariance_m2.values[0], b.position_covariance_m2.values[4])
                           for b in items], dtype=float).reshape((-1, 6))
    current = values(boxes)
    before = current if previous is None else values(previous)
    uncertainty = 2*np.sqrt(np.maximum(current[:, 4:], before[:, 4:]))
    return (np.minimum(current[:, :2]-current[:, 2:4], before[:, :2]-before[:, 2:4])-uncertainty,
            np.maximum(current[:, :2]+current[:, 2:4], before[:, :2]+before[:, 2:4])+uncertainty)


def clearance_many(x, y, yaw, lower, upper, profile, sampling_margin=0.):
    """Separate first; resolve ambiguous clearance with exact rectangle distance."""
    footprint=getattr(profile,'footprint_xy',None)
    if footprint is not None:
        from astribot_s1_robot_geometry.polygon import box_distance_many
        # A circumscribed-circle lower bound cheaply certifies distant pairs.
        lo,hi=np.asarray(lower),np.asarray(upper)
        xx,yy,aa,lx,ly,ux,uy,margin=np.broadcast_arrays(x,y,yaw,lo[...,0],lo[...,1],hi[...,0],hi[...,1],
            profile.clearance_margin_m+profile.payload_extra_margin_m+sampling_margin)
        radius=float(np.max(np.linalg.norm(footprint,axis=1)))
        bound=np.hypot(np.maximum(np.maximum(lx-xx,0.),xx-ux),np.maximum(np.maximum(ly-yy,0.),yy-uy))-radius
        close=bound<=margin+1e-12;result=np.array(bound-margin-1e-12,copy=True)
        if np.any(close):
            result[close]=box_distance_many(footprint,xx[close],yy[close],aa[close],
                np.column_stack((lx[close],ly[close])),np.column_stack((ux[close],uy[close])))-margin[close]-1e-12
        return result
    x, y, yaw = np.asarray(x), np.asarray(y), np.asarray(yaw)
    length, width = profile.half_length_m, profile.half_width_m
    c, s = np.cos(yaw), np.sin(yaw)
    ac, ass = np.abs(c), np.abs(s)
    bx, by = (lower[..., 0]+upper[..., 0])/2, (lower[..., 1]+upper[..., 1])/2
    hx, hy = (upper[..., 0]-lower[..., 0])/2, (upper[..., 1]-lower[..., 1])/2
    dx, dy = x-bx, y-by
    separation = np.maximum(np.maximum(np.abs(dx)-ac*length-ass*width-hx,
                                       np.abs(dy)-ass*length-ac*width-hy),
                            np.maximum(np.abs(dx*c+dy*s)-length-hx*ac-hy*ass,
                                       np.abs(-dx*s+dy*c)-width-hx*ass-hy*ac))
    dx = np.maximum(np.maximum(lower[..., 0]-x, 0.), x-upper[..., 0])
    dy = np.maximum(np.maximum(lower[..., 1]-y, 0.), y-upper[..., 1])
    circle = np.hypot(dx, dy)-math.hypot(length, width)
    margin=profile.clearance_margin_m+profile.payload_extra_margin_m+sampling_margin
    bound,margin=np.broadcast_arrays(np.maximum(separation,circle),margin)
    # Far separation and physical overlap are already decided. Only close,
    # disjoint pairs need vertex-to-edge distances instead of a SAT lower bound.
    close=(separation>0)&(bound<=margin+1e-12)
    if not np.any(close):return bound-margin-1e-12
    values=np.broadcast_arrays(x,y,c,s,lower[...,0],lower[...,1],upper[...,0],upper[...,1],bound)
    xx,yy,cc,ss,lx,ly,ux,uy,_=(value[close] for value in values)
    distance=np.full(xx.shape,np.inf)
    for sx,sy in ((-1,-1),(1,-1),(1,1),(-1,1)):
        px=xx+cc*sx*length-ss*sy*width;py=yy+ss*sx*length+cc*sy*width
        distance=np.minimum(distance,np.hypot(np.maximum(np.maximum(lx-px,0.),px-ux),
                                              np.maximum(np.maximum(ly-py,0.),py-uy)))
        dx=(lx if sx<0 else ux)-xx;dy=(ly if sy<0 else uy)-yy
        distance=np.minimum(distance,np.hypot(np.maximum(np.abs(cc*dx+ss*dy)-length,0.),
                                              np.maximum(np.abs(-ss*dx+cc*dy)-width,0.)))
    result=np.array(bound,copy=True);result[close]=distance
    return result-margin-1e-12


@lru_cache(maxsize=4096)
def footprint_axes(length, width, yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return c,s,abs(c)*length+abs(s)*width,abs(s)*length+abs(c)*width,math.hypot(length,width)


def obstacle_bounds(box, previous=None):
    previous = box if previous is None else previous
    lower, upper = [], []
    for axis, index in (('x', 0), ('y', 4)):
        uncertainty = 2 * math.sqrt(max(previous.position_covariance_m2.values[index],
                                        box.position_covariance_m2.values[index]))
        lower.append(min(getattr(b.center_m, axis) - getattr(b.size_m, axis) / 2
                         for b in (previous, box)) - uncertainty)
        upper.append(max(getattr(b.center_m, axis) + getattr(b.size_m, axis) / 2
                         for b in (previous, box)) + uncertainty)
    return lower, upper


def footprint_clearance(x, y, yaw, lower, upper, profile, sampling_margin=0.):
    if getattr(profile,'footprint_xy',None) is not None:
        return float(clearance_many(x,y,yaw,np.asarray(lower),np.asarray(upper),profile,sampling_margin))
    length,width=profile.half_length_m,profile.half_width_m
    c,s,rx,ry,radius=footprint_axes(length,width,yaw)
    bx, by = (lower[0] + upper[0]) / 2, (lower[1] + upper[1]) / 2
    hx, hy = (upper[0] - lower[0]) / 2, (upper[1] - lower[1]) / 2
    margin = profile.clearance_margin_m + profile.payload_extra_margin_m + sampling_margin
    # Each projection gap is a lower bound on Euclidean separation. The robot
    # axes remove empty AABB corners without reducing physical clearance.
    dx,dy=x-bx,y-by
    separation=max(abs(dx)-rx-hx,abs(dy)-ry-hy,
                   abs(dx*c+dy*s)-length-hx*abs(c)-hy*abs(s),
                   abs(-dx*s+dy*c)-width-hx*abs(s)-hy*abs(c))
    dx, dy = max(lower[0]-x, 0., x-upper[0]), max(lower[1]-y, 0., y-upper[1])
    circle = math.hypot(dx, dy) - radius
    bound=max(separation,circle)
    if separation>0 and bound<=margin+1e-12:
        distance=math.inf
        for sx,sy in ((-1,-1),(1,-1),(1,1),(-1,1)):
            px=x+c*sx*length-s*sy*width;py=y+s*sx*length+c*sy*width
            distance=min(distance,math.hypot(max(lower[0]-px,0.,px-upper[0]),
                                             max(lower[1]-py,0.,py-upper[1])))
            dx=(lower[0] if sx<0 else upper[0])-x;dy=(lower[1] if sy<0 else upper[1])-y
            distance=min(distance,math.hypot(max(abs(c*dx+s*dy)-length,0.),
                                             max(abs(-s*dx+c*dy)-width,0.)))
        bound=distance
    return bound-margin-1e-12


def path_samples(route, reach, profile):
    """Cover translation and corner rotation; return their sampling error bounds."""
    step = min(.025, profile.clearance_margin_m / 2)
    distance, previous_heading = 0., None
    radius = math.hypot(profile.half_length_m, profile.half_width_m)
    for a, b in zip(route, route[1:]):
        length = math.dist(a, b)
        if length < 1e-9:
            continue
        heading = math.atan2(b[1]-a[1], b[0]-a[0])
        if previous_heading is not None:
            turn = math.remainder(heading-previous_heading, 2*math.pi)
            count = max(1, math.ceil(abs(turn)/.05))
            for j in range(count+1):
                yield distance, a[0], a[1], previous_heading+turn*j/count, radius*abs(turn)/(2*count)
        travel = min(length, max(0., reach-distance))
        count = max(1, math.ceil(travel/step))
        for j in range(count+1):
            d = travel*j/count
            yield distance+d, a[0]+(b[0]-a[0])*d/length, a[1]+(b[1]-a[1])*d/length, heading, travel/(2*count)
        distance += length
        if distance > reach:
            break
        previous_heading = heading
