"""Bound complete constant-twist intervals against swept obstacle boxes."""
import math
import numpy as np
from .motion_geometry import body_pose
from .swept_geometry import clearance_many


def motion_clearance(command, begin, end, lower, upper, profile, origin=(0., 0., 0.)):
    """Positive bounds certify the whole interval; unresolved boundaries reject.

    Obstacle bounds must enclose the obstacle throughout each interval. Subdivide
    only intervals whose midpoint and enclosing circle cannot establish safety.
    The depth limit bounds work, never the physical clearance requirement.
    """
    if not all(math.isfinite(v) for v in (*command, *origin)):
        raise ValueError('finite motion required')
    lower, upper = np.asarray(lower), np.asarray(upper)
    values = np.broadcast_arrays(begin, end, lower[..., 0], lower[..., 1],
                                 upper[..., 0], upper[..., 1])
    shape = values[0].shape
    start, finish, lx, ly, ux, uy = (np.asarray(v, dtype=float).ravel() for v in values)
    if (not all(np.all(np.isfinite(v)) for v in values) or
            np.any(start < 0) or np.any(finish < start) or
            np.any(lx > ux) or np.any(ly > uy)):
        raise ValueError('finite ordered sweep intervals and bounds required')
    lo, hi = np.column_stack((lx, ly)), np.column_stack((ux, uy))
    result = np.full(start.size, np.inf)
    owners = np.arange(start.size)
    speed = math.hypot(*command[:2])
    radius = math.hypot(profile.half_length_m, profile.half_width_m)
    boundary_speed = speed + radius * abs(command[2])
    margin = profile.clearance_margin_m + profile.payload_extra_margin_m
    c, s = math.cos(origin[2]), math.sin(origin[2])
    for depth in range(11):
        if not owners.size:
            break
        mid, half = (start + finish) / 2, (finish - start) / 2
        bx, by, angle = body_pose(command, mid, np)
        x, y = origin[0] + c*bx - s*by, origin[1] + s*bx + c*by
        lower, upper = lo[owners], hi[owners]
        point = clearance_many(x, y, origin[2] + angle, lower, upper, profile)
        dx = np.maximum(np.maximum(lower[:, 0] - x, 0.), x - upper[:, 0])
        dy = np.maximum(np.maximum(lower[:, 1] - y, 0.), y - upper[:, 1])
        circle = np.hypot(dx, dy) - radius - margin - speed*half - 1e-12
        bound = np.maximum(point - boundary_speed*half, circle)
        safe = bound > 0
        np.minimum.at(result, owners[safe], bound[safe])
        rejected = (~safe) & ((point <= 0) | (depth == 10))
        np.minimum.at(result, owners[rejected], bound[rejected])
        pending = (~safe) & (~rejected) & (result[owners] > 0)
        owners = np.repeat(owners[pending], 2)
        start, finish = (np.column_stack((start[pending], mid[pending])).ravel(),
                         np.column_stack((mid[pending], finish[pending])).ravel())
    return result.reshape(shape)
