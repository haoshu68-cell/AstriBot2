"""Stable spatial occupancy for lidar; cell identity is not object identity."""
import math


def occupied_cells(points, resolution):
    if not math.isfinite(resolution) or resolution<=0:
        raise ValueError('positive scan occupancy resolution required')
    cells=set()
    for x,y in points:
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('finite scan point required')
        cells.add((math.floor(x/resolution),math.floor(y/resolution)))
    return tuple((f'cell:{resolution:g}:{x}:{y}',(x+.5)*resolution,(y+.5)*resolution)
                 for x,y in sorted(cells))


def angular_box_free(corners, ranges, range_min, range_max, angle_min, angle_increment, resolution):
    """Require fresh free rays across the whole box, including bracketing beams.

    Unmeasured angular gaps wider than one occupancy cell cannot erase it.
    A missed return or an occluding surface anywhere in the span retains it.
    """
    cx=sum(p[0] for p in corners)/len(corners);cy=sum(p[1] for p in corners)/len(corners)
    center=math.atan2(cy,cx)
    angles=[center+math.remainder(math.atan2(y,x)-center,2*math.pi) for x,y in corners]
    far=max(math.hypot(x,y) for x,y in corners)
    near=min(math.hypot(x,y) for x,y in corners)
    if (near<range_min or far>=range_max-.2 or max(angles)-min(angles)>=math.pi or
            far*angle_increment>resolution):return False
    lo=math.floor((min(angles)-angle_min)/angle_increment)
    hi=math.ceil((max(angles)-angle_min)/angle_increment)
    full=abs(len(ranges)*angle_increment-2*math.pi)<=1.5*angle_increment
    for i in range(lo,hi+1):
        if not 0<=i<len(ranges):
            if not full:return False
            i%=len(ranges)
        observed=ranges[i]
        if math.isnan(observed) or observed==-math.inf or min(observed,range_max)<far+.15:return False
    return True
