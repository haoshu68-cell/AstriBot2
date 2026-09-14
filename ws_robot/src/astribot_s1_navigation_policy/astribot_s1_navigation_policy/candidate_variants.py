"""Bounded, endpoint-preserving alternatives; every result needs full validation."""
import math


def lateral_variants(route, maximum):
    if len(route)<3 or not 0<maximum<=.2:return ()
    lengths=[0.]
    for a,b in zip(route,route[1:]):lengths.append(lengths[-1]+math.dist(a,b))
    dx,dy=route[-1][0]-route[0][0],route[-1][1]-route[0][1]
    chord=math.hypot(dx,dy)
    if not math.isfinite(lengths[-1]) or lengths[-1]<.6 or chord<.1:return ()
    # C2 endpoint taper preserves position and tangent at takeover and goal.
    weights=[64*(s/lengths[-1])**3*(1-s/lengths[-1])**3 for s in lengths]
    weights[0]=weights[-1]=0.
    return tuple(tuple((a[0]-dy/chord*offset*w,a[1]+dx/chord*offset*w)
                       for a,w in zip(route,weights))
                 for offset in (-maximum/4,maximum/4,-maximum/2,maximum/2,
                                -3*maximum/4,3*maximum/4,-maximum,maximum))
