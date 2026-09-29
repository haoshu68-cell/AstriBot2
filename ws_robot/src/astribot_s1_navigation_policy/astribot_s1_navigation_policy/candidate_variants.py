"""Bounded, endpoint-preserving alternatives; every result needs full validation."""
import math
import os


_native = None
if os.environ.get('ASTRIBOT_NAV_NATIVE_KERNELS', '').lower() in ('1', 'true', 'yes'):
    try:
        from astribot_s1_navigation_policy_native import _navigation_math_native as _native
    except ImportError:  # pragma: no cover - Python-only overlays remain supported
        _native = None


def lateral_variants(route, maximum):
    if len(route)<3 or not 0<maximum<=.2:return ()
    if (_native is not None and
            all(len(point) == 2 for point in route) and
            all(isinstance(value, (int, float)) and math.isfinite(value)
                for point in route for value in point)):
        flat = _native.lateral_variants(
            [float(value) for point in route for value in point], float(maximum))
        if not flat:
            return ()
        count = len(route)
        return tuple(tuple(tuple(flat[offset * count * 2 + index * 2 + axis]
                                for axis in range(2))
                          for index in range(count))
                     for offset in range(8))
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
