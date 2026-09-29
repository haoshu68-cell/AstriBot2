"""Pure body-to-world velocity conversion used by the ROS adapter."""
import math

try:
    import os
    if os.environ.get('ASTRIBOT_NAV_NATIVE_KERNELS', '').lower() in ('1', 'true', 'yes'):
        from astribot_s1_navigation_policy_native import _navigation_math_native as _native
    else:
        _native = None
except ImportError:
    _native = None


def body_to_world_xy(vx, vy, yaw):
    """Rotate a body-frame planar velocity using the adapter's legacy formula."""
    if _native is not None:
        return tuple(_native.body_to_world_xy(float(vx), float(vy), float(yaw)))
    body_speed = math.hypot(vx, vy)
    if body_speed <= 1e-6:
        return 0.0, 0.0
    body_angle = math.atan2(vy, vx)
    world_angle = body_angle + yaw
    return math.cos(world_angle) * body_speed, math.sin(world_angle) * body_speed
