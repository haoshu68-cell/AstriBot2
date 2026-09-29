"""Body-frame constant-twist envelope shared by prediction and final protection."""
import math

try:
    import os
    if os.environ.get('ASTRIBOT_NAV_NATIVE_KERNELS', '').lower() in ('1', 'true', 'yes'):
        from astribot_s1_navigation_policy_native import _navigation_math_native as _native
    else:
        _native = None
except ImportError:
    _native = None


def stopping_horizon(command, profile):
    if _native is not None:
        return _native.stopping_horizon(
            list(command), profile.reaction_time_s,
            profile.brake_deceleration_m_s2,
            profile.angular_brake_deceleration_rad_s2,
            getattr(profile, 'linear_stop_delay_s', 0.))
    vx,vy,wz=command
    linear=math.hypot(vx,vy)
    linear_tail=(linear/profile.brake_deceleration_m_s2+
                 getattr(profile,'linear_stop_delay_s',0.)) if linear>0 else 0.
    return profile.reaction_time_s+max(linear_tail,
                                       abs(wz)/profile.angular_brake_deceleration_rad_s2)


def body_pose(command, t, trig=math):
    if _native is not None and trig is math:
        return tuple(_native.body_pose(list(command), t))
    vx,vy,wz=command;theta=wz*t
    if abs(wz)<1e-6:return vx*t,vy*t,theta
    return ((vx*trig.sin(theta)+vy*(trig.cos(theta)-1))/wz,
            (vx*(1-trig.cos(theta))+vy*trig.sin(theta))/wz,theta)


def sampling_margin(command, profile, step):
    if _native is not None:
        return _native.sampling_margin(
            list(command), profile.half_length_m, profile.half_width_m, step)
    return (math.hypot(*command[:2])+math.hypot(profile.half_length_m,profile.half_width_m)*
            abs(command[2]))*step/2
