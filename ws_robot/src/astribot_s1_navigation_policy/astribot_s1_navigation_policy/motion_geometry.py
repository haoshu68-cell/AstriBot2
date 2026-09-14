"""Body-frame constant-twist envelope shared by prediction and final protection."""
import math


def stopping_horizon(command, profile):
    vx,vy,wz=command
    return profile.reaction_time_s+max(math.hypot(vx,vy)/profile.brake_deceleration_m_s2,
                                       abs(wz)/profile.angular_brake_deceleration_rad_s2)


def body_pose(command, t, trig=math):
    vx,vy,wz=command;theta=wz*t
    if abs(wz)<1e-6:return vx*t,vy*t,theta
    return ((vx*trig.sin(theta)+vy*(trig.cos(theta)-1))/wz,
            (vx*(1-trig.cos(theta))+vy*trig.sin(theta))/wz,theta)


def sampling_margin(command, profile, step):
    return (math.hypot(*command[:2])+math.hypot(profile.half_length_m,profile.half_width_m)*
            abs(command[2]))*step/2
