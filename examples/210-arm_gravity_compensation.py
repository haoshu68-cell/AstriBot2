#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# Copyright (c) 2024, Astribot Co., Ltd.
# All rights reserved.
# License: BSD 3-Clause License
# -----------------------------------------------------------------------------
# Author: Astribot Team
# -----------------------------------------------------------------------------

"""
File: 210-arm_gravity_compensation.py
Brief: The right arm is set to gravity compensation mode

Key Points
----------
* Note: Joints torque control is not supported in simulation and will not respond to commands.
"""

from astribot_sdk.core.astribot_api.astribot_client import Astribot
import astribot_ros_middleware as ast_astribot_middleware

if __name__ == '__main__':
    # Connect astribot
    freq = 250.0    # Control frequency of robot (in Hz)
    dt = 1.0 / freq
    astribot = Astribot(freq=freq)

    # Create rate object
    rate = ast_astribot_middleware.Rate(freq)

    names = [astribot.arm_right_name]
    torque_cmd = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    print("###########################################################################\n"
        "##    ⚠ CHECK robot safe, covers & grippers installed, arm unloaded ⚠    ##\n"
        "##              >>>  Press Enter to enable Zero-G  <<<                   ##\n"
        "###########################################################################\n")
    input()
    print("########################################################################\n"
        "##  Zero-G active — gently move the right arm and feel free motion.   ##\n"
        "########################################################################\n")
    
    lower_limit, upper_limit = astribot.get_joints_position_limit(names)
    while ast_astribot_middleware.ok():
        astribot.set_joints_torque(names, torque_cmd)

        # speed ​​protection and position over limit protection
        speed = astribot.get_current_joints_velocity(names=names)
        joints_position = astribot.get_current_joints_position(names=names)
        result = any(a < b for row1, row2 in zip(joints_position, lower_limit) for a, b in zip(row1, row2)) \
                 or any(a > b for row1, row2 in zip(joints_position, upper_limit) for a, b in zip(row1, row2))
        if any(abs(v) > 2.0 for v in speed[0]):
            print("Joint speed exceeds limit, stopping movement.")
            astribot.stop_robot()
            break
        if result:
            print("Joint position exceeded limit, stopping movement.")
            astribot.stop_robot()
            break
        rate.sleep()
