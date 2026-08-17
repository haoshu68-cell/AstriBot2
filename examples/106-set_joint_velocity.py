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
File: 106-set_joint_velocity.py
Brief: Control the robot to reach the desired joint velocity in real-time.

Overview
========
1. start a control loop
2. set desired control in each control loop
3. sleep according to the set frequency and then enter the next loop

Key Points
----------
* Note: Joints velocity control is not supported in simulation and will not respond to commands.
"""

from astribot_sdk.core.astribot_api.astribot_client import Astribot
import astribot_ros_middleware as ast_astribot_middleware

if __name__ == '__main__':
    # Connect astribot
    freq = 250.0    # Control frequency of robot (in Hz)
    astribot = Astribot(freq=freq)
    
    # Create rate object
    rate = ast_astribot_middleware.Rate(freq)

    # The setting range of the gripping speed is 0-1000,
    # and its unit is the gripping jaw amplitude/s.
    # The range of the gripping jaw amplitude is 0-100, 100 is fully closed, 0 is fully open.
    close_vel = 200.0
    open_vel = -40.0
    stage = 0
    
    while ast_astribot_middleware.ok():
        position_state = astribot.get_current_joints_position([astribot.effector_right_name])
        if stage == 0:
            vel_list = [[close_vel]]
            if position_state[0][0] > 90.0:
                stage = 1
        elif stage == 1:
            vel_list = [[open_vel]]
            if position_state[0][0] < 10.0:
                break
        astribot.set_joints_velocity([astribot.effector_right_name], vel_list)
        rate.sleep()
