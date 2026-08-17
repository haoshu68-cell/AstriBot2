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
File: 203-chassis_joy_control_global.py
Brief: Control Astribot chassis with joy velocity command in global coordinate.
The global coordinate system can be defined at the time when the chassis driver starts or when the program starts.

Overview
========
1. start a control loop
2. get the desired control form joy
3. set desired control in each control loop
4. sleep according to the set frequency and then enter the next loop
"""

import numpy as np
import threading
from tools.joy_tools import XboxController
from astribot_sdk.core.astribot_api.astribot_client import Astribot
import astribot_ros_middleware as ast_astribot_middleware

if __name__ == '__main__':
    # Connect astribot
    freq = 250.0    # Control frequency of robot (in Hz)
    astribot = Astribot(freq=freq)

    # Create rate object and joy controller
    rate = ast_astribot_middleware.Rate(freq)
    joy_controller = XboxController(mode='chassis_control')

    use_cur_start_theta_as_reference = True 
    # if use_cur_start_theta_as_reference is True, use the current theta coordinate as the reference, 
    # otherwise use driver's initial theta coordinate as the reference
    cur_theta_start = astribot.get_desired_joints_position([astribot.chassis_name])[0][2]
    pos_cmd = astribot.get_desired_joints_position([astribot.chassis_name])[0]

    def spin_loop():
      while ast_astribot_middleware.ok():
          ast_astribot_middleware.spin()

    spin_thread = threading.Thread(target=spin_loop, daemon=True)
    spin_thread.start()

    while ast_astribot_middleware.ok():
        vel = joy_controller.get_vel()
        cur_theta = astribot.get_desired_joints_position([astribot.chassis_name])[0][2]
        if use_cur_start_theta_as_reference:
            cur_theta = cur_theta - cur_theta_start
        # rot mat from local coordinate to global coordinate
        rot_mat = np.array([[np.cos(cur_theta), -np.sin(cur_theta), 0],
                                  [np.sin(cur_theta), np.cos(cur_theta), 0],
                                  [0, 0, 1]])
        vel_cmd = rot_mat.T @ vel # rotate global velocity command to local coordinate

        pos_cmd[0] += vel_cmd[0] / freq
        pos_cmd[1] += vel_cmd[1] / freq
        pos_cmd[2] += vel_cmd[2] / freq
        astribot.set_joints_position([astribot.chassis_name], [pos_cmd])
        # ast_astribot_middleware.spin()
        rate.sleep()
