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
File: 202-chassis_joy_control_local.py
Brief: Use joy to control the chassis movement

Overview
========
1. start a control loop
2. get the desired control form joy
3. set desired control in each control loop
4. sleep according to the set frequency and then enter the next loop
"""

import threading
from tools.joy_tools import XboxController
from astribot_sdk.core.astribot_api.astribot_client import Astribot
import astribot_ros_middleware as ast_astribot_middleware

if __name__ == '__main__':
    # Connect astribot
    freq = 250.0    # Control frequency of robot (in Hz)
    astribot = Astribot(freq=freq)

    # Create rate object
    rate = ast_astribot_middleware.Rate(freq)
    joy_controller = XboxController(mode='chassis_control')

    def spin_loop():
      while ast_astribot_middleware.ok():
          ast_astribot_middleware.spin()

    spin_thread = threading.Thread(target=spin_loop, daemon=True)
    spin_thread.start()

    pos_cmd = astribot.get_desired_joints_position([astribot.chassis_name])[0]
    while ast_astribot_middleware.ok():
        vel_cmd = joy_controller.get_vel()
        pos_cmd[0] += vel_cmd[0] / freq
        pos_cmd[1] += vel_cmd[1] / freq
        pos_cmd[2] += vel_cmd[2] / freq
        astribot.set_joints_position([astribot.chassis_name], [pos_cmd])
        # ast_astribot_middleware.spin()
        rate.sleep()
