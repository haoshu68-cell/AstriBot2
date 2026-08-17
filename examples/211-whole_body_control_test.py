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
File: 211-whole_body_control_test.py
Brief: Demonstrates whole body control function.

Overview
========
1. start a control loop
2. the current position is used as the expectation, and wbc control is turned on
3. sleep according to the set frequency and then enter the next loop

Key Points
----------
* In this mode, you can try to push the robot's torso. Even if the robot's torso moves for any reason, the arms can still ensure stable end tracking.
"""
import time
from astribot_sdk.core.astribot_api.astribot_client import Astribot
import astribot_ros_middleware as ast_astribot_middleware

if __name__ == '__main__':
    # Connect astribot
    freq = 250.0    # Control frequency of robot (in Hz)
    dt = 1.0 / freq
    astribot = Astribot(freq=freq)

    # Create rate object
    rate = ast_astribot_middleware.Rate(freq)

    # astribot.move_to_home()

    names = [astribot.torso_name, astribot.arm_left_name, astribot.arm_right_name]
    command_list = astribot.get_desired_cartesian_pose(names=names)

    print("#################################################################\n"
          "##    WBC Demo: try gently pushing the torso; the arms will    ##\n"
          "##    keep their end-effectors on target.                      ##\n"
          "#################################################################\n")

    try:
        while not ast_astribot_middleware.is_shutdown():
            astribot.set_cartesian_pose(names, command_list, control_way="direct", use_wbc=True)

            # Use ROS2 rate instead of time.sleep
            rate.sleep()
    except KeyboardInterrupt:
        print("Stopping whole body control...")
    finally:
        ast_astribot_middleware.shutdown()
