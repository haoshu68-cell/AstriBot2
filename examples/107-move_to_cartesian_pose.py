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
File: 107-move_to_cartesian_pose.py
Brief: Move the robot from its current cartesian pose to the target cartesian pose with blocked motion mode.

Overview
========
1. set the target cartesian pose
2. move to target cartesian pose

Key Points
----------
* Units: position m, quaternion [qx qy qz qw]
* The effector does not have a cartesian pose, use 1D joint position in the move_cartesian_pose function
"""

from astribot_sdk.core.astribot_api.astribot_client import Astribot

if __name__ == '__main__':
    # Connect astribot
    astribot = Astribot()

    astribot.move_to_home()
    
    # The first three-dimensional data of the head, torso, left arm, and right arm are translations, and the unit is m.
    # The last four dimensions are postures, expressed in quaternions, in the order of (qx, qy, qz, qw).
    # The unit of the effector is dimensionless, 100 means fully closed, and 0 means fully open.
    targets = {
        astribot.torso_name          : [ 0.0, 0.0, 1.3, 0.0, 0.0, 0.0, 1.0],
        astribot.arm_left_name       : [ 0.4,  0.4, 1.1, 0.0, 0.0, 0.707, 0.707],
        astribot.arm_right_name      : [ 0.4, -0.4, 1.1, 0.0, 0.0, 0.707, 0.707],
        astribot.effector_left_name  : [50.0],
        astribot.effector_right_name : [50.0],
    }
    names, command_list = zip(*targets.items()) 
    astribot.move_cartesian_pose(list(names), list(command_list), duration=3.0, use_wbc=True)

    print("107 Move to cartesian pose example finished.")
