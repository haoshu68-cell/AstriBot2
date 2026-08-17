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
File: 204-forward_kinematics.py
Brief: Get forward kinematics result of astribot.
"""

from astribot_sdk.core.astribot_api.astribot_client import Astribot

if __name__ == '__main__':
    # Connect astribot
    astribot = Astribot()

    names = [astribot.head_name, astribot.torso_name, astribot.arm_left_name, astribot.arm_right_name]
    joints_position = [[0.5, -0.2],
                       [0.275, -0.55, 0.275, 0.0],
                       [-0.09622, -0.4218, -1.1273, 1.6168, -0.4149, 0.0645,  0.4225],
                       [0.09622, -0.4218,  1.1273, 1.6168,  0.4149, 0.0645, -0.4225]]
    cartesian_pose_result = astribot.get_forward_kinematics(names, joints_position)
    print("Forward kinematics result: ", cartesian_pose_result)
