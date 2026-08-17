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
File: 205-inverse_kinematics.py
Brief: Get inverse kinematics result of astribot.
"""

from astribot_sdk.core.astribot_api.astribot_client import Astribot

if __name__ == '__main__':
    # Connect astribot
    astribot = Astribot()

    # Acceptable names include: elbows, arms and torso
    # if you want to input elbows' pose, please use astribot.elbow_names / astribot.elbow_left_name / astribot.elbow_right_name
    names = [astribot.torso_name, astribot.arm_left_name]
    cartesian_pose = [[-0.2, 0.0, 1.317, 0.0, 0.0, 0.0, 1.0],
                      [-0.2, 0.2, 0.55, 0.707, 0.0, 0.0, 0.7071]]

    ik_flag, joints_position_result = astribot.get_inverse_kinematics(names, cartesian_pose)
    print("Inverse kinematics flag: ", ik_flag)
    print("Inverse kinematics result: ", joints_position_result)
