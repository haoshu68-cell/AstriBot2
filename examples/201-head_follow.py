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
File: 201-head_follow.py
Brief: Set the head control mode to track the center of the dual arm effector

Overview
========
1. set the head control mode to track the center of the dual arm effector
2. move arms to observe the phenomenon

Key Points
----------
* After setting the head to follow the effector mode, the head will no longer respond to any other control signals until it is reset to False.
"""

from astribot_sdk.core.astribot_api.astribot_client import Astribot

if __name__ == '__main__':
    # Connect astribot
    astribot = Astribot()
    astribot.move_to_home()
    astribot.set_head_follow_effector(True)

    names = [astribot.torso_name, astribot.arm_left_name, astribot.arm_right_name]
    torso_cartesian_pose = astribot.get_current_cartesian_pose(names=[astribot.torso_name])
    left_arm_cartesian_pose = astribot.get_current_cartesian_pose(names=[astribot.arm_left_name])
    right_arm_cartesian_pose = astribot.get_current_cartesian_pose(names=[astribot.arm_right_name])

    torso_cartesian_pose = torso_cartesian_pose[0]
    left_arm_cartesian_pose = left_arm_cartesian_pose[0]
    right_arm_cartesian_pose = right_arm_cartesian_pose[0]

    left_arm_cartesian_pose[0] -= 0.08
    right_arm_cartesian_pose[0] -= 0.08
    command_list = [torso_cartesian_pose, left_arm_cartesian_pose, right_arm_cartesian_pose]
    astribot.move_cartesian_pose(names, command_list, duration=1.0, use_wbc=True)

    for idx in range(4):
        if idx % 2 == 0:
            left_arm_cartesian_pose[0] += 0.2
            right_arm_cartesian_pose[0] += 0.2
        else:
            left_arm_cartesian_pose[0] -= 0.2
            right_arm_cartesian_pose[0] -= 0.2
        command_list = [torso_cartesian_pose, left_arm_cartesian_pose, right_arm_cartesian_pose]
        astribot.move_cartesian_pose(names, command_list, duration=1.1, use_wbc=True)
    
    astribot.set_head_follow_effector(False)
