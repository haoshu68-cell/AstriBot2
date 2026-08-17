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
File: 207-move_cartesian_waypoints.py
Brief: Follow sequence of cartesian waypoints.

Overview
========
1. set a series of waypoints and arrival times
2. call the function to reach the path points in sequence within the set time

Key Points
----------
* No need to give a point at time 0, the default time is 0 at the current position
"""


from astribot_sdk.core.astribot_api.astribot_client import Astribot

if __name__ == '__main__':
    # Connect astribot
    freq = 250.0    # Control frequency of robot (in Hz)
    dt = 1.0 / freq
    astribot = Astribot(freq=freq)
    astribot.move_to_home()

    names = [astribot.torso_name, astribot.arm_left_name, astribot.arm_right_name]
    desired_pose = astribot.get_desired_cartesian_pose(names=names)
    waypoints = list()
    time_list = list()
    waypoints.append([[ 0.0, 0.0, 1.2, 0.0, 0.0, 0.0, 1.0], desired_pose[1], desired_pose[2]])
    time_list.append(1.5)
    waypoints.append([desired_pose[0], desired_pose[1], desired_pose[2]])
    time_list.append(3.0)
    waypoints.append([[ 0.0, 0.0, 1.2, 0.0, 0.0, 0.0, 1.0], desired_pose[1], desired_pose[2]])
    time_list.append(4.5)
    waypoints.append([desired_pose[0], desired_pose[1], desired_pose[2]])
    time_list.append(6.0)

    astribot.move_cartesian_waypoints(names, waypoints, time_list, use_wbc=True)
    astribot.move_to_home()
