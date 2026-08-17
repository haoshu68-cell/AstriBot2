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
File: 101-get_joint_states.py
Brief: Print six sets of Cartesian poses, include current joints position,
       desired joints position, current joints velocity, desired joints velocity,
       current joints torque, desired joints torque.

Overview
========
1. current joints position of each part of the astribot
2. desired joints position of each part of the astribot
3. current joints velocity of each part of the astribot
4. desired joints velocity of each part of the astribot
5. current joints torque of each part of the astribot
6. desired joints torque of each part of the astribot

Key Points
----------
* Units: joints position 'rad', joints velocity 'rad/s', joints torque 'Nm'

Example Output
--------------
.. code-block:: text

   Astribot parts' name  1st joint current position 2nd joint current position 3rd joint current position 4th joint current position 5th joint current position 6th joint current position 7th joint current position"
   --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   astribot_torso                  +0.000                      +0.000                  +0.000                         +0.000                    +0.000                    +0.000                    +0.000
"""

from tabulate import tabulate
from astribot_sdk.core.astribot_api.astribot_client import Astribot

if __name__ == '__main__':
    # Connect astribot
    astribot = Astribot()
    current_joints_position = astribot.get_current_joints_position()
    desired_joints_position = astribot.get_desired_joints_position()
    current_joints_velocity = astribot.get_current_joints_velocity()
    desired_joints_velocity = astribot.get_desired_joints_velocity()
    current_joints_torque = astribot.get_current_joints_torque()
    desired_joints_torque = astribot.get_desired_joints_torque()

    headers = ["Astribot parts' name", "1st joint current position", "2nd joint current position", "3rd joint current position", 
               "4th joint current position", "5th joint current position", "6th joint current position", "7th joint current position"]
    current_position_table = []
    for i in range(len(astribot.whole_body_names)):
        current_position_table.append([astribot.whole_body_names[i]] + current_joints_position[i])
    print(tabulate(current_position_table, headers=headers, tablefmt="grid"))

    headers = ["Astribot parts' name", "1st joint desired position", "2nd joint desired position", "3rd joint desired position", 
               "4th joint desired position", "5th joint desired position", "6th joint desired position", "7th joint desired position"]
    desired_position_table = []
    for i in range(len(astribot.whole_body_names)):
        desired_position_table.append([astribot.whole_body_names[i]] + desired_joints_position[i])
    print(tabulate(desired_position_table, headers=headers, tablefmt="grid"))

    headers = ["Astribot parts' name", "1st joint current velocity", "2nd joint current velocity", "3rd joint current velocity", 
               "4th joint current velocity", "5th joint current velocity", "6th joint current velocity", "7th joint current velocity"]
    current_velocity_table = []
    for i in range(len(astribot.whole_body_names)):
        current_velocity_table.append([astribot.whole_body_names[i]] + current_joints_velocity[i])
    print(tabulate(current_velocity_table, headers=headers, tablefmt="grid"))

    headers = ["Astribot parts' name", "1st joint desired velocity", "2nd joint desired velocity", "3rd joint desired velocity", 
               "4th joint desired velocity", "5th joint desired velocity", "6th joint desired velocity", "7th joint desired velocity"]
    desired_velocity_table = []
    for i in range(len(astribot.whole_body_names)):
        desired_velocity_table.append([astribot.whole_body_names[i]] + desired_joints_velocity[i])
    print(tabulate(desired_velocity_table, headers=headers, tablefmt="grid"))

    headers = ["Astribot parts' name", "1st joint current torque", "2nd joint current torque", "3rd joint current torque", 
               "4th joint current torque", "5th joint current torque", "6th joint current torque", "7th joint current torque"]
    current_torque_table = []
    for i in range(len(astribot.whole_body_names)):
        current_torque_table.append([astribot.whole_body_names[i]] + current_joints_torque[i])
    print(tabulate(current_torque_table, headers=headers, tablefmt="grid"))

    headers = ["Astribot parts' name", "1st joint desired torque", "2nd joint desired torque", "3rd joint desired torque", 
               "4th joint desired torque", "5th joint desired torque", "6th joint desired torque", "7th joint desired torque"]
    desired_torque_table = []
    for i in range(len(astribot.whole_body_names)):
        desired_torque_table.append([astribot.whole_body_names[i]] + desired_joints_torque[i])
    print(tabulate(desired_torque_table, headers=headers, tablefmt="grid"))
