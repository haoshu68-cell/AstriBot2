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
File: 102_get_cartesian_states.py
Brief: Print two sets of Cartesian poses
       include current (world frame) and desired (chassis frame).

Overview
========
1. current pose of each main part in **world** frame
2. desired pose in **chassis** frame

Key Points
----------
* Units: position m, quaternion [qx qy qz qw]

Example Output
--------------
.. code-block:: text

   Astribot parts' name       x(m)    y(m)    z(m)   qx     qy     qz     qw
   ---------------------------------------------------------------------------
   astribot_torso           +0.000  +0.000  +1.300 +0.000 +0.000 +0.000 +1.000
"""

from tabulate import tabulate
from astribot_sdk.core.astribot_api.astribot_client import Astribot

if __name__ == '__main__':
    # Connect astribot
    astribot = Astribot()
    current_cartesian_pose = astribot.get_current_cartesian_pose(frame=astribot.world_frame_name)
    desired_cartesian_pose = astribot.get_desired_cartesian_pose(frame=astribot.chassis_frame_name)

    headers = ["Astribot parts' name", "x(m)", "y(m)", "z(m)", "qx", "qy", "qz", "qw"]
    current_cartesian_table = []
    desired_cartesian_table = []
    for i in range(len(astribot.whole_body_names)):
        current_cartesian_table.append([astribot.whole_body_names[i]+"(current)"] + current_cartesian_pose[i])
        desired_cartesian_table.append([astribot.whole_body_names[i]+"(desired)"] + desired_cartesian_pose[i])
    print(tabulate(current_cartesian_table, headers=headers, tablefmt="grid"))
    print(tabulate(desired_cartesian_table, headers=headers, tablefmt="grid"))
