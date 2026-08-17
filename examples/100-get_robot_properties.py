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
File: 100-get_robot_properties.py
Brief: Print some properties of astribot,

Overview
========
1. Print normal info
2. Print dofs of each part of the astribot
3. Print joints position limit of each part of the astribot
4. Print joints velocity limit of each part of the astribot
5. Print joints torque limit of each part of the astribot

Key Points
----------
* Units: joints position 'rad', joints velocity 'rad/s', joints torque 'Nm'

Example Output
--------------
.. code-block:: text

   Astribot parts' name       Astribot parts joint dof
   ---------------------------------------------------
   astribot_torso                      4
"""

import time
from tabulate import tabulate
from astribot_sdk.core.astribot_api.astribot_client import Astribot

# Connect astribot
astribot = Astribot()
astribot.get_info()

dofs = astribot.get_dof()
dof_table = [[astribot.whole_body_names[i], dofs[i]] for i in range(len(astribot.whole_body_names))]
headers = ["Astribot parts' name", "Astribot parts joint dof"]
print(tabulate(dof_table, headers=headers, tablefmt="grid"))

upper_limit, lower_limit = astribot.get_joints_position_limit()
velocity_limit = astribot.get_joints_velocity_limit()
torque_limit = astribot.get_joints_torque_limit()
headers = ["Astribot parts' name", "Astribot parts joint upper limit",
           "Astribot parts joint lower limit", "Astribot parts joint velocity limit",
           "Astribot parts joint torque limit"]
limits_table = []
for i in range(len(astribot.whole_body_names)):
    limits_table.append([astribot.whole_body_names[i], upper_limit[i], lower_limit[i], velocity_limit[i], torque_limit[i]])
print(tabulate(limits_table, headers=headers, tablefmt="grid"))
