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
File: 109-effector_open_close.py
Brief: Set the maximum gripping force of the effector and open and close the effector

Overview
========
1. set the maximum holding force of the effectors
2. close the effectors
3. open the effectors

Key Points
----------
* If you do not set the holding force, you can also control the opening and closing, and it will run at the default value 48N
"""

import time
import threading
from astribot_sdk.core.astribot_api.astribot_client import Astribot

if __name__ == '__main__':
    # Connect astribot
    freq = 250.0    # Control frequency of robot (in Hz)
    dt = 1.0 / freq
    astribot = Astribot()

    for idx in range(4):
        if idx % 2 == 0:
            # set gripper max force, max_force is changed by each grasp
            # the minimum clamping force is 10N, the maximum clamping force varies with the robot mode, 40N in safe mode, 60N in standard mode, and 80N in extreme mode.
            astribot.set_effector_max_force(astribot.effector_names, max_force=[40, 40])
            astribot.close_effector(names=astribot.effector_names, duration=1.0)
        else:
            astribot.open_effector(names=astribot.effector_names, duration=1.0)

        time.sleep(1.0)
