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
File: 300-get_images.py
Brief:  Example code for Astribot Robotics.
        Get real-time images data from the robot camera.
"""

import os
import cv2
import numpy as np
from astribot_sdk.core.astribot_api.astribot_client import Astribot
import time

# global variable
camera_name_list = [
    "head_rgbd",
    "torso_rgbd",
    "left_wrist_rgbd",
    "right_wrist_rgbd",
    "head_stereo"  #head_stereo camera did not support depth image type
]

# support image type
image_type = [
    "depth",
    "color"
]

"""
    Image callback function for processing the camera image data received from the astribot camera abs driver.

    This function takes in a ROS CompressImage message and a 1D/3D numpy array containing the image data.

    Args:
        msg (CompressImage): The CompressImage message import from ROS, base usage is list as below:
          msg.format:      "jpeg" when a color image and "raw" when a depth image
          msg.size:        total size of data
        
        height:    height of image if need_decode is set as True, or 0
        width:     width of image if need_decode is set as True, or 0
        array:     The numpy array containing the image data.
                   If need_decode set as True, array is a 3D numpy array containing BGR
                   If need_decode set as False, array is a 1D numpy array containing JPEG

    Returns:
        None
"""
def image_callback(topic_name, msg, width, height, array: np.ndarray):
        if msg.format.lower() == "jpeg":
            # Handle BGR image
            # If enable_decode set as True when call register_image_callback，array's shape will be BGR(HWC).
            # If enable_decode set as False, array will remain as jpeg.
            parts = topic_name.split('/')
            camera_name = parts[2]
            # print(f"receive jpeg image from {camera_name}")
            cv2.imshow(camera_name, array)
            cv2.waitKey(1)

if __name__ == '__main__':
    astribot = Astribot()
    astribot.activate_camera()

    target_camera = "head_rgbd"
    #check if head_rgbd camera is already activated
    cameras_stat = astribot.get_cameras_info()
    if cameras_stat[target_camera]["activate"] != True:
        total_seconds = 10
        print(f"Waiting for camera module activate for {total_seconds} seconds ", end="", flush=True)
    
        for _ in range(total_seconds):
            cameras_stat = astribot.get_cameras_info()
            if cameras_stat[target_camera]["activate"] == True:
                break
            print(".", end="", flush=True)
        
        print("\n Waiting end!")

    # get cameras activate state
    cameras_stat = astribot.get_cameras_info()
    print(f"cameras status: {cameras_stat}")

    if cameras_stat[target_camera]["activate"] != True:
        print(f"Can not activate camera {target_camera}")
        os._exit(1)

    # register as a subscriber to get real-time image of head
    subscriber = astribot.register_image_callback(target_camera, "color", image_callback, need_decode=True)
   
    input("Press any key to close the camera...")
    robot_shutdown()
    cv2.destroyAllWindows()
    
