#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# Copyright (c) 2025, Astribot Co., Ltd.
# All rights reserved.
# License: BSD 3-Clause License
# -----------------------------------------------------------------------------

"""
File: 301-get_lidar_scan_refactored.py
Brief: Example code for getting real-time LiDAR scan data using ROS Topics
       and the Astribot SDK (Refactored).
"""

import time
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2

# SDK Import
from astribot_sdk.core.astribot_api.astribot_client import Astribot

class LidarMonitor:
    def __init__(self):
        # 1. Initialize SDK and reuse the existing Node
        self.astribot = Astribot()
        self.node = self.astribot.astribot_interface.node
        
        # 2. Internal statistics variables
        self.frame_count = 0
        self.last_print_time = time.time()
        
        # 3. Configure QoS Profile
        # LiDAR data usually requires 'Best Effort' reliability due to high bandwidth.
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # 4. ROS Topic Subscriptions
        # Subscribe to Front LiDAR
        self.sub_front = self.node.create_subscription(
            PointCloud2,
            '/livox/lidar_front',
            self.lidar_callback,
            qos_profile
        )
        
        # Subscribe to Back LiDAR
        self.sub_back = self.node.create_subscription(
            PointCloud2,
            '/livox/lidar_back',
            self.lidar_callback,
            qos_profile
        )
        
        self.start_hardware()

    def start_hardware(self):
        """Activate the LiDAR hardware via SDK."""
        
        print("Ensuring LiDAR is deactivated first...")
        self.astribot.deactivate_lidar()
        time.sleep(1.0) # Short wait

        print("Activating LiDAR...")
        result = self.astribot.activate_lidar()
        if result:
            print("LiDAR activated successfully.")
        else:
            print("Failed to activate LiDAR.")

    def lidar_callback(self, msg):
        """
        Callback function for PointCloud2 messages.
        Calculates and prints FPS statistics.
        """
        self.frame_count += 1
        current_time = time.time()
        
        # Print stats every 1 second
        if current_time - self.last_print_time >= 1.0:
            # msg.width typically represents the number of points in an unorganized cloud
            print(f"[LiDAR Stats] FPS: {self.frame_count}, Points in last frame: {msg.width}")
            
            # Reset counters
            self.frame_count = 0
            self.last_print_time = current_time

    def close(self):
        """Clean up resources."""
        print("Deactivating LiDAR Hardware...")
        self.astribot.deactivate_lidar()


def main():
    # 1. Instantiate the monitor class
    lidar_monitor = LidarMonitor()
    print("Listening for LiDAR data... Press Ctrl+C to stop.")

    # 2. Keep the node running to process callbacks
    # Using spin() here because we want continuous monitoring until interruption
    rclpy.spin(lidar_monitor.node)
        
    

if __name__ == '__main__':
    main()