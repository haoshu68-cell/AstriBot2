#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# Copyright (c) 2025, Astribot Co., Ltd.
# All rights reserved.
# License: BSD 3-Clause License
# -----------------------------------------------------------------------------

"""
File: 302-ros_audio_record_play.py
Brief: Example code for recording and playing back audio using ROS Topics
       and the Astribot SDK.
"""

import time
import rclpy
from std_msgs.msg import Float64MultiArray
from astribot_sdk.core.astribot_api.astribot_client import Astribot

class AudioRecorderPlayer:
    def __init__(self):
        # 1. Initialize SDK and reuse the existing Node
        # Astribot() initializes the ROS node internally
        self.astribot = Astribot()
        self.node = self.astribot.astribot_interface.node
        
        # 2. Internal variables
        self.audio_buffer = []      # Buffer to store recorded audio frames
        self.is_recording = False   # Recording flag
        
        # 3. ROS Topic Settings
        # Subscribe to microphone input
        self.mic_sub = self.node.create_subscription(
            Float64MultiArray,
            '/astribot_audio/microphone/stream',
            self.mic_callback,
            10
        )
        
        # Publisher for speaker output
        self.speaker_pub = self.node.create_publisher(
            Float64MultiArray, 
            '/astribot_audio/speaker/stream', 
            10
        )
        
        # 4. Activate Audio Hardware
        print("Activating Audio Hardware...")
        self.astribot.activate_audio()
        # Wait a moment for hardware to be ready
        time.sleep(1.0) 

    def mic_callback(self, msg):
        """Microphone callback: Save data only when recording is enabled."""
        if self.is_recording:
            self.audio_buffer.append(msg)

    def record_audio(self, duration=5.0):
        """Record audio for a specified duration."""
        print(f"Start recording for {duration} seconds...")
        
        self.audio_buffer = []  # Clear buffer
        self.is_recording = True
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            # Key step: Manually spin to receive callback data
            # since we are not in a global spin loop.
            rclpy.spin_once(self.node, timeout_sec=0.01)
            
        self.is_recording = False
        print(f"Recording finished. Captured {len(self.audio_buffer)} frames.")

    def play_audio(self):
        """Publish the buffered audio via Topic."""
        if not self.audio_buffer:
            print("No audio data to play!")
            return

        print("Start playing...")
        
        # Iterate through buffer and publish
        for msg in self.audio_buffer:
            self.speaker_pub.publish(msg)
            # Simple flow control: Add a tiny delay to prevent congestion
            # If the audio stutters or plays too fast, adjust this sleep time (e.g., 0.01 or 0.02) or write a better loop to control the playback speed
            time.sleep(0.01) 
            
        print("Playback finished.")

    def close(self):
        """Clean up resources."""
        print("Deactivating Audio Hardware...")
        self.astribot.deactivate_audio()


def main():
    # 1. Instantiate our audio operation class
    audio_task = AudioRecorderPlayer()
    
    for i in range(100):
        temp=input(f"Press Enter to record {i} time, and q to quit")
        if temp == "q":
            break
        
        # 2. Record for 5 seconds
        print("Record 5 seconds...")
        audio_task.record_audio(5)
        
        # 3. Wait briefly (Optional)
        time.sleep(1)
        
        # 4. Play back the recorded audio
        print("Playback...")
        audio_task.play_audio()
    
    # 5. Clean up and deactivate hardware
    audio_task.close()

if __name__ == '__main__':
    main()