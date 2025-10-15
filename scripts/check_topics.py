#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn

"""
Quick script to check available IMU and camera topics
"""

import rclpy
from rclpy.node import Node
import subprocess
import sys

def main():
    print("Checking available IMU and camera topics...")
    print("=" * 50)
    
    try:
        # Get topic list
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            
            # Filter for IMU and camera related topics
            imu_topics = [t for t in topics if 'imu' in t.lower()]
            camera_topics = [t for t in topics if any(keyword in t.lower() 
                           for keyword in ['camera', 'oakd', 'rgb', 'stereo', 'depth'])]
            
            print("IMU Topics:")
            if imu_topics:
                for topic in imu_topics:
                    print(f"  {topic}")
            else:
                print("  No IMU topics found")
            
            print("\nCamera Topics:")
            if camera_topics:
                for topic in camera_topics[:10]:  # Limit to first 10
                    print(f"  {topic}")
                if len(camera_topics) > 10:
                    print(f"  ... and {len(camera_topics) - 10} more")
            else:
                print("  No camera topics found")
                
            print(f"\nTotal topics: {len(topics)}")
            
        else:
            print("Error: Could not get topic list. Is ROS2 running?")
            print("Try: source /opt/ros/jazzy/setup.bash && source /home/ros/sigyn_ws/install/setup.bash")
            
    except subprocess.TimeoutExpired:
        print("Timeout: ROS2 command took too long. Is the system running?")
    except FileNotFoundError:
        print("Error: ros2 command not found. Is ROS2 installed?")

if __name__ == '__main__':
    main()