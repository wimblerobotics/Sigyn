#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('perimeter_roamer_v3')
    
    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for the waypoint capture node'
    )
    
    # Waypoint capture node
    waypoint_capture_node = Node(
        package='perimeter_roamer_v3',
        executable='capture_waypoints.py',
        name='waypoint_capture',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time', default=True)
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    return LaunchDescription([
        log_level_arg,
        waypoint_capture_node
    ])
