#!/usr/bin/env python3
"""
Simple launch file for the wall_finder node.

This is a minimal launch file that starts the wall_finder node with default settings.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Wall finder node with default parameters
    wall_finder_node = Node(
        package='wall_finder',
        executable='wall_finder',
        name='wall_finder',
        output='screen',
        parameters=[{
            'costmap_topic': '/global_costmap/static_layer',
            'publish_walls_costmap': True,
            'database_path': 'walls.db',
            'costmap_threshold': 50,
            'min_wall_length_inches': 2.0,
            'min_wall_length_meters': 0.051,
            'hough_threshold': 30,
            'hough_min_line_length': 15,
            'hough_max_line_gap': 15,
            'angle_tolerance': 0.174532925,  # 10 degrees
            'distance_tolerance': 0.2,        # 20 cm
            'proximity_tolerance': 0.15,      # 15 cm
        }]
    )
    
    return LaunchDescription([
        wall_finder_node
    ])
