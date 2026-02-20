#!/usr/bin/env python3
"""
Launch file for the wall extraction process.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for wall extraction."""
    
    # Wall extractor node
    wall_extractor_node = Node(
        package='experiments',
        executable='extract_walls',
        name='wall_extractor',
        output='screen',
        parameters=[],
        remappings=[],
    )
    
    return LaunchDescription([
        wall_extractor_node,
    ])
