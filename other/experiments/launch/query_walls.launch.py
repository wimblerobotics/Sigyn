#!/usr/bin/env python3
"""
Launch file for querying the wall database.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for querying walls."""
    
    # Wall query node
    wall_query_node = Node(
        package='experiments',
        executable='query_walls',
        name='wall_query',
        output='screen',
        parameters=[],
        remappings=[],
    )
    
    return LaunchDescription([
        wall_query_node,
    ])
