#!/usr/bin/env python3
"""
Launch file for the wall database visualizer.

This launch file starts the visualize_db node which reads the wall database
and allows interactive navigation through walls with costmap visualization.
The node will automatically get costmap parameters from the global costmap.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate the launch description for wall database visualization."""
    
    # Declare launch arguments
    costmap_value_arg = DeclareLaunchArgument(
        'costmap_value',
        default_value='50',
        description='Value to use for occupied cells in costmap (0-100)'
    )
    
    visualize_db_node = Node(
        package='experiments',
        executable='visualize_db',
        name='wall_visualizer',
        output='screen',
        parameters=[{
            'costmap_value': LaunchConfiguration('costmap_value'),
        }],
        emulate_tty=True,  # This helps with stdin/stdout handling
    )
    
    return LaunchDescription([
        costmap_value_arg,
        visualize_db_node,
    ])
