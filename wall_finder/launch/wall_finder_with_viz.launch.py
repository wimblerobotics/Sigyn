#!/usr/bin/env python3
"""
Complete launch file for wall_finder with visualization.

This launch file starts the wall_finder node and optionally RViz2 for visualization.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to the package
    package_dir = get_package_share_directory('wall_finder')
    
    # Path to the default config file
    default_config_file = os.path.join(package_dir, 'config', 'wall_finder.yaml')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the wall_finder configuration file'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz2 for visualization'
    )
    
    costmap_topic_arg = DeclareLaunchArgument(
        'costmap_topic',
        default_value='/global_costmap/static_layer',
        description='Topic name for the input costmap'
    )
    
    # Wall finder node
    wall_finder_node = Node(
        package='wall_finder',
        executable='wall_finder',
        name='wall_finder',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    # RViz2 node (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', 'default']  # You can create a custom RViz config file later
    )
    
    return LaunchDescription([
        config_file_arg,
        use_rviz_arg,
        costmap_topic_arg,
        wall_finder_node,
        rviz_node
    ])
