#!/usr/bin/env python3
"""
Launch file for the wall_finder node.

This launch file starts the wall_finder node with the default configuration
and automatically loads parameters from the YAML config file.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    
    costmap_topic_arg = DeclareLaunchArgument(
        'costmap_topic',
        default_value='/global_costmap/static_layer',
        description='Topic name for the input costmap'
    )
    
    publish_walls_costmap_arg = DeclareLaunchArgument(
        'publish_walls_costmap',
        default_value='true',
        description='Whether to publish detected walls as a costmap for visualization'
    )
    
    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value='walls.db',
        description='Path to the SQLite database file for storing wall data'
    )
    
    # Wall finder node
    wall_finder_node = Node(
        package='wall_finder',
        executable='wall_finder',
        name='wall_finder',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'costmap_topic': LaunchConfiguration('costmap_topic'),
                'publish_walls_costmap': LaunchConfiguration('publish_walls_costmap'),
                'database_path': LaunchConfiguration('database_path'),
            }
        ],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        costmap_topic_arg,
        publish_walls_costmap_arg,
        database_path_arg,
        wall_finder_node
    ])
