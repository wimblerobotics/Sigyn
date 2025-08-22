#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments
    enable_motion_correction_arg = DeclareLaunchArgument(
        'enable_motion_correction',
        default_value='true',
        description='Enable motion correction for LIDAR data during robot movement'
    )
    
    use_config_file_arg = DeclareLaunchArgument(
        'use_config_file',
        default_value='multi_lidar_with_motion_correction.yaml',
        description='Config file to use (multi_lidar_config.yaml or multi_lidar_with_motion_correction.yaml)'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sigyn_lidar_v2'),
            'config',
            LaunchConfiguration('use_config_file')
        ]),
        description='Full path to configuration file'
    )
    
    # LIDAR node with motion correction
    lidar_node = Node(
        package='sigyn_lidar_v2',
        executable='sigyn_lidar_v2_node',
        name='multi_lidar_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'enable_motion_correction': LaunchConfiguration('enable_motion_correction'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        enable_motion_correction_arg,
        use_config_file_arg,
        config_file_arg,
        lidar_node,
    ])
