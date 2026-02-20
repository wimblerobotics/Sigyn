#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('sigyn_house_patroller')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for battery monitor'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sigyn_house_patroller'),
            'config',
            'battery_monitor.yaml'
        ]),
        description='Path to battery monitor configuration file'
    )
    
    # Battery monitor node
    battery_monitor_node = Node(
        package='sigyn_house_patroller',
        executable='battery_monitor_node',
        name='battery_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'critical_battery_level': 0.15},
            {'low_battery_level': 0.25},
            {'monitoring_frequency': 2.0},
            {'battery_timeout': 30.0},
            {'battery_topic': '/battery_state'},
            {'enable_predictions': True},
            {'prediction_window': 300.0},
            LaunchConfiguration('config_file'),
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('/battery_state', '/battery_state'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        config_file_arg,
        battery_monitor_node,
    ])
