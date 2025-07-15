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
        description='Log level for temperature monitor'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sigyn_house_patroller'),
            'config',
            'temperature_monitor.yaml'
        ]),
        description='Path to temperature monitor configuration file'
    )
    
    # Temperature monitor node
    temperature_monitor_node = Node(
        package='sigyn_house_patroller',
        executable='temperature_monitor_node',
        name='temperature_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'anomaly_threshold': 5.0},
            {'monitoring_frequency': 1.0},
            {'temperature_timeout': 60.0},
            {'temperature_topic': '/temperature'},
            {'history_size': 100},
            {'baseline_learning_samples': 50},
            {'alert_cooldown': 300.0},
            {'enable_trend_analysis': True},
            {'trend_window': 600.0},
            # Room baseline temperatures
            {'room_baselines.living_room': 22.0},
            {'room_baselines.kitchen': 20.0},
            {'room_baselines.bedroom_1': 21.0},
            {'room_baselines.bedroom_2': 20.0},
            {'room_baselines.bathroom': 23.0},
            {'room_baselines.hallway': 21.0},
            {'room_baselines.entry': 20.0},
            LaunchConfiguration('config_file'),
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('/temperature', '/temperature'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        config_file_arg,
        temperature_monitor_node,
    ])
