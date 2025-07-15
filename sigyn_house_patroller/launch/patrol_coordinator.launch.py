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
        description='Log level for patrol coordinator'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sigyn_house_patroller'),
            'config',
            'patrol_config.yaml'
        ]),
        description='Path to patrol configuration file'
    )
    
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sigyn_house_patroller'),
            'config',
            'waypoints.yaml'
        ]),
        description='Path to waypoint configuration file'
    )
    
    room_file_arg = DeclareLaunchArgument(
        'room_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sigyn_house_patroller'),
            'config',
            'rooms.yaml'
        ]),
        description='Path to room configuration file'
    )
    
    # Patrol coordinator node
    patrol_coordinator_node = Node(
        package='sigyn_house_patroller',
        executable='patrol_coordinator_node',
        name='patrol_coordinator',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'patrol_config_file': LaunchConfiguration('config_file')},
            {'waypoint_config_file': LaunchConfiguration('waypoint_file')},
            {'room_config_file': LaunchConfiguration('room_file')},
            {'patrol_frequency_hz': 1.0},
            {'status_frequency_hz': 2.0},
            {'waypoint_tolerance': 0.5},
            {'navigation_timeout': 300.0},
            {'battery_critical_level': 0.15},
            {'battery_low_level': 0.25},
            {'base_frame': 'base_link'},
            {'map_frame': 'map'},
            {'use_perimeter_following': False},
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('/navigate_to_pose', '/navigate_to_pose'),
            ('/battery_state', '/battery_state'),
            ('/scan', '/scan'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        config_file_arg,
        waypoint_file_arg,
        room_file_arg,
        patrol_coordinator_node,
    ])
