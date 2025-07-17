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
        description='Log level for door monitor'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sigyn_house_patroller'),
            'config',
            'door_monitor.yaml'
        ]),
        description='Path to door monitor configuration file'
    )
    
    # Door monitor node
    door_monitor_node = Node(
        package='sigyn_house_patroller',
        executable='door_monitor_node',
        name='door_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'monitoring_frequency': 2.0},
            {'scan_timeout': 30.0},
            {'scan_topic': '/scan'},
            {'alert_cooldown': 120.0},
            {'distance_tolerance': 0.1},
            {'angle_tolerance': 0.1},
            {'enable_room_filtering': True},
            {'map_frame': 'map'},
            {'robot_frame': 'base_link'},
            # Door configurations
            {'doors.front_door.name': 'Front Door'},
            {'doors.front_door.angle': 0.0},
            {'doors.front_door.expected_distance': 0.5},
            {'doors.front_door.expected_open': False},
            {'doors.front_door.tolerance': 0.1},
            {'doors.back_door.name': 'Back Door'},
            {'doors.back_door.angle': 3.14159},
            {'doors.back_door.expected_distance': 0.5},
            {'doors.back_door.expected_open': False},
            {'doors.back_door.tolerance': 0.1},
            LaunchConfiguration('config_file'),
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('/scan', '/scan'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        config_file_arg,
        door_monitor_node,
    ])
