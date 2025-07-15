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
        description='Log level for threat response'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sigyn_house_patroller'),
            'config',
            'threat_response.yaml'
        ]),
        description='Path to threat response configuration file'
    )
    
    # Threat response node
    threat_response_node = Node(
        package='sigyn_house_patroller',
        executable='threat_response_node',
        name='threat_response',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'response_timeout': 300.0},
            {'investigation_distance': 1.0},
            {'threat_priority_threshold': 2},
            {'enable_auto_response': True},
            {'response_cooldown': 120.0},
            {'max_investigation_attempts': 3},
            {'emergency_stop_topic': '/emergency_stop'},
            {'email_alerts': True},
            {'patrol_mode_service': '/sigyn_house_patroller/set_patrol_mode'},
            {'patrol_action': '/sigyn_house_patroller/patrol_to_waypoint'},
            # Response waypoints for different threat types
            {'response_waypoints.battery_critical': 'entry_center'},
            {'response_waypoints.temperature_anomaly': 'kitchen_center'},
            {'response_waypoints.door_state_change': 'front_door_check'},
            {'response_waypoints.motion_detection': 'living_room_center'},
            {'response_waypoints.default': 'hallway_center'},
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('/emergency_stop', '/emergency_stop'),
            ('/sigyn_house_patroller/threat_alerts', '/sigyn_house_patroller/threat_alerts'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        config_file_arg,
        threat_response_node,
    ])
