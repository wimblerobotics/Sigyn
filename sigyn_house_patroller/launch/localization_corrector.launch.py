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
        description='Log level for localization corrector'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sigyn_house_patroller'),
            'config',
            'localization_corrector.yaml'
        ]),
        description='Path to localization corrector configuration file'
    )
    
    # Localization corrector node
    localization_corrector_node = Node(
        package='sigyn_house_patroller',
        executable='localization_corrector_node',
        name='localization_corrector',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'monitoring_frequency': 2.0},
            {'pose_timeout': 30.0},
            {'scan_timeout': 30.0},
            {'pose_topic': '/amcl_pose'},
            {'scan_topic': '/scan'},
            {'map_topic': '/map'},
            {'initialpose_topic': '/initialpose'},
            {'map_frame': 'map'},
            {'robot_frame': 'base_link'},
            {'odom_frame': 'odom'},
            {'pose_covariance_threshold': 0.5},
            {'position_jump_threshold': 1.0},
            {'orientation_jump_threshold': 0.5},
            {'scan_match_threshold': 0.8},
            {'correction_cooldown': 60.0},
            {'enable_auto_correction': True},
            {'min_scan_points': 100},
            LaunchConfiguration('config_file'),
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('/amcl_pose', '/amcl_pose'),
            ('/scan', '/scan'),
            ('/map', '/map'),
            ('/initialpose', '/initialpose'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        config_file_arg,
        localization_corrector_node,
    ])
