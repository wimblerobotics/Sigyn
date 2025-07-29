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
        description='Log level for change detector'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sigyn_house_patroller'),
            'config',
            'change_detector.yaml'
        ]),
        description='Path to change detector configuration file'
    )
    
    # Change detector node
    change_detector_node = Node(
        package='sigyn_house_patroller',
        executable='change_detector_node',
        name='change_detector',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'monitoring_frequency': 1.0},
            {'pointcloud_timeout': 30.0},
            {'image_timeout': 30.0},
            {'pointcloud_topic': '/camera/depth/points'},
            {'image_topic': '/camera/color/image_raw'},
            {'enable_visual_diff': True},
            {'enable_3d_diff': True},
            {'visual_diff_threshold': 0.3},
            {'3d_diff_threshold': 0.05},
            {'min_change_area': 100},
            {'alert_cooldown': 60.0},
            {'reference_update_interval': 600.0},
            {'enable_auto_reference_update': True},
            {'change_confidence_threshold': 0.7},
            {'enable_room_filtering': True},
            LaunchConfiguration('config_file'),
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('/camera/depth/points', '/camera/depth/points'),
            ('/camera/color/image_raw', '/camera/color/image_raw'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        config_file_arg,
        change_detector_node,
    ])
