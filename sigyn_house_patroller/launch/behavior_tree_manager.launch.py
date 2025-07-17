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
        description='Log level for behavior tree manager'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sigyn_house_patroller'),
            'config',
            'behavior_tree_manager.yaml'
        ]),
        description='Path to behavior tree manager configuration file'
    )
    
    behavior_tree_dir_arg = DeclareLaunchArgument(
        'behavior_tree_dir',
        default_value=PathJoinSubstitution([
            FindPackageShare('sigyn_house_patroller'),
            'config',
            'behavior_trees'
        ]),
        description='Path to behavior tree XML files directory'
    )
    
    # Behavior tree manager node
    behavior_tree_manager_node = Node(
        package='sigyn_house_patroller',
        executable='behavior_tree_manager_node',
        name='behavior_tree_manager',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'config_directory': LaunchConfiguration('behavior_tree_dir')},
            {'default_behavior_tree': 'normal_patrol.xml'},
            {'switch_cooldown': 10.0},
            {'monitoring_frequency': 1.0},
            {'status_frequency': 2.0},
            {'threat_level_threshold': 2},
            {'battery_critical_threshold': 0.15},
            {'system_health_threshold': 0.5},
            {'enable_auto_switching': True},
            {'enable_nav2_integration': True},
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            ('/bt_navigator/set_parameters', '/bt_navigator/set_parameters'),
            ('/lifecycle_manager_navigation/manage_nodes', '/lifecycle_manager_navigation/manage_nodes'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        config_file_arg,
        behavior_tree_dir_arg,
        behavior_tree_manager_node,
    ])
