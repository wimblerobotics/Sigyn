#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('sigyn_house_patroller')
    
    # Launch arguments for individual node control
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for all nodes'
    )
    
    # Node enable/disable arguments
    enable_battery_monitor_arg = DeclareLaunchArgument(
        'enable_battery_monitor',
        default_value='true',
        description='Enable battery monitor node'
    )
    
    enable_temperature_monitor_arg = DeclareLaunchArgument(
        'enable_temperature_monitor',
        default_value='true',
        description='Enable temperature monitor node'
    )
    
    enable_door_monitor_arg = DeclareLaunchArgument(
        'enable_door_monitor',
        default_value='true',
        description='Enable door monitor node'
    )
    
    enable_change_detector_arg = DeclareLaunchArgument(
        'enable_change_detector',
        default_value='true',
        description='Enable change detector node'
    )
    
    enable_behavior_tree_manager_arg = DeclareLaunchArgument(
        'enable_behavior_tree_manager',
        default_value='true',
        description='Enable behavior tree manager node'
    )
    
    enable_threat_response_arg = DeclareLaunchArgument(
        'enable_threat_response',
        default_value='true',
        description='Enable threat response node'
    )
    
    enable_localization_corrector_arg = DeclareLaunchArgument(
        'enable_localization_corrector',
        default_value='true',
        description='Enable localization corrector node'
    )
    
    enable_patrol_coordinator_arg = DeclareLaunchArgument(
        'enable_patrol_coordinator',
        default_value='true',
        description='Enable patrol coordinator node'
    )
    
    # Include individual launch files conditionally
    battery_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sigyn_house_patroller'),
                'launch',
                'battery_monitor.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_battery_monitor'))
    )
    
    temperature_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sigyn_house_patroller'),
                'launch',
                'temperature_monitor.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_temperature_monitor'))
    )
    
    door_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sigyn_house_patroller'),
                'launch',
                'door_monitor.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_door_monitor'))
    )
    
    change_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sigyn_house_patroller'),
                'launch',
                'change_detector.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_change_detector'))
    )
    
    behavior_tree_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sigyn_house_patroller'),
                'launch',
                'behavior_tree_manager.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_behavior_tree_manager'))
    )
    
    threat_response_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sigyn_house_patroller'),
                'launch',
                'threat_response.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_threat_response'))
    )
    
    localization_corrector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sigyn_house_patroller'),
                'launch',
                'localization_corrector.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_localization_corrector'))
    )
    
    patrol_coordinator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sigyn_house_patroller'),
                'launch',
                'patrol_coordinator.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_patrol_coordinator'))
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        log_level_arg,
        enable_battery_monitor_arg,
        enable_temperature_monitor_arg,
        enable_door_monitor_arg,
        enable_change_detector_arg,
        enable_behavior_tree_manager_arg,
        enable_threat_response_arg,
        enable_localization_corrector_arg,
        enable_patrol_coordinator_arg,
        
        # Launch files
        battery_monitor_launch,
        temperature_monitor_launch,
        door_monitor_launch,
        change_detector_launch,
        behavior_tree_manager_launch,
        threat_response_launch,
        localization_corrector_launch,
        patrol_coordinator_launch,
    ])
