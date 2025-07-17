#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('sigyn_house_patroller')
    config_dir = os.path.join(package_dir, 'config')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    patrol_config = LaunchConfiguration('patrol_config', default=os.path.join(config_dir, 'patrol_config.yaml'))
    
    # Behavior tree manager node
    behavior_tree_manager_node = Node(
        package='sigyn_house_patroller',
        executable='behavior_tree_manager_node',
        name='behavior_tree_manager',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_directory': os.path.join(package_dir, 'config/behavior_trees'),
            'default_behavior_tree': 'normal_patrol.xml',
            'switch_cooldown': 10.0,
            'monitoring_frequency': 1.0,
            'status_frequency': 2.0,
            'threat_level_threshold': 2,
            'battery_critical_threshold': 0.15,
            'system_health_threshold': 0.5,
            'enable_auto_switching': True,
            'enable_nav2_integration': True,
        }],
        output='screen'
    )
    
    # Core monitoring nodes
    battery_monitor_node = Node(
        package='sigyn_house_patroller',
        executable='battery_monitor_node',
        name='battery_monitor',
        parameters=[{
            'use_sim_time': use_sim_time,
            'monitoring_frequency': 1.0,
            'battery_topic': '/battery_state'
        }],
        output='screen'
    )
    
    temperature_monitor_node = Node(
        package='sigyn_house_patroller',
        executable='temperature_monitor_node',
        name='temperature_monitor',
        parameters=[{
            'use_sim_time': use_sim_time,
            'monitoring_frequency': 1.0,
            'temperature_topic': '/temperature'
        }],
        output='screen'
    )
    
    door_monitor_node = Node(
        package='sigyn_house_patroller',
        executable='door_monitor_node',
        name='door_monitor',
        parameters=[{
            'use_sim_time': use_sim_time,
            'monitoring_frequency': 2.0,
            'scan_topic': '/scan'
        }],
        output='screen'
    )
    
    change_detector_node = Node(
        package='sigyn_house_patroller',
        executable='change_detector_node',
        name='change_detector',
        parameters=[{
            'use_sim_time': use_sim_time,
            'monitoring_frequency': 1.0,
            'enable_visual_diff': True,
            'enable_3d_diff': True
        }],
        output='screen'
    )
    
    localization_corrector_node = Node(
        package='sigyn_house_patroller',
        executable='localization_corrector_node',
        name='localization_corrector',
        parameters=[{
            'use_sim_time': use_sim_time,
            'monitoring_frequency': 5.0,
            'max_pose_jump': 0.5
        }],
        output='screen'
    )
    
    threat_response_node = Node(
        package='sigyn_house_patroller',
        executable='threat_response_node',
        name='threat_response',
        parameters=[{
            'use_sim_time': use_sim_time,
            'response_timeout': 300.0,
            'max_concurrent_responses': 3
        }],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'patrol_config',
            default_value=os.path.join(config_dir, 'patrol_config.yaml'),
            description='Path to patrol configuration file'
        ),
        
        # Launch all nodes
        behavior_tree_manager_node,
        battery_monitor_node,
        temperature_monitor_node,
        door_monitor_node,
        change_detector_node,
        localization_corrector_node,
        threat_response_node,
    ])
