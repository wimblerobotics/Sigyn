#!/usr/bin/env python3
"""
Launch file for TeensyV2 communication system.

Starts the main node required for communication with TeensyV2 embedded system:
- teensy_bridge_node: Main communication bridge handling all message parsing and ROS2 publishing

The bridge node handles all TeensyV2 message types (BATT, PERF, IMU, DIAG) and publishes
to appropriate ROS2 topics through the unified bridge system.

Author: Sigyn Robotics
Date: 2025
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for TeensyV2 system."""
    
    # Package directory
    pkg_dir = FindPackageShare('sigyn_to_sensor_v2')
    
    # Configuration file path
    config_file = PathJoinSubstitution([
        pkg_dir,
        'config',
        'teensy_v2_config.yaml'
    ])
    
    # Launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='sigyn',
        description='Namespace for all nodes'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to configuration file'
    )
    
    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='Use composed nodes for better performance'
    )
    
    enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='true',
        description='Enable diagnostic reporting'
    )
    
    board1_port_arg = DeclareLaunchArgument(
        'board1_port',
        default_value='/dev/teensy_sensor',
        description='Serial port for Board 1 (main controller)'
    )
    
    board2_port_arg = DeclareLaunchArgument(
        'board2_port',
        default_value='/dev/teensy_sensor2',
        description='Serial port for Board 2 (sensor board)'
    )
    
    board3_port_arg = DeclareLaunchArgument(
        'board3_port',
        default_value='/dev/teensy_gripper',
        description='Serial port for Board 3 (gripper)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for all nodes'
    )
    
    # Common parameters
    common_params = {
        'namespace': LaunchConfiguration('namespace'),
        'config_file': LaunchConfiguration('config_file'),
        'enable_diagnostics': LaunchConfiguration('enable_diagnostics'),
        'board1_port': LaunchConfiguration('board1_port'),
        'board2_port': LaunchConfiguration('board2_port'),
        'board3_port': LaunchConfiguration('board3_port'),
    }
    
    # Individual nodes (default mode)
    individual_nodes = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('use_composition')),
        actions=[
            # Main communication bridge node
            Node(
                package='sigyn_to_sensor_v2',
                executable='teensy_bridge_node',
                name='teensy_bridge',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    LaunchConfiguration('config_file'),
                    common_params,
                ],
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                respawn=True,
                respawn_delay=2.0,
            ),
        ]
    )
    
    # Composed nodes (for better performance)
    composed_nodes = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_composition')),
        actions=[
            # Container for composed nodes
            Node(
                package='rclcpp_components',
                executable='component_container',
                name='teensy_v2_container',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            ),
            
            # Load all components into the container
            LoadComposableNodes(
                target_container=PathJoinSubstitution([
                    LaunchConfiguration('namespace'),
                    'teensy_v2_container'
                ]),
                composable_node_descriptions=[
                    ComposableNode(
                        package='sigyn_to_sensor_v2',
                        plugin='sigyn_to_sensor_v2::TeensyBridgeNode',
                        name='teensy_bridge',
                        parameters=[
                            LaunchConfiguration('config_file'),
                            common_params,
                        ],
                    ),
                ],
            ),
        ]
    )
    
    # Status logging
    startup_log = LogInfo(
        msg=[
            'Starting TeensyV2 communication system...\n',
            'Namespace: ', LaunchConfiguration('namespace'), '\n',
            'Config file: ', LaunchConfiguration('config_file'), '\n',
            'Board 1 port: ', LaunchConfiguration('board1_port'), '\n',
            'Board 2 port: ', LaunchConfiguration('board2_port'), '\n',
            'Composition mode: ', LaunchConfiguration('use_composition'), '\n',
            'Log level: ', LaunchConfiguration('log_level')
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        namespace_arg,
        config_file_arg,
        use_composition_arg,
        enable_diagnostics_arg,
        board1_port_arg,
        board2_port_arg,
        board3_port_arg,
        log_level_arg,
        
        # Startup information
        startup_log,
        
        # Node groups
        individual_nodes,
        composed_nodes,
    ])
