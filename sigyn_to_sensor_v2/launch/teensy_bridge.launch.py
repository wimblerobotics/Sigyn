#!/usr/bin/env python3
"""Launch TeensyV2 bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for TeensyV2 system."""
    pkg_dir = FindPackageShare('sigyn_to_sensor_v2')
    config_file = PathJoinSubstitution([pkg_dir, 'config', 'teensy_v2_config.yaml'])

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='sigyn',
        description='Namespace for all nodes',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to configuration file',
    )

    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='Use composed nodes (currently not supported).',
    )

    enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='true',
        description='Enable diagnostic reporting',
    )

    log_estop_raw_lines_arg = DeclareLaunchArgument(
        'log_estop_raw_lines',
        default_value='false',
        description='Log raw Teensy serial frames for E-STOP events',
    )

    board1_port_arg = DeclareLaunchArgument(
        'board1_port',
        default_value='/dev/teensy_sensor',
        description='Serial port for Board 1 (main controller)',
    )

    board2_port_arg = DeclareLaunchArgument(
        'board2_port',
        default_value='/dev/teensy_sensor2',
        description='Serial port for Board 2 (sensor board)',
    )

    board3_port_arg = DeclareLaunchArgument(
        'board3_port',
        default_value='/dev/teensy_gripper',
        description='Serial port for Board 3 (gripper)',
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for all nodes',
    )

    common_params = {
        'namespace': LaunchConfiguration('namespace'),
        'config_file': LaunchConfiguration('config_file'),
        'enable_diagnostics': LaunchConfiguration('enable_diagnostics'),
        'log_estop_raw_lines': LaunchConfiguration('log_estop_raw_lines'),
        'board1_port': LaunchConfiguration('board1_port'),
        'board2_port': LaunchConfiguration('board2_port'),
        'board3_port': LaunchConfiguration('board3_port'),
    }

    individual_nodes = GroupAction(
        actions=[
            Node(
                package='sigyn_to_sensor_v2',
                executable='teensy_bridge_node',
                name='teensy_bridge',
                namespace=LaunchConfiguration('namespace'),
                parameters=[LaunchConfiguration('config_file'), common_params],
                remappings=[('cmd_vel', '/cmd_vel')],
                output='screen',
                arguments=[
                    '--ros-args',
                    '--log-level',
                    LaunchConfiguration('log_level'),
                ],
                respawn=True,
                respawn_delay=2.0,
            ),
        ],
    )

    startup_log = LogInfo(
        msg=[
            'Starting TeensyV2 communication system...\n',
            'Namespace: ',
            LaunchConfiguration('namespace'),
            '\n',
            'Config file: ',
            LaunchConfiguration('config_file'),
            '\n',
            'Board 1 port: ',
            LaunchConfiguration('board1_port'),
            '\n',
            'Board 2 port: ',
            LaunchConfiguration('board2_port'),
            '\n',
            'Composition mode: ',
            LaunchConfiguration('use_composition'),
            '\n',
            'Log level: ',
            LaunchConfiguration('log_level'),
        ],
    )

    return LaunchDescription(
        [
            namespace_arg,
            config_file_arg,
            use_composition_arg,
            enable_diagnostics_arg,
            log_estop_raw_lines_arg,
            board1_port_arg,
            board2_port_arg,
            board3_port_arg,
            log_level_arg,
            startup_log,
            individual_nodes,
        ]
    )
