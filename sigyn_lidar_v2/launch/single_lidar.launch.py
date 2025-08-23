#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sigyn_lidar_v2'), 'config', 'single_lidar_config.yaml'
        ]),
        description='Path to single LIDAR YAML config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time'
    )

    lidar_node = Node(
        package='sigyn_lidar_v2',
        executable='sigyn_lidar_v2_node',
        name='single_lidar_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        lidar_node,
    ])
