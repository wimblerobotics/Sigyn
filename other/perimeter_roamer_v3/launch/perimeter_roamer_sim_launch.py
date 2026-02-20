# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory('perimeter_roamer_v3')

    # Launch arguments
    bt_xml_filename_arg = DeclareLaunchArgument(
        'bt_xml_filename',
        default_value=os.path.join(pkg_dir, 'bt_xml', 'perimeter_roamer.xml'),
        description='Full path to the behavior tree XML file')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'perimeter_roamer_params.yaml'),
        description='Full path to the configuration file')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    start_battery_sim_arg = DeclareLaunchArgument(
        'start_battery_simulator',
        default_value='true',
        description='Start battery simulator node')

    # Battery simulator node (conditional)
    battery_simulator = Node(
        package='perimeter_roamer_v3',
        executable='battery_simulator.py',
        name='battery_simulator',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('start_battery_simulator'))
    )

    # Main perimeter roamer node
    perimeter_roamer_node = Node(
        package='perimeter_roamer_v3',
        executable='perimeter_roamer',
        name='perimeter_roamer_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'bt_xml_filename': LaunchConfiguration('bt_xml_filename'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[
            # These should match your simulation topics
            ('/scan', '/scan'),
            ('/battery_state', '/sigyn/teensy_bridge/battery/status'),
            ('/navigate_to_pose', '/navigate_to_pose'),
            ('/odom', '/odom'),
            ('/cmd_vel', '/cmd_vel_nav'),
        ]
    )

    return LaunchDescription([
        bt_xml_filename_arg,
        config_file_arg,
        use_sim_time_arg,
        start_battery_sim_arg,
        battery_simulator,
        perimeter_roamer_node
    ])
