# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory('perimeter_roamer_v3')

    # Launch arguments
    bt_xml_filename_arg = DeclareLaunchArgument(
        'bt_xml_filename',
        default_value='simple_test.xml',
        description='Behavior tree XML filename (without path)')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'perimeter_roamer_params.yaml'),
        description='Full path to the configuration file')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Construct behavior tree file path from parameter
    bt_xml_file = PathJoinSubstitution([
        FindPackageShare('perimeter_roamer_v3'),
        'bt_xml',
        LaunchConfiguration('bt_xml_filename')
    ])

    # Node configuration
    perimeter_roamer_node = Node(
        package='perimeter_roamer_v3',
        executable='perimeter_roamer',
        name='perimeter_roamer_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'bt_xml_filename': bt_xml_file,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[
            # Remap to actual topics on your robot
            ('/scan', '/scan'),
            ('/battery_state', '/sigyn/teensy_bridge/battery/status'),
            ('/navigate_to_pose', '/navigate_to_pose'),
        ]
    )

    # Battery simulator for simulation only
    battery_simulator_node = Node(
        package='perimeter_roamer_v3',
        executable='battery_simulator.py',
        name='battery_simulator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )

    # Delay perimeter_roamer start to ensure Nav2 action server is available
    delayed_roamer = TimerAction(
        period=5.0,
        actions=[perimeter_roamer_node]
    )
    return LaunchDescription([
        bt_xml_filename_arg,
        config_file_arg,
        use_sim_time_arg,
        battery_simulator_node,
        delayed_roamer
    ])