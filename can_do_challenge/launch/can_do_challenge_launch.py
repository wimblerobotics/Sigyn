#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Wimblerobotics

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('can_do_challenge')
    
    # Declare launch arguments
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    enable_groot = LaunchConfiguration('enable_groot')
    groot_port = LaunchConfiguration('groot_port')
    
    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(pkg_dir, 'bt_xml', 'main.xml'),
        description='Full path to the behavior tree XML file'
    )
    
    declare_enable_groot_cmd = DeclareLaunchArgument(
        'enable_groot',
        default_value='true',
        description='Enable Groot monitoring'
    )
    
    declare_groot_port_cmd = DeclareLaunchArgument(
        'groot_port',
        default_value='1667',
        description='Port for Groot ZMQ publisher'
    )
    
    # Create the can_do_challenge node
    can_do_challenge_node = Node(
        package='can_do_challenge',
        executable='can_do_challenge_node',
        name='can_do_challenge_node',
        output='screen',
        parameters=[{
            'bt_xml_filename': bt_xml_file,
            'enable_groot_monitoring': enable_groot,
            'groot_port': groot_port,
        }]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Declare arguments
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_enable_groot_cmd)
    ld.add_action(declare_groot_port_cmd)
    
    # Add nodes
    ld.add_action(can_do_challenge_node)
    
    return ld
