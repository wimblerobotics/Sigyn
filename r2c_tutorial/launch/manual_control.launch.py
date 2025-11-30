#!/usr/bin/env python3
"""
Launch file for manual control and testing of the R2C Tutorial Robot

This launch file is useful for:
1. Testing the URDF without Gazebo
2. Manual joint control using joint_state_publisher_gui
3. Visualizing the robot in RViz

Usage:
    ros2 launch r2c_tutorial manual_control.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directory
    pkg_r2c_tutorial = get_package_share_directory('r2c_tutorial')
    
    # Declare launch arguments
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use joint_state_publisher_gui for manual control'
    )
    
    # Get URDF via xacro (fake hardware mode for testing without Gazebo)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('r2c_tutorial'), 'urdf', 'r2c_test.xacro']
            ),
            ' ',
            'sim_mode:=false',
            ' ',
            'use_fake_hardware:=true',
        ]
    )
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Joint state publisher GUI for manual control
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )
    
    # RViz2
    rviz_config_file = os.path.join(pkg_r2c_tutorial, 'config', 'r2c_tutorial.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )
    
    return LaunchDescription([
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
