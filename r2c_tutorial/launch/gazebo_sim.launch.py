#!/usr/bin/env python3
"""
Launch file for R2C Tutorial Robot in Gazebo Simulation

This launch file brings up:
1. Gazebo with the robot model
2. robot_state_publisher for TF tree
3. Controller manager with configured controllers
4. RViz2 for visualization

Usage:
    ros2 launch r2c_tutorial gazebo_sim.launch.py

Optional arguments:
    use_rviz:=false     # Don't start RViz
    world:=<path>       # Use custom world file
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_r2c_tutorial = get_package_share_directory('r2c_tutorial')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2 for visualization'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_r2c_tutorial, 'worlds', 'empty.world'),
        description='Full path to world file to load'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Get URDF via xacro
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution(
                    [FindPackageShare('r2c_tutorial'), 'urdf', 'r2c_test.xacro']
                ),
                ' ',
                'sim_mode:=true',
                ' ',
                'use_fake_hardware:=false',
            ]
        ),
        value_type=str
    )
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r'],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'r2c_test_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Bridge between Gazebo and ROS topics
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/temperature_sensor/raw@sensor_msgs/msg/Temperature[gz.msgs.Double',
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Joint state broadcaster (auto-started by controller manager)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Differential drive controller spawner
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Arm position controller spawner
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_position_controller'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # RViz2 node
    rviz_config_file = os.path.join(pkg_r2c_tutorial, 'config', 'r2c_tutorial.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        # Arguments
        use_rviz_arg,
        world_arg,
        use_sim_time_arg,
        
        # Nodes
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot_node,
        gz_bridge_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        arm_controller_spawner,
        rviz_node,
    ])
