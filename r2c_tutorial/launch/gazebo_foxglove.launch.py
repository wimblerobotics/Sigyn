#!/usr/bin/env python3
"""
Headless Launch with Foxglove for R2C Tutorial Robot

This launch file runs Gazebo in headless mode with Foxglove Bridge for visualization.

Usage:
    ros2 launch r2c_tutorial gazebo_foxglove.launch.py
    
Then open Foxglove Studio and connect to ws://localhost:8765
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_r2c_tutorial = get_package_share_directory('r2c_tutorial')
    
    # Set environment variables for Gazebo to find ROS2 plugins
    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value=''),
            os.pathsep if os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH') else '',
            '/opt/ros/jazzy/lib'
        ]
    )
    
    # Declare launch arguments
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
    
    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Port for Foxglove Bridge WebSocket server'
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
    
    # Gazebo server (headless - no GUI)
    gazebo_server = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            '-r',  # Run on start
            '-s',  # Server only (headless)
            LaunchConfiguration('world')
        ],
        output='screen'
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
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Joint state broadcaster
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

    # Temperature sensor simulator (publishes sensor_msgs/Temperature)
    temperature_sensor_node = Node(
        package='r2c_tutorial',
        executable='temperature_sensor_sim.py',
        name='temperature_sensor_sim',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'initial_temperature': 25.0},
            {'publish_rate': 10.0},
        ]
    )
    
    # Twist to TwistStamped converter (for teleop compatibility)
    twist_converter_node = Node(
        package='r2c_tutorial',
        executable='twist_to_stamped.py',
        name='twist_to_stamped',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'input_topic': '/cmd_vel'},
            {'output_topic': '/diff_drive_controller/cmd_vel'},
        ]
    )

    # Foxglove Bridge
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[
            {'port': LaunchConfiguration('foxglove_port')},
            {'address': '0.0.0.0'},
            {'tls': False},
            {'topic_whitelist': ['.*']},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Delay controller spawners to give hardware interface time to initialize
    delayed_joint_state_broadcaster = TimerAction(
        period=10.0,
        actions=[joint_state_broadcaster_spawner]
    )
    
    delayed_diff_drive_controller = TimerAction(
        period=10.0,
        actions=[diff_drive_controller_spawner]
    )
    
    delayed_arm_controller = TimerAction(
        period=10.0,
        actions=[arm_controller_spawner]
    )
    
    return LaunchDescription([
        # Environment variables
        gz_plugin_path,
        
        # Arguments
        world_arg,
        use_sim_time_arg,
        foxglove_port_arg,
        
        # Nodes
        robot_state_publisher_node,
        gazebo_server,
        spawn_robot_node,
        gz_bridge_node,
        delayed_joint_state_broadcaster,
        delayed_diff_drive_controller,
        delayed_arm_controller,
        temperature_sensor_node,
        twist_converter_node,
        foxglove_bridge_node,
    ])
