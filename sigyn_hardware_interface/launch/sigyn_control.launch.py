#!/usr/bin/env python3
"""
Launch file for Sigyn ROS2 Control system.

Starts the ros2_control system with hardware interface and controllers for
the Sigyn robot differential drive system. This provides a standard interface
for motor control while maintaining compatibility with the TeensyV2 embedded system.

Author: Wimble Robotics
Date: 2025
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Generate launch description for Sigyn ros2_control system."""
    
    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="sigyn_diff_drive_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "teensy_port",
            default_value="/dev/teensy_sensor",
            description="Serial port for Teensy communication.",
        )
    )

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    robot_controller = LaunchConfiguration("robot_controller")
    teensy_port = LaunchConfiguration("teensy_port")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("description"),
                    "urdf",
                    "sigyn.urdf.xacro",
                ]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "teensy_port:=",
            teensy_port,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot controllers configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("sigyn_hardware_interface"),
            "config",
            "controllers.yaml",
        ]
    )

    # ros2_control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Robot controller spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "--controller-manager", "/controller_manager"],
    )

    # Delay robot controller spawner after joint state broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Optional: Teensy bridge for sensor data (runs alongside ros2_control)
    # This is needed until all Teensy communication is moved to the hardware interface
    teensy_bridge_node = Node(
        package="sigyn_to_sensor_v2",
        executable="teensy_bridge_node",
        name="teensy_bridge_sensors",
        parameters=[
            {"board1_port": teensy_port},
            {"enable_motor_control": False},  # Disable motor control in bridge
        ],
        output="screen",
        condition=IfCondition("false"),  # Disabled by default for Phase 1
    )

    return LaunchDescription(
        declared_arguments +
        [
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
            teensy_bridge_node,
        ]
    )
