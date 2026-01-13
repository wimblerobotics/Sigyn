#!/usr/bin/env python3
"""
Manual simulation launch for Can Do Challenge.

This launch file starts:
1. Manual controller GUI for stepping through BT and controlling sensors
2. can_do_challenge_node in manual stepping mode
3. Connects to an already-running simulation (run can_do_sim_launch.py first)

Usage:
  ros2 launch can_do_challenge can_do_sim_launch.py  # Terminal 1
  ros2 launch can_do_challenge manual_sim_launch.py  # Terminal 2
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    can_do_pkg = get_package_share_directory("can_do_challenge")
    
    # Behavior tree file
    bt_xml_file = os.path.join(can_do_pkg, "bt_xml", "main.xml")
    
    # Launch arguments
    step_manually_arg = DeclareLaunchArgument(
        "step_manually",
        default_value="true",
        description="Enable manual stepping mode (true) or automatic execution (false)"
    )
    
    use_groot_arg = DeclareLaunchArgument(
        "use_groot",
        default_value="true",
        description="Enable Groot monitoring on port 1667"
    )
    
    # Manual controller GUI node
    manual_controller_gui = Node(
        package="can_do_challenge",
        executable="manual_controller_gui.py",
        name="manual_controller_gui",
        output="screen",
        parameters=[{
            "use_sim_time": True,
        }],
    )
    
    # Can Do Challenge behavior tree node with manual stepping
    can_do_node = Node(
        package="can_do_challenge",
        executable="can_do_challenge_node",
        name="can_do_challenge_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "bt_xml_filename": bt_xml_file,
            "step_manually": LaunchConfiguration("step_manually"),
            "enable_groot_monitoring": LaunchConfiguration("use_groot"),
            "groot_port": 1667,
        }],
    )
    
    return LaunchDescription([
        step_manually_arg,
        use_groot_arg,
        manual_controller_gui,
        can_do_node,
    ])
