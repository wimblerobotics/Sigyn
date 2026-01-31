#!/usr/bin/env python3
"""
Launch file for Real Robot Step 1: OAK-D Detection Test.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    base_pkg = get_package_share_directory("base")
    can_do_pkg = get_package_share_directory("can_do_challenge")
    
    # Include main sigyn launch for Real Robot (use_sim_time=false)
    # Enable OAK-D to get the detector running
    sigyn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_pkg, "launch", "sigyn.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "do_rviz": "true",
            "make_map": "false",
            "do_oakd": "true", # Important: Enable OAK-D driver
            "do_top_lidar": "true",
        }.items()
    )
    
    # Real Robot BT Node
    can_do_node = Node(
        package="can_do_challenge",
        executable="can_do_challenge_node_real", # NEW executable
        name="can_do_challenge_node",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "enable_groot_monitoring": True,
            "groot_port": 1667,
            # Point to the Step 1 BT XML
            "bt_xml_filename": os.path.join(can_do_pkg, "bt_xml", "step1_real_oakd.xml") 
        }],
    )
    
    return LaunchDescription([
        sigyn_launch,
        can_do_node,
    ])