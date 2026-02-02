#!/usr/bin/env python3
"""
Launch file for Step 4: ElevatorAtHeight Test.
Tests the ElevatorAtHeight conditional node on the real robot.
Assumes sigyn.launch.py is already running.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    can_do_pkg = get_package_share_directory("can_do_challenge")
    
    # BT Node for real robot
    can_do_node = Node(
        package="can_do_challenge",
        executable="can_do_challenge_node_real",
        name="can_do_challenge_node",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "enable_groot_monitoring": True,
            "groot_port": 1667,
            # Point to the Step 4 BT XML
            "bt_xml_filename": os.path.join(can_do_pkg, "bt_xml", "step4_elevator_height.xml") 
        }],
    )
    
    return LaunchDescription([
        can_do_node,
    ])
