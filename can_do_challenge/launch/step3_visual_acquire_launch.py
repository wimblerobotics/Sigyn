#!/usr/bin/env python3
"""
Launch file for Real Robot Step 3: VisuallyAcquireCan subtree test.

This test runs the VisuallyAcquireCan subtree using the real robot stack.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
	base_pkg = get_package_share_directory("base")
	can_do_pkg = get_package_share_directory("can_do_challenge")

	sigyn_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(base_pkg, "launch", "sigyn.launch.py")
		),
		launch_arguments={
			"use_sim_time": "false",
			"do_rviz": "true",
			"make_map": "false",
			"do_oakd": "true",
			"do_top_lidar": "true",
		}.items(),
	)

	can_do_node = Node(
		package="can_do_challenge",
		executable="can_do_challenge_node_real",
		name="can_do_challenge_node",
		output="screen",
		parameters=[{
			"use_sim_time": False,
			"enable_groot_monitoring": True,
			"groot_port": 1667,
			"bt_xml_filename": os.path.join(can_do_pkg, "bt_xml", "step3_real_visual_acquire.xml"),
		}],
	)

	return LaunchDescription([
		sigyn_launch,
		can_do_node,
	])
