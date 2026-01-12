#!/usr/bin/env python3
"""
Simple launch file to just view the can_challenge world in Gazebo.
Does not spawn the robot - just shows the table and can.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the world file path
    description_pkg = get_package_share_directory("description")
    world_file = os.path.join(description_pkg, "worlds", "can_challenge.world")
    
    # Launch Gazebo with the world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world_file}",
        }.items(),
    )
    
    return LaunchDescription([gazebo])
