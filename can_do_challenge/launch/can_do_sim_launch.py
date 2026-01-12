#!/usr/bin/env python3
"""
Launch file for Can Do Challenge simulation environment.

This is a wrapper around the base sigyn.launch.py that:
- Spawns the Sigyn robot in Gazebo with the can_challenge.world
- Includes sim camera plugins for OAK-D and Pi Camera
- Launches the can_do_challenge behavior tree node
- Launches RViz with proper configuration
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    base_pkg = get_package_share_directory("base")
    description_pkg = get_package_share_directory("description")
    
    # World file for can challenge
    world_file = os.path.join(description_pkg, "worlds", "can_challenge.world")
    
    # Launch arguments
    use_groot_arg = DeclareLaunchArgument(
        "use_groot",
        default_value="true",
        description="Enable Groot monitoring on port 1667"
    )
    
    # Include the main sigyn launch with simulation parameters
    # This handles: Gazebo, robot spawning, Nav2, RViz, controllers, etc.
    sigyn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_pkg, "launch", "sigyn.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "world": world_file,
            "do_rviz": "true",
            "make_map": "false",
        }.items()
    )
    
    # Can Do Challenge behavior tree node (disabled by default - launch separately when ready)
    # Uncomment to enable:
    # can_do_node = Node(
    #     package="can_do_challenge",
    #     executable="can_do_challenge_node",
    #     name="can_do_challenge_node",
    #     output="screen",
    #     parameters=[{
    #         "use_sim_time": True,
    #         "enable_groot_monitoring": LaunchConfiguration("use_groot"),
    #         "groot_port": 1667,
    #     }],
    # )
    
    return LaunchDescription([
        use_groot_arg,
        sigyn_launch,
        # can_do_node,  # Uncomment to auto-start the behavior tree
    ])
