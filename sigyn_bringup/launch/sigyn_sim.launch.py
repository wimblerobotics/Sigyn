# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Wimble Robotics
#
# sigyn_sim.launch.py — Launch Sigyn in Gazebo simulation.
#
# Thin wrapper over sigyn.launch.py that pre-sets use_sim_time:=true and exposes
# the most commonly customised arguments for simulation runs.
#
# Usage:
#   ros2 launch base sigyn_sim.launch.py
#   ros2 launch base sigyn_sim.launch.py world:=/path/to/my.world
#   ros2 launch base sigyn_sim.launch.py do_rviz:=false

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    base_pkg = get_package_share_directory("sigyn_bringup")
    description_pkg = get_package_share_directory("sigyn_description")
    default_world = os.path.join(description_pkg, "worlds", "home.world")

    bt_xml = LaunchConfiguration("bt_xml")
    do_joint_state_gui = LaunchConfiguration("do_joint_state_gui")
    do_joystick = LaunchConfiguration("do_joystick")
    do_oakd = LaunchConfiguration("do_oakd")
    do_oakd_yolo26 = LaunchConfiguration("do_oakd_yolo26")
    do_rviz = LaunchConfiguration("do_rviz")
    do_top_lidar = LaunchConfiguration("do_top_lidar")
    urdf_file_name = LaunchConfiguration("urdf_file_name")
    world = LaunchConfiguration("world")

    ld.add_action(DeclareLaunchArgument(
        "bt_xml",
        default_value="/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/"
                      "navigate_to_pose_w_replanning_and_recovery.xml",
        description="Full path to the nav_to_pose behavior tree XML.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "do_joint_state_gui",
        default_value="false",
        description="Launch joint_state_publisher_gui if true.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "do_joystick",
        default_value="false",
        description="Launch the Bluetooth joystick node if true.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "do_oakd",
        default_value="false",
        description="Launch OAK-D camera nodes if true.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "do_oakd_yolo26",
        default_value="true",
        description="Launch YOLO26 CPU detector (requires do_oakd:=true).",
    ))
    ld.add_action(DeclareLaunchArgument(
        "do_rviz",
        default_value="true",
        description="Launch RViz if true.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "do_top_lidar",
        default_value="true",
        description="Use top LiDAR; if false, use only the cup LiDAR.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "urdf_file_name",
        default_value="sigyn.urdf.xacro",
        description="URDF/xacro file name inside sigyn_description/urdf/.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Absolute path to the Gazebo world file.",
    ))

    # Include the main sigyn launch with use_sim_time forced to true.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_pkg, "launch", "sigyn.launch.py")
        ),
        launch_arguments={
            "bt_xml": bt_xml,
            "do_joint_state_gui": do_joint_state_gui,
            "do_joystick": do_joystick,
            "do_oakd": do_oakd,
            "do_oakd_yolo26": do_oakd_yolo26,
            "do_rviz": do_rviz,
            "do_top_lidar": do_top_lidar,
            "urdf_file_name": urdf_file_name,
            "use_sim_time": "true",
            "world": world,
        }.items(),
    ))

    return ld
