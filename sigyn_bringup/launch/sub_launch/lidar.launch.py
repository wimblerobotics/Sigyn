# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Wimble Robotics

"""lidar.launch.py — LiDAR driver bringup sub-launch.

Starts the wr_ldlidar drivers and the angular laser-filter chain.

Two physical LiDARs are fitted to Sigyn:
  • top_ldlidar  — mounted high, full 360° view  (/dev/lidar_top)
  • cup_ldlidar  — mounted low at cup height       (/dev/lidar_cup)

When do_top_lidar=true (default), both sensors are active:
  top_ldlidar   publishes to raw_scan  → filtered by scan_to_scan_filter_chain → /scan
  cup_ldlidar   publishes directly to /scan_cup (no filter needed)

When do_top_lidar=false, only the cup LiDAR is used:
  cup_ldlidar   masquerades as the top_ldlidar frame and publishes to raw_scan
                → filtered by scan_to_scan_filter_chain → /scan

Launch arguments:
  do_top_lidar    (true)   Enable the top LiDAR; false → single cup-height LiDAR only.
  range_threshold (0.0)    Minimum valid range in metres (0 = accept all).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory("sigyn_bringup")

    # ---------------------------------------------------------------------------
    # Declare all arguments up-front.
    # ---------------------------------------------------------------------------
    declared_args = [
        DeclareLaunchArgument(
            "do_top_lidar",
            default_value="true",
            description="Enable top LiDAR; false → single cup-height LiDAR only",
        ),
        DeclareLaunchArgument(
            "range_threshold",
            default_value="0.0",
            description="Minimum valid range in metres (0.0 = accept all returns)",
        ),
    ]

    do_top_lidar    = LaunchConfiguration("do_top_lidar")
    range_threshold = LaunchConfiguration("range_threshold")

    # ---------------------------------------------------------------------------
    # LiDAR drivers.
    # ---------------------------------------------------------------------------

    # Top LiDAR — only when do_top_lidar=true.
    # Output remapped to raw_scan so the filter chain can process it before /scan.
    top_ldlidar = Node(
        package="wr_ldlidar",
        executable="wr_ldlidar",
        name="top_ldlidar",
        output="screen",
        condition=IfCondition(do_top_lidar),
        parameters=[{
            "serial_port":     "/dev/lidar_top",
            "topic_name":      "scan",
            "lidar_frame":     "lidar_frame_top_lidar",
            "range_threshold": range_threshold,
        }],
        remappings=[("scan", "raw_scan")],
    )

    # Cup LiDAR — only when do_top_lidar=true (secondary sensor).
    # Publishes directly to scan_cup; no filter chain needed.
    cup_ldlidar = Node(
        package="wr_ldlidar",
        executable="wr_ldlidar",
        name="cup_ldlidar",
        output="screen",
        condition=IfCondition(do_top_lidar),
        parameters=[{
            "serial_port":     "/dev/lidar_cup",
            "topic_name":      "scan_cup",
            "lidar_frame":     "lidar_frame_cup_lidar",
            "range_threshold": range_threshold,
        }],
        remappings=[("scan", "scan_cup")],
    )

    # Cup LiDAR standing in for the top LiDAR — only when do_top_lidar=false.
    # Uses the top_lidar TF frame so the rest of the stack needs no reconfiguration.
    cup_as_top_ldlidar = Node(
        package="wr_ldlidar",
        executable="wr_ldlidar",
        name="top_ldlidar",
        output="screen",
        condition=UnlessCondition(do_top_lidar),
        parameters=[{
            "serial_port":     "/dev/lidar_cup",
            "topic_name":      "scan_cup",
            "lidar_frame":     "lidar_frame_top_lidar",
            "range_threshold": range_threshold,
        }],
        remappings=[("scan", "raw_scan")],
    )

    # ---------------------------------------------------------------------------
    # Angular laser filter.
    # Reads raw_scan, removes out-of-range bearings, publishes to /scan.
    # ---------------------------------------------------------------------------
    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="scan_to_scan_filter_chain",
        output="screen",
        parameters=[
            PathJoinSubstitution([bringup_pkg, "config", "laser_filters_angular.yaml"]),
        ],
        remappings=[
            ("scan",          "raw_scan"),
            ("scan_filtered", "scan"),
        ],
    )

    return LaunchDescription([
        *declared_args,
        top_ldlidar,
        cup_ldlidar,
        cup_as_top_ldlidar,
        laser_filter,
    ])
