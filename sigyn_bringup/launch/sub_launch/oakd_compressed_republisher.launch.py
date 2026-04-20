# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Wimble Robotics

"""oakd_compressed_republisher.launch.py — OAK-D compressed image republisher.

Republishes raw OAK-D color images as compressed JPEG so that RViz can use
the bandwidth-efficient image_transport compressed subscriber.

The depthai_ros driver does not natively publish the
compressed variant; this shim bridges that gap on the real robot.

Included by sigyn.launch.py when do_oakd=true and the robot is real
(use_sim_time=false).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Republish /oakd_top/color/image (raw) → /oakd_top/color/image/compressed.
    color_republisher = Node(
        package="image_transport",
        executable="republish",
        name="oakd_color_compressed_republisher",
        namespace="oakd_top",
        output="screen",
        arguments=["raw", "compressed"],
        remappings=[
            ("in",              "color/image"),
            ("out/compressed",  "color/image/compressed"),
        ],
        parameters=[{
            "compressed.jpeg_quality": 80,
            "compressed.png_level":    9,
        }],
    )

    return LaunchDescription([color_republisher])
