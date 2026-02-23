# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn
"""Sigyn bringup shim: delegates to sigyn_oakd_detection's production launch.

Included by sigyn.launch.py when do_oakd:=true and do_oakd_yolo26:=true.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """Delegate to sigyn_oakd_detection's oakd_detector.launch.py."""
    pkg_share = get_package_share_directory("sigyn_oakd_detection")

    detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "oakd_detector.launch.py")
        ),
    )

    return LaunchDescription([detector_launch])
