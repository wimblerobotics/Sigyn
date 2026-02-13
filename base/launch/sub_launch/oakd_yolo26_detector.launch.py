# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch YOLO OAK-D detector for Coke can detection using yolo_oakd_test package."""
    
    from launch_ros.actions import Node
    
    yolo_pkg = get_package_share_directory("yolo_oakd_test")
    
    # Use the yolo_oakd_test node directly
    oakd_node = Node(
        package="yolo_oakd_test",
        executable="oakd_can_detector.py",
        name="oakd_can_detector",
        output="screen",
        parameters=[{
            "blob_path": os.path.join(yolo_pkg, "models", "can_detector.blob"),
            "camera_frame": "oak_rgb_camera_optical_frame",
            "spatial_axis_map": "-z,x,y",
            "log_tf_debug": True,
        }],
        remappings=[
            ("/oakd_top/can_point_camera", "/oakd/can_detection"),
            ("/oakd_top/annotated_image", "/oakd/annotated_image")
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(oakd_node)
    
    return ld
