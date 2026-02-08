#!/usr/bin/env python3
"""
Minimal OAK-D detection test launch.

Runs:
- robot description (TF tree)
- yolo_oakd_test detector node
- can_do_challenge BT using bt_xml/oakd_detection_test1.xml
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
    yolo_pkg = get_package_share_directory("yolo_oakd_test")

    sigyn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_pkg, "launch", "sigyn.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "do_rviz": "true",
            "make_map": "false",
            "do_oakd": "false",
            "do_oakd_yolo26": "false",
            "oakd_params_file": os.path.join(base_pkg, "config", "oakd_camera_can_yolov8.yaml"),
            "do_top_lidar": "true",
        }.items(),
    )

    oakd_node = Node(
        package="yolo_oakd_test",
        executable="oakd_can_detector.py",
        name="oakd_can_detector_custom",
        output="screen",
        parameters=[{
            "blob_path": os.path.join(yolo_pkg, "models", "can_detector.blob"),
            "camera_frame": "oak_rgb_camera_optical_frame",
            "spatial_axis_map": "-z,x,y",
            "log_tf_debug": False,
        }],
        remappings=[
            ("/oakd_top/can_point_camera", "/oakd/can_detection"),
            ("/oakd_top/annotated_image", "/oakd/annotated_image"),
        ],
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
            "bt_xml_filename": os.path.join(can_do_pkg, "bt_xml", "oakd_detection_test1.xml"),
        }],
    )

    return LaunchDescription([
        sigyn_launch,
        oakd_node,
        can_do_node,
    ])