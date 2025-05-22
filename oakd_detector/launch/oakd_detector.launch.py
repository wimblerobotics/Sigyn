#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    oakd_node = Node(
        package='oakd_detector',
        executable='oakd_detector_node',
        name='oakd_detector_node',
        output='screen',
        parameters=[
            {'mxId': '14442C1051B665D700'},
            {'nnName': 'yolov4_tiny_coco_416x416_openvino_2021.4_6shave_bgr.blob'},
            {'previewWidth': 416},
            {'previewHeight': 416}
        ]
    )
    return LaunchDescription([oakd_node])