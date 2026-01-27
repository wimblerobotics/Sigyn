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
    """Launch YOLO26 OAK-D detector for Coke can detection."""
    
    # Path to yolo26_oakd_detector launch file
    yolo26_launch_path = PathJoinSubstitution(
        [FindPackageShare('oakd_detector'), 'launch', 'yolo26_oakd_detector.launch.py']
    )
    
    # Include the YOLO26 detector launch file
    yolo26_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(yolo26_launch_path),
        launch_arguments={
            'mxId': '14442C1051B665D700',
            'model_path': os.path.join(
                get_package_share_directory('oakd_detector'),
                'resources',
                'best.pt'  # RoboFlow v3 model
            ),
            'confidence_threshold': '0.5',
            'image_size': '640',
            'trained_images_dir': '/home/ros/sigyn_ws/src/Sigyn/trained_images',
        }.items()
    )
    
    ld = LaunchDescription()
    ld.add_action(yolo26_detector)
    
    return ld
