# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Use the modern depthai_ros_driver via sigyn_camera_launch
    camera_launch_path = PathJoinSubstitution(
        [FindPackageShare('base'), 'launch', 'sub_launch', 'sigyn_camera_launch.py']
    )
    
    # Use custom camera config with BEST_EFFORT QoS for real-time sensor data
    base_prefix = get_package_share_directory("base")
    default_config = os.path.join(base_prefix, "config", "oakd_camera.yaml")
    params_file = LaunchConfiguration("params_file")

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            name="params_file",
            default_value=default_config,
            description="OAK-D camera params file",
        )
    )

    # Launch OAK-D camera in oakd_top namespace
    oakd_top_camera = GroupAction(
        actions=[
            PushRosNamespace('oakd_top'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch_path),
                launch_arguments={
                    'name': 'oak',
                    'parent_frame': 'oak-d-base-frame',
                    'cam_pos_x': '0.0',
                    'cam_pos_y': '0.0',
                    'cam_pos_z': '0.0',
                    'params_file': params_file,
                    'camera_model': 'OAK-D',
                    'use_rviz': 'false',
                    'use_composition': 'true',
                    'pointcloud.enable': 'false',  # We don't need pointcloud for detection
                    'rectify_rgb': 'true',
                }.items()
            )
        ]
    )
    ld.add_action(oakd_top_camera)

    return ld