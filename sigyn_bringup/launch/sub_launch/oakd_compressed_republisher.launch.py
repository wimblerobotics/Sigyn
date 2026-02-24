# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn

"""
Launch file to republish raw OAK-D images as compressed.
This is needed because the older depthai_examples bridge doesn't 
natively support compressed image publishing.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Republish color image as compressed
    color_republisher = Node(
        package='image_transport',
        executable='republish',
        name='oakd_color_compressed_republisher',
        namespace='oakd_top',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', 'color/image'),
            ('out/compressed', 'color/image/compressed'),
        ],
        parameters=[{
            'compressed.jpeg_quality': 80,
            'compressed.png_level': 9,
        }]
    )
    ld.add_action(color_republisher)

    return ld
