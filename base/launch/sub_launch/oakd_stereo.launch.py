import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from  launch_ros.actions import PushRosNamespace
import launch_ros.actions
import launch_ros.descriptions
from launch_ros.substitutions import FindPackageShare
import sys


def generate_launch_description():
    oakd_launch_path = PathJoinSubstitution(
        [FindPackageShare('base'), 'launch', 'sub_launch',
         'stereo_inertial_node.launchX.py'
        ]
    )


    ld = LaunchDescription()

    stereo_node_left = GroupAction(
        actions=[
            PushRosNamespace('oakd_top'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(oakd_launch_path),
                launch_arguments={
                    'base_frame' : 'oak',
                    'camera_model' : 'OAK-D',
                    # 'parent_frame': 'oakd_left',
                    # 'rgbResolution': '1080p',
                    'tf_prefix' : 'oak',
                    # 'mxId' : '14442C1051B665D700',
                }.items()
            )
        ]
    )
    ld.add_action(stereo_node_left)

    return ld