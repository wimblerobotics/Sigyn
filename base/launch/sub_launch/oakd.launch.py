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


sub_launch_path = os.path.join(get_package_share_directory('s_base'),
                                    'launch', 'sub_launch')
sys.path.append(sub_launch_path)
import common

def generate_launch_description():
    oakd_launch_path = PathJoinSubstitution(
        [FindPackageShare('s_base'), 'launch', 'sub_launch',
         'stereo_inertial_node.launchX.py'
        ]
    )


    ld = LaunchDescription()

    stereo_node_left = GroupAction(
        actions=[
            PushRosNamespace('oakd_left'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(oakd_launch_path),
                launch_arguments={
                    'base_frame' : 'oakd_left_base',
                    'camera_model' : 'OAK-D',
                    'parent_frame': 'oakd_left',
                    # 'rgbResolution': '1080p',
                    'tf_prefix' : 'oakd_left',
                    'mxId' : '14442C1051B665D700',
                }.items()
            )
        ]
    )
    ld.add_action(stereo_node_left)

    stereo_node_right = GroupAction(
        actions=[
            PushRosNamespace('oakd_right'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(oakd_launch_path),
                launch_arguments={
                    'base_frame' : 'oakd_right_base',
                    'camera_model' : 'OAK-D',
                    'parent_frame': 'oakd_right',
                    # 'rgbResolution': '1080p',
                    'tf_prefix' : 'oakd_right',
                    'mxId' : '14442C10910D5ED700',
                }.items()
            )
        ]
    )
    ld.add_action(stereo_node_right)

    # # stereo_node_left = GroupAction(
    # #     actions=[
    # #         PushRosNamespace('oakd_left'),
    # #         IncludeLaunchDescription(
    # #             PythonLaunchDescriptionSource(oakd_launch_path),
    # #             launch_arguments={(
    #     actions=[
    #         PushRosNamespace('oakd_left'),
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(oakd_launch_path),
    #             launch_arguments={
    #                 'base_frame' : 'oakd_left_base',
    #                 'camera_model' : 'OAK-D',
    #                 'parent_frame': 'oakd_left',
    #                 # 'rgbResolution': '1080p',
    #                 'tf_prefix' : 'oakd_left',
    #                 'mxId' : '14442C1051B665D700',
    #             }.items()
    #         )
    #     ]
    # )
    # # stereo_node_left = GroupAction(
    # #     actions=[
    # #         PushRosNamespace('oakd_left'),
    # #         IncludeLaunchDescription(
    # #             PythonLaunchDescriptionSource(oakd_launch_path),
    # #             launch_arguments={
    # #                 'base_frame' : 'oakd_left_base',
    # #                 'parent_frame': 'oakd_left',
    # #                 # 'rgbResolution': '1080p',
    # #                 'tf_prefix' : 'oakd_left',
    # #                 'mxId' : '14442C1051B665D700',
    # #             }.items()
    # #         )
    # #     ]
    # # )
    # ld.add_action(stereo_node_left)

    # # stereo_node_right = GroupAction(
    # #     actions=[
    # #         PushRosNamespace('oakd_right'),
    # #         IncludeLaunchDescription(
    # #             PythonLaunchDescriptionSource(oakd_launch_path),
    # #             launch_arguments={
    # #                 'base_frame' : 'oakd_right_base',
    # #                 'parent_frame': 'oakd_right',
    # #                 'rgbResolution': '1080p',
    # #                 'tf_prefix' : 'oakd_right',
    # #                 'mxId' : '14442C10910D5ED700',
    # #             }.items()
    # #         )
    # #     ]
    # # )
    # # ld.add_action(stereo_node_right)
    # #     ]
    # # )
    # ld.add_action(stereo_node_left)

    # # stereo_node_right = GroupAction(
    # #     actions=[
    # #         PushRosNamespace('oakd_right'),
    # #         IncludeLaunchDescription(
    # #             PythonLaunchDescriptionSource(oakd_launch_path),
    # #             launch_arguments={
    # #                 'base_frame' : 'oakd_right_base',
    # #                 'parent_frame': 'oakd_right',
    # #                 'rgbResolution': '1080p',
    # #                 'tf_prefix' : 'oakd_right',
    # #                 'mxId' : '14442C10910D5ED700',
    # #             }.items()
    # #         )
    # #     ]
    # # )
    # # ld.add_action(stereo_node_right)

    return ld