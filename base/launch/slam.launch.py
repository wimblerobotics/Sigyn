import os
import sys
import xacro
import yaml

# import launch
import launch_ros.actions
# from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


sub_launch_path = os.path.join(get_package_share_directory('base'), 'launch', 'sub_launch')
sys.path.append(sub_launch_path)
import common

def generate_launch_description():

    base_launch_path = PathJoinSubstitution(
        [FindPackageShare('base'), 'launch', 'sub_launch', 'base.launch.py']
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path),
        launch_arguments={
            'publish_joints': 'true',
        }.items()
    )
    common.ld.add_action(base_launch)

    # Bring up the navigation stack.
    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    nav2_config_path =os.path.join(common.base_directory_path, 'config', 'navigation.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            'autostart': 'True',
            'map': common.map_path,
            # 'params_file': nav2_config_path,
            'slam': 'False',
            'use_composition': 'True',
            'use_respawn': 'True',
            # 'use_sim_time': 'false',
        }.items()
    )
    # common.ld.add_action(nav2_launch)
    
    # # Bring up the slam_toolbox.
    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    slam_config_path =os.path.join(common.base_directory_path, 'config', 'navigation.yaml')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'autostart': 'True',
            'map': common.map_path,
            'params_file': slam_config_path,
            # 'slam': 'False',
            # 'use_composition': 'True',
            # 'use_respawn': 'True',
            # 'use_sim_time': 'false',
        }.items()
    )
    common.ld.add_action(slam_launch)

    return common.ld
