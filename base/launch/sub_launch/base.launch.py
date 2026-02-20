import os
import sys
import xacro
import yaml

import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    ekf_config_path = os.path.join(get_package_share_directory('base'), 'config/ekf.yaml')
    multiplexer_directory_path = get_package_share_directory('wr_twist_multiplexer');
    base_directory_path = get_package_share_directory('base')
    description_directory_path = get_package_share_directory('sigyn_description')
    use_sim_time = False
    publish_joints = LaunchConfiguration('publish_joints')
    ld.add_action(DeclareLaunchArgument(
        name='publish_joints', 
        default_value='true',
        description='Launch joint_states_publisher if true'))

    # Bring up the twist multiplexer.
    multiplexer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [multiplexer_directory_path, '/launch/wr_twist_multiplexer.launch.py'])
    )
    ld.add_action(multiplexer_launch)

    # Bring up the robot description (URDF).
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            description_directory_path, '/launch/description.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'publish_joints': publish_joints,
            'urdf_file_name': 'sigyn.urdf.xacro'
        }.items()
    )
    ld.add_action(description_launch)

    # Bring up the LIDAR.
    lidars_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [base_directory_path, '/launch/sub_launch/lidar.launch.py']))
    ld.add_action(lidars_launch)
    
    # Bring of the EKF node.
    start_robot_localization_cmd = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ 
            {'use_sim_time': use_sim_time},
            ekf_config_path,
        ],
        remappings=[('/odometry/filtered', 'odom'), ('/odom/unfiltered', '/sigyn/wheel_odom')]
    )
    ld.add_action(start_robot_localization_cmd)
    
    # # Bring up the OAK-Ds
    # oakds_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [base_directory_path, '/launch/sub_launch/oakds.launch.py']))
    # ld.add_action(oakds_launch)

    return ld
