# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn

import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import OpaqueFunction


def launch_robot_state_publisher(context, file_name_var, use_sim_time):
    description_directory_path = get_package_share_directory('description')
    file_name = context.perform_substitution(file_name_var)
    print(F"file_name: {file_name}")
    xacro_file_path = os.path.join(
        description_directory_path, 'urdf', file_name)
    print(F"xacro_file_path: {xacro_file_path}")
    urdf_as_xml = xacro.process_file(xacro_file_path).toxml()
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'frame_prefix': '',
                'ignore_timestamp': False,
                'publish_frequency': 30.0,
                'use_sim_time': use_sim_time,
                'robot_description':  urdf_as_xml
            }
        ]
    )
    return [robot_state_publisher_node]


def generate_launch_description():
    do_rviz = LaunchConfiguration('do_rviz')
    gui = LaunchConfiguration('gui')
    publish_joints = LaunchConfiguration('publish_joints')
    urdf_file_name = LaunchConfiguration('urdf_file_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld = LaunchDescription()

    description_directory_path = get_package_share_directory('description')
    rviz_directory_path = get_package_share_directory('rviz')

    ld.add_action(DeclareLaunchArgument(
        name='do_rviz',
        default_value='true',
        description='Launch RViz if true'))
    ld.add_action(DeclareLaunchArgument(
        name='gui',
        default_value='False',
        description='Flag to enable joint_state_publisher_gui'))
    ld.add_action(DeclareLaunchArgument(
        name='publish_joints',
        default_value='True',
        description='Launch joint_states_publisher if true'))

    ld.add_action(DeclareLaunchArgument(
        name='urdf_file_name',
        default_value='sigyn.urdf.xacro',
        description='URDF file name'))

    ld.add_action(DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))

    log_info_action = LogInfo(
        msg=[
            "description.launch.py, do_rviz: ",
            do_rviz,
            ", gui:", gui,
            ", publish_joints: ", publish_joints,
            ", urdf_file_name: ", urdf_file_name,
            ", use_sim_time: ", use_sim_time
        ]
    )
    ld.add_action(log_info_action)
    
     # Publish joints.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        condition=UnlessCondition(use_sim_time),
        executable='joint_state_publisher',
        name='joint_state_publisher',
        ### condition=IfCondition(LaunchConfiguration("publish_joints")),
        # parameters=[
        #     {
        #         'delta': 0.0,
        #         'publish_default_efforts': False,
        #         'publish_default_positions': True,
        #         'publish_default_velocities': False,
        #         'rate': 30.0,
        #         'use_mimic_tag': True,
        #         'use_smallest_joint_limits': True
        #     }
        # ]
    )
    ld.add_action(joint_state_publisher_node)

    # ld.add_action(Node(
    #     condition=IfCondition(gui),
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui'))

    ld.add_action(OpaqueFunction(function=launch_robot_state_publisher, args=[
        LaunchConfiguration('urdf_file_name'), LaunchConfiguration('use_sim_time')]))
    
    echo_action = ExecuteProcess(
        cmd=['echo', 'Rviz config file path: ' +
             os.path.join(rviz_directory_path, 'config', 'config.rviz')],
        output='screen'
    )
    ld.add_action(echo_action)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration("do_rviz")),
        arguments=[
            '-d', os.path.join(rviz_directory_path, 'config', 'config.rviz')],
    )
    ld.add_action(rviz_node)
    
    ld.add_action(LogInfo(
      msg=["[description] URDF file name: ", LaunchConfiguration('urdf_file_name'), " use_sim_time: ", LaunchConfiguration('use_sim_time')]))

    return ld
