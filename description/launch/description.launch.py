import os
import xacro
import yaml

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch.actions import LogInfo
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals

def generate_launch_description():
    
    publish_joints = LaunchConfiguration('publish_joints')
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld = LaunchDescription()
    description_directory_path = get_package_share_directory('description')

    ld.add_action(DeclareLaunchArgument(
        name='publish_joints', 
        default_value='true',
        description='Launch joint_states_publisher if true'))

    ld.add_action(DeclareLaunchArgument(
        name='use_sim_time', 
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))
    
    
    # Publish joints.
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration("publish_joints")),
        parameters=[
            {
                'use_sim_time': use_sim_time
            }
        ]
    )
    ld.add_action(joint_state_publisher_node)

    # Publish robot state and URDF.
    urdf_path = os.path.join(description_directory_path, 'urdf/', 'base.urdf.xacro')
    processed_urdf = xacro.process_file(urdf_path).toxml()
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': processed_urdf,
                'use_sim_time': use_sim_time
            }
        ]
    )
    ld.add_action(robot_state_publisher_node)

    return ld
