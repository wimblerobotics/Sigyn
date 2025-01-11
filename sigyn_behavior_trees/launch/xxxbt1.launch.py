import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
import launch_ros.actions
from launch_ros.actions import Node

def generate_launch_description():
    behavior_directory = get_package_share_directory('sigyn_behavior_trees')
    # xml_path = os.path.join(behavior_directory, 'config', 'bt_fall_2021.xml')
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            name="xml_path",
            default_value="foo",
            description="Flag to enable joint_state_publisher_gui",
        )
    )
    
    xml_path_substitution = PathJoinSubstitution([
      behavior_directory,
      'config',
      LaunchConfiguration('xml_path')
    ])

    behavior_node = Node(package='sigyn_behavior_trees',
                         executable='sigyn_bt1',
                         name='sigyn_bt1_node',
                         parameters=[{
                             'xml_path': xml_path_substitution
                         }],
                        #  prefix=['xterm -e gdb --args'],
                         respawn=False,
                         output='screen')
    ld.add_action(behavior_node)

    return ld
