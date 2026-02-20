from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch.substitutions import LaunchConfiguration, PythonExpression
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():
  ld = LaunchDescription()

  upfn = Node(
    package='sigyn_nav_goals',
    executable='unknown_points_finder_node',
    name='unknown_points_pointer',
    output='screen',
    parameters=[{'param_name': 'param_value'}]
  )
  ld.add_action(upfn)
  return ld
