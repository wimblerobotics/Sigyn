from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='sigyn_nav_goals',
      executable='unknown_points_finder_node',
      name='unknown_points_pointer',
      output='screen',
      parameters=[{'param_name': 'param_value'}]
    )
  ])