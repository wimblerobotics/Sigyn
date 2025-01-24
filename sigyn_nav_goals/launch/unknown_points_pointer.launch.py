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

  # autostart = LaunchConfiguration('autostart')
  # declare_autostart_cmd = DeclareLaunchArgument(
  #       'autostart',
  #       default_value='true',
  #       description='Automatically startup the nav2 stack',
  #   )
  # ld.add_action(declare_autostart_cmd)

  # use_sim_time = LaunchConfiguration('use_sim_time')
  # declare_use_sim_time_cmd = DeclareLaunchArgument(
  #   'use_sim_time',
  #   default_value='true',
  #   description='Use simulation (Gazebo) clock if true',
  # )
  # ld.add_action(declare_use_sim_time_cmd)

  # param_substitutions = {
  #         'use_sim_time': use_sim_time,
  #         'autostart': autostart}
  # base_pgk = get_package_share_directory("base")
  # nav2_config_path = os.path.join(
  #     base_pgk, "config", "navigation_sim.yaml"
  # )   

  # configured_params = ParameterFile(
  #     RewrittenYaml(
  #         source_file=nav2_config_path,
  #         root_key='',
  #         param_rewrites=param_substitutions,
  #         convert_types=True,
  #     ),
  #     ### allow_substs=True,
  # )
  
  # btn = Node(
  #   package='nav2_bt_navigator',
  #   executable='bt_navigator',
  #   name='bt_navigator',
  #   output='screen',
  #   # respawn=use_respawn,
  #   respawn_delay=2.0,
  #   parameters=[configured_params],
  #   # arguments=['--ros-args', '--log-level', log_level],
  #   # remappings=remappings,
  # )
  # ld.add_action(btn)
  
  # lifecycle_nodes = [
  #     'bt_navigator',
  # ]

  # lcm = Node(
  #     package='nav2_lifecycle_manager',
  #     executable='lifecycle_manager',
  #     name='lifecycle_manager_navigation',
  #     output='screen',
  #     # arguments=['--ros-args', '--log-level', log_level],
  #     parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
  # )
  # ld.add_action(lcm)

  upfn = Node(
    package='sigyn_nav_goals',
    executable='unknown_points_finder_node',
    name='unknown_points_pointer',
    output='screen',
    parameters=[{'param_name': 'param_value'}]
  )
  ld.add_action(upfn)
  return ld
