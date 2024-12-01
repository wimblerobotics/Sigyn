import os
import sys
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
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

  cartographer_config_dir = os.path.join(common.base_directory_path, 'config')
  configuration_basename = os.path.join(common.base_directory_path, 'config', 'cartographer.lua')

  # cartographer_launch = Node(
  #         package='cartographer_ros',
  #         executable='cartographer_node',
  #         name='cartographer_node',
  #         output='screen',
  #         parameters=[{'use_sim_time': False}],
  #         arguments=['-configuration_directory', cartographer_config_dir,
  #                    '-configuration_basename', configuration_basename]),
  # common.ld.add_action(cartographer_launch)
  cartographer_node = Node(
    package='cartographer_ros',
    executable='cartographer_node',
    parameters=[{'use_sim_time': False}],
    arguments=[
        '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
        '-configuration_basename', 'backpack_2d.lua'
    ],
    remappings=[
        ('echoes', 'horizontal_laser_2d')
    ],
    output='screen'
  )
  common.ld.add_action(cartographer_node)

  
  cartographer_occupancy_node = Node(
          package='cartographer_ros',
          executable='cartographer_occupancy_grid_node',
          name='cartographer_occupancy_grid_node',
          output='screen',
          parameters=[{'use_sim_time': False}],
          arguments=['-resolution', '0.05', 'publish_period_sec', '0.5'])
  common.ld.add_action(cartographer_occupancy_node)
  
  return common.ld