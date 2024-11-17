import os
import sys
import xacro
import yaml

import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  use_sim_time = LaunchConfiguration('use_sim_time')
  base_pgk = get_package_share_directory("base")
  description_pkg = get_package_share_directory("description")
  xacro_file = os.path.join(description_pkg, 'urdf', 'base.urdf.xacro')

  robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', 'false', ' sim_mode:=', use_sim_time])
  robot_description_raw = xacro.process_file(xacro_file).toxml()

  robot_state_publisher_node = launch_ros.actions.Node(
      executable='robot_state_publisher',
      output='screen',
      package='robot_state_publisher',
      parameters=[{'robot_description': robot_description_raw,
                    'use_sim_time': use_sim_time
                    }]
  )
  
  
  default_world = os.path.join(
    description_pkg,
    'worlds',
    'home.world'
    # 'obstacles.world'
    )    

  world = LaunchConfiguration('world')

  world_arg = DeclareLaunchArgument(
      'world',
      default_value=default_world,
      description='World to load'
      )

  # Include the Gazebo launch file, provided by the ros_gz_sim package
  gazebo = IncludeLaunchDescription(
              PythonLaunchDescriptionSource([os.path.join(
                  get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                  launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
            )

  # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
  spawn_entity = Node(package='ros_gz_sim', executable='create',
                      arguments=['-topic', 'robot_description',
                                  '-name', 'sigyn',
                                  '-z', '0.1'],
                      output='screen')

  diff_drive_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["diff_cont"],
  )

  joint_broad_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=["joint_broad"],
  )


  bridge_params = os.path.join(base_pgk,'config','gz_bridge.yaml')
  ros_gz_bridge = Node(
      package="ros_gz_bridge",
      executable="parameter_bridge",
      arguments=[
          '--ros-args',
          '-p',
          f'config_file:={bridge_params}',
      ]
  )

  # ros_gz_image_bridge = Node(
  #     package="ros_gz_image",
  #     executable="image_bridge",
  #     arguments=["/camera/image_raw"]
  # )

  return launch.LaunchDescription([
    DeclareLaunchArgument(
      'use_sim_time',
      default_value='true',
      description='Use sim time if true'),
    world_arg,
    gazebo,
    robot_state_publisher_node,
    spawn_entity,
    diff_drive_spawner,
    joint_broad_spawner,
    ros_gz_bridge,
    # ros_gz_image_bridge
  ])
