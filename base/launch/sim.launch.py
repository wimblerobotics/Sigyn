import os
import xacro

import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    base_pgk = get_package_share_directory("base")
    description_pkg = get_package_share_directory("description")
    xacro_file = os.path.join(description_pkg, 'urdf', 'sigyn.urdf.xacro')

    robot_state_publisher_node = launch_ros.actions.Node(
        executable='robot_state_publisher',
        output='screen',
        package='robot_state_publisher',
        parameters=[{'robot_description': xacro.process_file(
          xacro_file,
          mappings={'use_ros2_control': 'false', 'sim_mode': 'true'}).toxml()}],
    )

    default_world = os.path.join(
        description_pkg,
        'worlds',
        'home.world'
        # 'obstacles.world'
    )

    world = LaunchConfiguration('world')

    # Include the SLAM Toolbox launch file for mapping
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': [
            '-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'sigyn',
                                   '-x', '7.3',
                                   '-y', '0.0',
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

    bridge_params = os.path.join(base_pgk, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    # slam_toolbox_mapper = IncludeLaunchDescription(
    #           PythonLaunchDescriptionSource([os.path.join(
    #               get_package_share_directory('base'), 'launch', 'lifelong.launch.py')]),
    #               # launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    #         )

    # Bring up the navigation stack.
    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    base_directory_path = get_package_share_directory('base')
    map_path = os.path.join(base_directory_path, 'maps', 'snowberry4v31.yaml')
    nav2_config_path = os.path.join(
        base_directory_path, 'config', 'navigation.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            'autostart': 'True',
            'map': map_path,
            'params_file': nav2_config_path,
            'slam': 'False',
            'use_composition': 'True',
            'use_respawn': 'True',
            # 'use_sim_time': 'false',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        world_arg,
        gazebo,
        nav2_launch,
        robot_state_publisher_node,
        # slam_toolbox_mapper,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        ros_gz_bridge,
        ros_gz_image_bridge
    ])
