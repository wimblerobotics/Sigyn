import os
import xacro

import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    base_pgk = get_package_share_directory("base")
    description_pkg = get_package_share_directory("description")
    default_world = os.path.join(
        description_pkg,
        "worlds",
        "home.world",
        # 'obstacles.world'
    )
    
    make_map = LaunchConfiguration("make_map")
    make_map_arg = DeclareLaunchArgument(
        "make_map", default_value="False", description="Make a map vs navigate"
    )
    ld.add_action(make_map_arg)

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use sim time if true"
    )
    ld.add_action(use_sim_time_arg)

    world = LaunchConfiguration("world")
    world_arg = DeclareLaunchArgument(
        "world", default_value=default_world, description="World to load"
    )
    ld.add_action(world_arg)

    xacro_file = os.path.join(description_pkg, "urdf", "sigyn.urdf.xacro")

    robot_state_publisher_node = launch_ros.actions.Node(
        executable="robot_state_publisher",
        output="screen",
        package="robot_state_publisher",
        parameters=[
            {
                "robot_description": xacro.process_file(
                    xacro_file,
                    mappings={"use_ros2_control": "false", "sim_mode": "true"},
                ).toxml()
            }
        ],
    )
    ld.add_action(robot_state_publisher_node)


    # Include the SLAM Toolbox launch file for mapping
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                )
            ],
        ),
        condition=IfCondition(make_map),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": "/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml",
        }.items(),
    )
    ld.add_action(slam_toolbox)

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )
    ld.add_action(gazebo)

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "sigyn",
            "-x",
            "7.3",
            "-y",
            "0.0",
            "-z",
            "0.1",
        ],
        output="screen",
    )
    ld.add_action(spawn_entity)

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    ld.add_action(diff_drive_spawner)

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    ld.add_action(joint_broad_spawner)

    bridge_params = os.path.join(base_pgk, "config", "gz_bridge.yaml")
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
    )
    ld.add_action(ros_gz_bridge)

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
    )
    ld.add_action(ros_gz_image_bridge)

    # slam_toolbox_mapper = IncludeLaunchDescription(
    #           PythonLaunchDescriptionSource([os.path.join(
    #               get_package_share_directory('base'), 'launch', 'lifelong.launch.py')]),
    #               # launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    #         )

    # Bring up the navigation stack.
    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
    )

    base_directory_path = get_package_share_directory("base")
    map_path = os.path.join(base_directory_path, "maps", "my_map_mbr.yaml")
    nav2_config_path = os.path.join(base_directory_path, "config", "navigation_sim.yaml")

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        condition=UnlessCondition(make_map),
        launch_arguments={
            "autostart": "True",
            "map": map_path,
            "params_file": nav2_config_path,
            "slam": "False",
            "use_composition": "True",
            "use_respawn": "True",
            'use_sim_time': use_sim_time,
        }.items(),
    )
    ld.add_action(nav2_launch)

    return ld
