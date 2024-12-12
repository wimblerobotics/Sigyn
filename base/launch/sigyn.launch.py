# Options:
# do_joint_state_gui (false) - Flag to enable joint_state_publisher_gui
# do_rviz (true) - Launch RViz if true
# make_map (false) - Make a map vs navigate
# urdf_file_name (sigyn.urdf.xacro) - URDF file name
# use_sim_time (true) - Use simulation vs a real robot
# world (home.world) - World to load if simulating

import os
import xacro
import platform

import launch_ros.actions
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_robot_state_publisher(
    context, file_name_var, use_ros2_control, use_sim_time
):
    description_directory_path = get_package_share_directory("description")
    file_name = context.perform_substitution(file_name_var)
    print(f"[OpaqueFunction] file_name: {file_name}")
    xacro_file_path = os.path.join(description_directory_path, "urdf", file_name)
    print(f"[OpaqueFunction] xacro_file_path: {xacro_file_path}")
    print(f"[OpaqueFunction] use_sim_time: {use_sim_time.perform(context)}")
    # urdf_as_xml = Command(['xacro ', xacro_file_path, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    # print(F"urdf_as_xml: {urdf_as_xml}")

    urdf_as_xml = xacro.process_file(
        xacro_file_path, mappings={"use_ros2_control": "true", "sim_mode": "true"}
    ).toxml()
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                # 'frame_prefix': '',
                'ignore_timestamp': False,
                # 'publish_frequency': 30.0,
                "robot_description": urdf_as_xml,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    return [robot_state_publisher_node]


def generate_launch_description():
    ld = LaunchDescription()
    base_pgk = get_package_share_directory("base")
    description_pkg = get_package_share_directory("description")
    default_world = os.path.join(
        description_pkg,
        "worlds",
        "home.world",
    )

    rviz_directory_path = get_package_share_directory("rviz")
    rviz_config_path = os.path.join(rviz_directory_path, "config", "config.rviz")

    do_joint_state_gui = LaunchConfiguration("do_joint_state_gui")
    ld.add_action(
        DeclareLaunchArgument(
            name="do_joint_state_gui",
            default_value="False",
            description="Flag to enable joint_state_publisher_gui",
        )
    )

    do_rviz = LaunchConfiguration("do_rviz")
    ld.add_action(
        DeclareLaunchArgument(
            name="do_rviz", default_value="true", description="Launch RViz if true"
        )
    )

    make_map = LaunchConfiguration("make_map")
    make_map_arg = DeclareLaunchArgument(
        "make_map", default_value="False", description="Make a map vs navigate"
    )
    ld.add_action(make_map_arg)

    urdf_file_name = LaunchConfiguration("urdf_file_name")
    ld.add_action(
        DeclareLaunchArgument(
            name="urdf_file_name",
            default_value="sigyn.urdf.xacro",
            description="URDF file name",
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Simulation mode vs real robot"
    )
    ld.add_action(use_sim_time_arg)

    world = LaunchConfiguration("world")
    world_arg = DeclareLaunchArgument(
        "world", default_value=default_world, description="World to load"
    )
    ld.add_action(world_arg)

    # processor_name = subprocess.run(["uname", "-p"], capture_output=True, text=True)
    on_a_mac = platform.machine() == "aarch64"

    log_processor_action = LogInfo(
        msg=[
            f"on_a_mac: {on_a_mac}",
        ]
    )
    ld.add_action(log_processor_action)

    log_info_action = LogInfo(
        msg=[
            "do_joint_state_gui: [",
            do_joint_state_gui,
            "], do_rviz: [",
            do_rviz,
            "], make_map: [",
            make_map,
            "], urdf_file_name: [",
            urdf_file_name,
            "], use_sim_time: [",
            use_sim_time,
            "], world: [",
            world,
            "]",
        ]
    )
    ld.add_action(log_info_action)

    # Launch the robot state publisher
    ld.add_action(
        OpaqueFunction(
            function=launch_robot_state_publisher,
            args=[
                LaunchConfiguration("urdf_file_name"),
                "true",
                LaunchConfiguration("use_sim_time"),
            ],
        )
    )

    # Launch the joint state publisher GUI
    ld.add_action(
        Node(
            condition=IfCondition(do_joint_state_gui),
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
        )
    )

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
            "use_lifecycle_manager": "False",
            "use_sim_time": use_sim_time,
            "slam_params_file": os.path.join(
                base_pgk, "config", "mapper_params_online_async.yaml"
            ),
            # "params_file": "/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml",
        }.items(),
    )
    ld.add_action(slam_toolbox)

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gz_args = "-r -v4 --render-engine ogre " if on_a_mac else "-r -v4 "
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
        condition=IfCondition(use_sim_time),
        launch_arguments={
            "gz_args": [gz_args, world],
            "on_exit_shutdown": "true",
        }.items(),
    )
    ld.add_action(gazebo)

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        condition=IfCondition(use_sim_time),
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
        condition=IfCondition(use_sim_time),
        arguments=["diff_cont"],
    )
    ld.add_action(diff_drive_spawner)

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=IfCondition(use_sim_time),
        arguments=["joint_broad"],
    )
    ld.add_action(joint_broad_spawner)

    bridge_params = os.path.join(base_pgk, "config", "gz_bridge.yaml")
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        condition=IfCondition(use_sim_time),
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
        condition=IfCondition(use_sim_time),
        arguments=["/camera/image_raw"],
    )
    ld.add_action(ros_gz_image_bridge)

    # Bring up the navigation stack.
    navigation_launch_path = PathJoinSubstitution(
        [base_pgk, "launch", "nav2_bringup.launch.py"]
    )

    base_directory_path = get_package_share_directory("base")
    map_path = os.path.join(base_directory_path, "maps", "20241210l.yaml")
    nav2_config_path = os.path.join(
        base_directory_path, "config", "navigation_sim.yaml"
    )

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
            "use_sim_time": use_sim_time,
        }.items(),
    )
    ld.add_action(nav2_launch)

    echo_action = ExecuteProcess(
        cmd=["echo", "[sim] Rviz config file path: " + rviz_config_path],
        output="screen",
    )
    ld.add_action(echo_action)
    
    # Bring up the twist multiplexer.
    multiplexer_directory_path = get_package_share_directory('twist_multiplexer')
    multiplexer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [multiplexer_directory_path, '/launch/twist_multiplexer.launch.py']),
        condition=UnlessCondition(use_sim_time),
    )
    ld.add_action(multiplexer_launch)

    # Bring up the LIDAR.
    lidars_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [base_directory_path, '/launch/sub_launch/lidar.launch.py']),
        condition=UnlessCondition(use_sim_time),
    )
    ld.add_action(lidars_launch)
    
    # Bring of the EKF node.
    ekf_config_path = os.path.join(get_package_share_directory('base'), 'config/ekf.yaml')
    start_robot_localization_cmd = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        condition=UnlessCondition(use_sim_time),
        name='ekf_filter_node',
        output='screen',
        parameters=[ 
            {'use_sim_time': use_sim_time},
            ekf_config_path,
        ],
        remappings=[('/odometry/filtered', 'odom'), ('/odom/unfiltered', 'wheel_odom')]
    )
    ld.add_action(start_robot_localization_cmd)

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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(LaunchConfiguration("do_rviz")),
        arguments=["-d", rviz_config_path],
    )
    ld.add_action(rviz_node)

    return ld
