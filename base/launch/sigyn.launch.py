# Options:
# bt_xml = Full path to behavior tree overriding default_nav_to_pose_bt_xml in the navigation yaml file.
# do_joint_state_gui (false) - Flag to enable joint_state_publisher_gui.
# do_rviz (true) - Launch RViz if true.
# make_map (false) - Make a map vs navigate.
# urdf_file_name (sigyn.urdf.xacro) - URDF file name.
# use_sim_time (true) - Use simulation vs a real robot.
# world (home.world) - World to load if simulating.

import os
import platform
import tempfile
import xacro
import yaml

import launch_ros.actions
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    Command,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
    # PythonExpression,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_nav(context, ld, nav2_config_path, bt_xml, make_map, use_sim_time, map_path_sim, map_path_real, navigation_launch_path): 
    # Create our own temporary YAML files that include substitutions
    with open(nav2_config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    replacement_xml_path = context.perform_substitution(bt_xml)
    xml_path = config_yaml["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"]
    config_yaml["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"] = replacement_xml_path
    # config_yaml["map_server"]["ros__parameters"]["yaml_filename"] = map_path_sim ###
    nav_config_tempfile = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
    # print(f"[launch_nav] fp.name: {nav_config_tempfile.name}, replacement_xml_path: {replacement_xml_path}")
    with open(nav_config_tempfile.name, 'w') as f:
        yaml.dump(config_yaml, f)

    # nav_sim= IfCondition(AndSubstitution(make_map, use_sim_time))
    # nav_real = IfCondition(AndSubstitution(make_map, NotSubstitution(use_sim_time)))

    log_nav_params = LogInfo(
        msg=[
            f"[launch_nav] map_path_sim: {map_path_sim}, map_path_real: {map_path_real}, replacement_xml_path: {replacement_xml_path}, navigation_launch_path: {context.perform_substitution(navigation_launch_path)}",
        ]
    )
    ld.add_action(log_nav_params)

    nav2_launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        # condition=IfCondition(AndSubstitution(NotSubstitution(make_map), use_sim_time)),
        condition=IfCondition(AndSubstitution(NotSubstitution(make_map), use_sim_time)),
        launch_arguments={
            "autostart": "True",
            "map": map_path_sim,
            "params_file": nav_config_tempfile.name,
            "slam": "False",
            "use_composition": "True",
            "use_respawn": "True",
            "use_sim_time": use_sim_time,
        }.items(),
    )
    ld.add_action(nav2_launch_sim)

    nav2_launch_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        condition=IfCondition(
            AndSubstitution(NotSubstitution(make_map), NotSubstitution(use_sim_time))
        ),
        launch_arguments={
            "autostart": "True",
            "map": map_path_real,
            "params_file": nav_config_tempfile.name,
            "slam": "False",
            "use_composition": "True",
            "use_respawn": "True",
            "use_sim_time": use_sim_time,
        }.items(),
    )
    ld.add_action(nav2_launch_real)
    
def launch_robot_state_publisher(
    context, file_name_var, use_ros2_control, use_sim_time
):
    description_directory_path = get_package_share_directory("description")
    file_name = context.perform_substitution(file_name_var)
    print(f"[launch_robot_state_publisher] file_name: {file_name}")
    xacro_file_path = os.path.join(description_directory_path, "urdf", file_name)
    print(f"[launch_robot_state_publisher] xacro_file_path: {xacro_file_path}")
    print(f"[launch_robot_state_publisher] use_sim_time: {use_sim_time.perform(context)}")
    # urdf_as_xml = Command(['xacro ', xacro_file_path, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    # print(F"urdf_as_xml: {urdf_as_xml}")

    urdf_as_xml = xacro.process_file(
        xacro_file_path, mappings={"use_ros2_control": "true", "sim_mode": use_sim_time.perform(context)}
    ).toxml()
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                # 'frame_prefix': '',
                "ignore_timestamp": False,
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

    bt_xml = LaunchConfiguration("bt_xml")
    bt_xml_arg = DeclareLaunchArgument(
        "bt_xml",
        default_value="/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml",
        description="XML to use for nav_to_pose",
    )
    ld.add_action(bt_xml_arg)

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
        "use_sim_time",
        default_value="false",
        description="Simulation mode vs real robot",
    )
    ld.add_action(use_sim_time_arg)

    world = LaunchConfiguration("world")
    world_arg = DeclareLaunchArgument(
        "world", default_value=default_world, description="World to load"
    )
    ld.add_action(world_arg)

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
            "8.81",
            "-y",
            "2.59",
            "-z",
            "0.0",
        ],
        output="screen",
    )
    ld.add_action(spawn_entity)
    

    controller_params_file = PathJoinSubstitution(
        [description_pkg, "config", "my_controllers.yaml"])
    controller_manager = Node(
        condition=UnlessCondition(use_sim_time),
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file],
        output="both",
        remappings=[("~/robot_description", "/robot_description")],
    )
    ld.add_action(controller_manager)

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
    
    fwcommand_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", controller_params_file],
    )
    ld.add_action(fwcommand_spawner)

    # joint_state_broadcaster_spawner = Node(
    #     condition=UnlessCondition(use_sim_time),
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster"],
    # )
    # ld.add_action(joint_state_broadcaster_spawner)

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
 
    map_path_sim = os.path.join(base_pgk, "maps", "map2.yaml")
    map_path_real = os.path.join(base_pgk, "maps", "20241210l.yaml")
    
    nav2_config_path = os.path.join(
        base_pgk, "config", "navigation_sim.yaml"
        # "/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml"
    )         

    ld.add_action(
        OpaqueFunction(
            function=launch_nav,
            args=[
                ld,
                nav2_config_path,
                LaunchConfiguration("bt_xml"),
                LaunchConfiguration("make_map"),
                LaunchConfiguration("use_sim_time"),
                map_path_sim,
                map_path_real,
                navigation_launch_path,
            ],
        )
    )
    # nav2_launch_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(navigation_launch_path),
    #     # condition=IfCondition(AndSubstitution(NotSubstitution(make_map), use_sim_time)),
    #     # condition=IfCondition(AndSubstitution(NotSubstitution(make_map), use_sim_time)),
    #     launch_arguments={
    #         "autostart": "True",
    #         "map": map_path_sim,
    #         "params_file": nav2_config_path,
    #         "slam": "False",
    #         "use_composition": "True",
    #         "use_respawn": "True",
    #         "use_sim_time": use_sim_time,
    #     }.items(),
    # )
    # ld.add_action(nav2_launch_sim)

    
    echo_action = ExecuteProcess(
        cmd=["echo", "[sim] Rviz config file path: " + rviz_config_path],
        output="screen",
    )
    ld.add_action(echo_action)

    # Bring up the twist multiplexer.
    multiplexer_directory_path = get_package_share_directory("twist_multiplexer")
    multiplexer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [multiplexer_directory_path, "/launch/twist_multiplexer.launch.py"]
        ),
        condition=UnlessCondition(use_sim_time),
    )
    ld.add_action(multiplexer_launch)

    # Bring up the LIDAR.
    lidars_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [base_pgk, "/launch/sub_launch/lidar.launch.py"]
        ),
        condition=UnlessCondition(use_sim_time),
    )
    ld.add_action(lidars_launch)

    # Bring of the EKF node.
    ekf_config_path = os.path.join(
        get_package_share_directory("base"), "config/ekf.yaml"
    )
    start_robot_localization_cmd = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        condition=UnlessCondition(use_sim_time),
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            ekf_config_path,
        ],
        remappings=[("/odometry/filtered", "odom"), ("/odom/unfiltered", "wheel_odom")],
    )
    ld.add_action(start_robot_localization_cmd)

    # Publish joints.
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        condition=UnlessCondition(use_sim_time),
        executable="joint_state_publisher",
        name="joint_state_publisher",
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
    
    oakd_elevator_top = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          [base_pgk, "/launch/sub_launch/oakd_stereo.launch.py"]
        )
    )
    ld.add_action(oakd_elevator_top)
    
    pc2ls = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_node",
        output="screen",
        parameters=[
            {"target_frame": "base_link",
             "min_height": 0.13,
             "max_height": 2.0,
             "range_min": 0.5,
             "range_max": 10.0,
             "scan_time": 0.1,
             "use_inf": True,
            },
        ],
        remappings=[
          ("/cloud_in", "/stereo/points"),
          ("/scan", "/stereo/points2")],
    )
    ld.add_action(pc2ls)
    
    # <node pkg="pointcloud_to_laserscan" type="pointcloud_to_laserscan_node" name="downsampled_points">
    # <param name="target_frame" value="map"/>
    # <param name="min_height" value="0.0"/>
    # <param name="max_height" value="1.0"/>
    # <param name="range_min" value="0.5"/>
    # <param name="range_max" value="10.0"/>
    # <param name="scan_time" value="0.1"/>
    # <param name="use_inf" value="true"/>
    # </node>

    SaySomethingActionServer = Node(
        package="sigyn_behavior_trees",
        executable="SaySomethingActionServer",
        name="SaySomethingActionServer",
        # prefix=['xterm -e gdb -ex run --args'],
    )
    ld.add_action(SaySomethingActionServer)
    
    MoveAShortDistanceAheadActionServer = Node(
        package="sigyn_behavior_trees",
        executable="MoveAShortDistanceAheadActionServer",
        name="MoveAShortDistanceAheadActionServer",
        # prefix=['xterm -e gdb -ex run --args'],
    )
    ld.add_action(MoveAShortDistanceAheadActionServer)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(LaunchConfiguration("do_rviz")),
        arguments=["-d", rviz_config_path],
    )
    ld.add_action(rviz_node)

    return ld
