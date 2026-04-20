# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Wimble Robotics

"""sigyn.launch.py — Full bringup for the Sigyn house-patrol robot.

Launch arguments:
  bt_xml                    Full path to BT XML overriding default_nav_to_pose_bt_xml.
  do_joint_state_gui        (false)  Enable joint_state_publisher_gui.
  do_joystick               (false)  Launch nimbus_steelseries_joystick.
  do_oakd                   (false)  Launch OAK-D camera nodes.
  do_oakd_yolo26            (true)   Launch YOLO26 CPU detector (requires do_oakd=true).
  do_pi_cam                 (false)  Launch Pi Camera gripper detector.
  do_rviz                   (true)   Launch RViz.
  do_top_lidar              (true)   Include top LiDAR in URDF.
  urdf_file_name            (sigyn.urdf.xacro)  URDF/xacro file name.
  use_compressed_rviz_feeds (true)   Republish compressed feeds locally for RViz.
  use_sim_time              (false)  Use simulation clock.
  world                     (home.world)  Gazebo world to load (sim only).
"""

import os
import platform

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Evaluated once at generation time.  Used to select the Gazebo render engine
# and optionally force software rendering for RViz on Apple Silicon.
_ON_MAC = platform.machine() == "aarch64"


# ---------------------------------------------------------------------------
# OpaqueFunction helpers — need a resolved context to work.
# ---------------------------------------------------------------------------

def _launch_robot_state_publisher(context, urdf_file_name, use_sim_time, do_top_lidar):
    """Process the xacro file and return a robot_state_publisher Node.

    Real Teensy boards own hardware control, so ros2_control is only enabled
    in simulation.
    """
    description_pkg = get_package_share_directory("sigyn_description")
    xacro_path = os.path.join(
        description_pkg, "urdf", context.perform_substitution(urdf_file_name)
    )
    sim = context.perform_substitution(use_sim_time)
    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "use_ros2_control": sim,
            "sim_mode":         sim,
            "do_top_lidar":     context.perform_substitution(do_top_lidar),
        },
    ).toxml()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "ignore_timestamp":   False,
                "robot_description":  robot_description,
                "use_sim_time":       use_sim_time,
            }],
        )
    ]


def _launch_sigyn_to_teensy(context, use_sim_time):
    """Include wr_ros_teensy teensy_bridge launch only on the real robot."""
    if context.perform_substitution(use_sim_time).lower() == "true":
        return []
    pkg = get_package_share_directory("wr_ros_teensy")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "teensy_bridge.launch.py")
            ),
        )
    ]


def _launch_oakd_yolo26(context, bringup_pkg, use_sim_time, do_oakd, do_oakd_yolo26):
    """Include YOLO26 detector launch only when explicitly enabled on real robot."""
    if context.perform_substitution(use_sim_time).lower() == "true":
        return []
    if context.perform_substitution(do_oakd).lower() != "true":
        return []
    if context.perform_substitution(do_oakd_yolo26).lower() != "true":
        return []

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    bringup_pkg,
                    "launch",
                    "sub_launch",
                    "oakd_yolo26_detector.launch.py",
                )
            ),
        )
    ]


# ---------------------------------------------------------------------------
# Main entry point.
# ---------------------------------------------------------------------------

def generate_launch_description():
    bringup_pkg     = get_package_share_directory("sigyn_bringup")
    description_pkg = get_package_share_directory("sigyn_description")
    rviz_pkg        = get_package_share_directory("rviz")

    # -----------------------------------------------------------------------
    # Paths resolved at generation time.
    # -----------------------------------------------------------------------
    default_world    = os.path.join(description_pkg, "worlds",  "home.world")
    rviz_config      = os.path.join(rviz_pkg,        "config",  "config.rviz")
    map_path         = os.path.join(bringup_pkg,     "maps",    "my_map.yaml")
    nav2_config      = os.path.join(bringup_pkg,     "config",  "navigation.yaml")
    ekf_config       = os.path.join(bringup_pkg,     "config",  "ekf.yaml")
    gz_bridge_config = os.path.join(bringup_pkg,     "config",  "gz_bridge.yaml")
    nav2_launch_path = os.path.join(bringup_pkg,     "launch",  "nav2_bringup.launch.py")
    twist_mux_config = os.path.join(
        get_package_share_directory("wr_twist_multiplexer"),
        "config", "wr_twist_multiplexer.yaml",
    )

    # -----------------------------------------------------------------------
    # Declare all launch arguments up-front (must precede any LaunchConfiguration).
    # -----------------------------------------------------------------------
    declared_args = [
        DeclareLaunchArgument(
            "bt_xml",
            default_value=(
                "/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/"
                "navigate_to_pose_w_replanning_and_recovery.xml"
            ),
            description="Full path to BT XML for nav_to_pose",
        ),
        DeclareLaunchArgument(
            "do_joint_state_gui",
            default_value="false",
            description="Enable joint_state_publisher_gui",
        ),
        DeclareLaunchArgument(
            "do_joystick",
            default_value="false",
            description="Launch nimbus_steelseries_joystick node",
        ),
        DeclareLaunchArgument(
            "do_oakd",
            default_value="false",
            description="Launch OAK-D camera nodes",
        ),
        DeclareLaunchArgument(
            "do_oakd_yolo26",
            default_value="true",
            description="Launch YOLO26 CPU detector (requires do_oakd=true)",
        ),
        DeclareLaunchArgument(
            "do_pi_cam",
            default_value="false",
            description="Launch Pi Camera gripper detector",
        ),
        DeclareLaunchArgument(
            "do_rviz",
            default_value="true",
            description="Launch RViz",
        ),
        DeclareLaunchArgument(
            "do_top_lidar",
            default_value="true",
            description="Include top LiDAR in URDF (false = single LiDAR at cup origin)",
        ),
        DeclareLaunchArgument(
            "urdf_file_name",
            default_value="sigyn.urdf.xacro",
            description="URDF/xacro file under sigyn_description/urdf/",
        ),
        DeclareLaunchArgument(
            "use_compressed_rviz_feeds",
            default_value="true",
            description="Republish compressed image feeds locally for RViz Image displays",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock (true) vs real robot (false)",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description="Gazebo world file to load (simulation only)",
        ),
    ]

    # -----------------------------------------------------------------------
    # LaunchConfiguration handles (substitutions, not Python values).
    # -----------------------------------------------------------------------
    bt_xml                    = LaunchConfiguration("bt_xml")
    do_joint_state_gui        = LaunchConfiguration("do_joint_state_gui")
    do_joystick               = LaunchConfiguration("do_joystick")
    do_oakd                   = LaunchConfiguration("do_oakd")
    do_oakd_yolo26            = LaunchConfiguration("do_oakd_yolo26")
    do_pi_cam                 = LaunchConfiguration("do_pi_cam")
    do_rviz                   = LaunchConfiguration("do_rviz")
    do_top_lidar              = LaunchConfiguration("do_top_lidar")
    urdf_file_name            = LaunchConfiguration("urdf_file_name")
    use_compressed_rviz_feeds = LaunchConfiguration("use_compressed_rviz_feeds")
    use_sim_time              = LaunchConfiguration("use_sim_time")
    world                     = LaunchConfiguration("world")

    # -----------------------------------------------------------------------
    # Informational log (useful when inspecting launch output).
    # -----------------------------------------------------------------------
    log_config = LogInfo(
        msg=[
            "bt_xml: [", bt_xml, "]",
            "  do_joint_state_gui: [", do_joint_state_gui, "]",
            "  do_rviz: [", do_rviz, "]",
            "  do_top_lidar: [", do_top_lidar, "]",
            "  urdf_file_name: [", urdf_file_name, "]",
            "  use_sim_time: [", use_sim_time, "]",
            "  world: [", world, "]",
        ]
    )

    # -----------------------------------------------------------------------
    # Robot description (always launched).
    # -----------------------------------------------------------------------
    robot_state_publisher = OpaqueFunction(
        function=_launch_robot_state_publisher,
        args=[urdf_file_name, use_sim_time, do_top_lidar],
    )

    # GUI variant for development; plain publisher on the real robot.
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(do_joint_state_gui),
    )
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(use_sim_time),
    )

    # -----------------------------------------------------------------------
    # Simulation — Gazebo + controller spawners + topic bridges.
    # -----------------------------------------------------------------------

    # Mac uses ogre (ogre2 has shader issues on Apple Silicon);
    # AMD/Linux requires ogre2 for gpu_lidar support.
    gz_render_engine = "ogre" if _ON_MAC else "ogre2"

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(bringup_pkg, "..") + ":"
              + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        condition=IfCondition(use_sim_time),
        launch_arguments={
            "gz_args":        [f"-r -v4 --render-engine {gz_render_engine} ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        condition=IfCondition(use_sim_time),
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name",  "sigyn",
            "-x", "11.1",
            "-y", "7.0",
            "-z", "0.0",
        ],
    )

    # Fixed delays while Gazebo's controller manager becomes ready.
    delayed_joint_broadcaster = TimerAction(
        period=3.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            condition=IfCondition(use_sim_time),
            arguments=["joint_broadcaster"],
        )],
    )
    delayed_diff_drive = TimerAction(
        period=5.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            condition=IfCondition(use_sim_time),
            arguments=["diff_cont"],
        )],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        condition=IfCondition(use_sim_time),
        arguments=["--ros-args", "-p", f"config_file:={gz_bridge_config}"],
    )
    gz_image_bridge_oakd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        condition=IfCondition(use_sim_time),
        arguments=["/oakd_top/color/image"],
    )
    gz_image_bridge_pi = Node(
        package="ros_gz_image",
        executable="image_bridge",
        condition=IfCondition(use_sim_time),
        arguments=["/gripper/camera/image"],
    )

    # -----------------------------------------------------------------------
    # Navigation (Nav2).
    # -----------------------------------------------------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            "autostart":        "True",
            "bt_xml":           bt_xml,
            "container_name":   "nav2_container",
            "map":              map_path,
            "params_file":      nav2_config,
            "slam":             "False",
            "use_composition":  "True",
            "use_localization": "True",
            "use_respawn":      "True",
            "use_sim_time":     use_sim_time,
        }.items(),
    )

    # -----------------------------------------------------------------------
    # Real-robot hardware drivers.
    # -----------------------------------------------------------------------
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        condition=UnlessCondition(use_sim_time),
        output="screen",
        parameters=[
            ekf_config,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/odometry/filtered", "odom"),
            ("/odom/unfiltered",   "/sigyn/wheel_odom"),
        ],
    )

    lidars = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "sub_launch", "lidar.launch.py")
        ),
        condition=UnlessCondition(use_sim_time),
        launch_arguments={"do_top_lidar": do_top_lidar}.items(),
    )

    # Launched with namespace='sigyn' so output resolves to /sigyn/cmd_vel,
    # matching the teensy_bridge subscription.
    twist_mux = Node(
        package="wr_twist_multiplexer",
        executable="wr_twist_multiplexer",
        name="wr_twist_multiplexer_node",
        condition=UnlessCondition(use_sim_time),
        output="screen",
        emulate_tty=True,
        respawn=True,
        arguments=["--config", twist_mux_config],
    )

    sigyn_to_teensy = OpaqueFunction(
        function=_launch_sigyn_to_teensy,
        args=[use_sim_time],
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_node",
        condition=UnlessCondition(use_sim_time),
        output="screen",
        parameters=[{
            "target_frame": "base_footprint",
            "min_height":   0.03,
            "max_height":   2.0,
            "range_min":    0.27,
            "range_max":    5.0,
            "scan_time":    0.1,
            "use_inf":      True,
        }],
        remappings=[
            ("/cloud_in", "/stereo/points"),
            ("/scan",     "/stereo/points2"),
        ],
    )

    # -----------------------------------------------------------------------
    # OAK-D camera (optional, real robot only).
    # -----------------------------------------------------------------------
    oakd_nodes = GroupAction(
        condition=UnlessCondition(use_sim_time),
        actions=[
            GroupAction(
                condition=IfCondition(do_oakd),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                bringup_pkg,
                                "launch",
                                "sub_launch",
                                "oakd_compressed_republisher.launch.py",
                            )
                        ),
                    ),
                    OpaqueFunction(
                        function=_launch_oakd_yolo26,
                        args=[bringup_pkg, use_sim_time, do_oakd, do_oakd_yolo26],
                    ),
                    Node(
                        package="can_do_challenge",
                        executable="spatial_detection_annotator.py",
                        name="oakd_spatial_annotator",
                        condition=UnlessCondition(do_oakd_yolo26),
                        output="screen",
                        parameters=[{
                            "image_topic":      "/oakd_top/oak/rgb/image_raw",
                            "detections_topic": "/oakd_top/oak/nn/spatial_detections",
                            "annotated_topic":  "/oakd/annotated_image",
                            "labels":           ["Can"],
                            "min_score":        0.3,
                        }],
                    ),
                    Node(
                        package="can_do_challenge",
                        executable="spatial_to_detection2d_converter.py",
                        name="oakd_detection_converter",
                        condition=UnlessCondition(do_oakd_yolo26),
                        output="screen",
                    ),
                ],
            ),
        ],
    )

    # -----------------------------------------------------------------------
    # Pi gripper camera (optional, real robot only).
    # -----------------------------------------------------------------------
    pi_can_detector = Node(
        package="can_do_challenge",
        executable="simple_can_detector.py",
        name="gripper_can_detector",
        condition=IfCondition(do_pi_cam),
        output="screen",
        parameters=[{
            "use_sim_time":        use_sim_time,
            "camera_name":         "gripper",
            "use_depth":           False,
            "publish_debug_image": True,
            "min_area_px2":        500,
            "min_bbox_height_px":  60,
            "max_distance_m":      0.8,
            "min_center_y_ratio":  0.1,
            "max_center_y_ratio":  0.85,
            "max_abs_x_m":         0.12,
            "log_throttle_sec":    5.0,
        }],
    )
    pi_can_detector_group = GroupAction(
        condition=UnlessCondition(use_sim_time),
        actions=[pi_can_detector],
    )

    # -----------------------------------------------------------------------
    # Optional joystick.
    # -----------------------------------------------------------------------
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sigyn_bluetooth_joystick"),
                "launch", "sigyn_bluetooth_joystick.launch.py",
            )
        ),
        condition=IfCondition(do_joystick),
    )

    # -----------------------------------------------------------------------
    # Monitoring / status overlays.
    # -----------------------------------------------------------------------
    battery_overlay = Node(
        package="sigyn_bringup",
        executable="battery_overlay_publisher.py",
        name="battery_overlay_publisher",
        output="screen",
        parameters=[{
            "battery_topic":     "/sigyn/teensy_bridge/battery/status",
            "overlay_topic":     "/battery_overlay_text",
            "min_voltage":       30.0,
            "max_voltage":       42.0,
            "filter_battery_id": "36VLIPO",
        }],
    )

    # -----------------------------------------------------------------------
    # RViz + compressed-feed republishers.
    # -----------------------------------------------------------------------
    rviz_feed_republishers = GroupAction(
        condition=UnlessCondition(use_sim_time),
        actions=[
            GroupAction(
                condition=IfCondition(do_rviz),
                actions=[
                    Node(
                        package="image_transport",
                        executable="republish",
                        name="rviz_gripper_image_republisher",
                        condition=IfCondition(use_compressed_rviz_feeds),
                        arguments=["compressed", "raw"],
                        remappings=[
                            ("in/compressed", "/gripper/camera/annotated_image/compressed"),
                            ("out",           "/gripper/camera/annotated_image_rviz"),
                        ],
                    ),
                    Node(
                        package="image_transport",
                        executable="republish",
                        name="rviz_oakd_image_republisher",
                        condition=IfCondition(use_compressed_rviz_feeds),
                        arguments=["compressed", "raw"],
                        remappings=[
                            ("in/compressed", "/oakd/annotated_image/compressed"),
                            ("out",           "/oakd/annotated_image_rviz"),
                        ],
                    ),
                ],
            )
        ],
    )

    rviz_env = {"LIBGL_ALWAYS_SOFTWARE": "1"} if _ON_MAC else {}
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(do_rviz),
        arguments=["-d", rviz_config],
        additional_env=rviz_env,
    )

    # -----------------------------------------------------------------------
    # Assemble.  Arguments must appear before any action that references them.
    # -----------------------------------------------------------------------
    return LaunchDescription([
        *declared_args,
        log_config,

        # Robot description (always).
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,

        # Simulation stack (guarded by use_sim_time).
        gz_resource_path,
        gazebo,
        spawn_entity,
        delayed_joint_broadcaster,
        delayed_diff_drive,
        gz_bridge,
        gz_image_bridge_oakd,
        gz_image_bridge_pi,

        # Navigation.
        nav2,

        # Real-robot hardware (guarded by ~use_sim_time).
        ekf_node,
        lidars,
        twist_mux,
        sigyn_to_teensy,
        pointcloud_to_laserscan,

        # Optional sensors.
        oakd_nodes,
        pi_can_detector_group,

        # Optional joystick.
        joystick,

        # Monitoring.
        battery_overlay,

        # Visualization.
        rviz_feed_republishers,
        rviz,
    ])
