# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Wimble Robotics
#
# map_cartographer.launch.py — Create a new map using Cartographer SLAM.
#
# Usage:
#   ros2 launch base map_cartographer.launch.py
#   ros2 launch base map_cartographer.launch.py use_sim_time:=true
#
# When done mapping, finish the trajectory and save the state:
#   ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory \
#       "{trajectory_id: 0}"
#   ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
#       "{filename: '/tmp/my_map.pbstream', include_unfinished_submaps: true}"
# Then save the occupancy grid:
#   ros2 run nav2_map_server map_saver_cli -f ~/ros_ws/maps/my_map

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch_ros.actions import Node


def _launch_robot_state_publisher(context, urdf_file_name_var, use_sim_time, do_top_lidar):
    description_dir = get_package_share_directory("sigyn_description")
    xacro_path = os.path.join(
        description_dir, "urdf", context.perform_substitution(urdf_file_name_var)
    )
    use_sim = use_sim_time.perform(context).lower() == "true"
    urdf_xml = xacro.process_file(
        xacro_path,
        mappings={
            "use_ros2_control": "true" if use_sim else "false",
            "sim_mode": use_sim_time.perform(context),
            "do_top_lidar": do_top_lidar.perform(context),
        },
    ).toxml()
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": urdf_xml,
                "use_sim_time": use_sim_time,
            }],
        )
    ]


def generate_launch_description():
    ld = LaunchDescription()
    base_pkg = get_package_share_directory("base")
    rviz_config_path = os.path.join(
        get_package_share_directory("rviz"), "config", "config.rviz"
    )

    ld.add_action(DeclareLaunchArgument(
        "do_joystick",
        default_value="false",
        description="Launch bluetooth joystick if true.",
    ))
    do_joystick = LaunchConfiguration("do_joystick")

    cartographer_config_dir = LaunchConfiguration("cartographer_config_dir")
    cartographer_config_basename = LaunchConfiguration("cartographer_config_basename")
    do_top_lidar = LaunchConfiguration("do_top_lidar")
    do_rviz = LaunchConfiguration("do_rviz")
    resolution = LaunchConfiguration("resolution")
    urdf_file_name = LaunchConfiguration("urdf_file_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    ld.add_action(DeclareLaunchArgument(
        "cartographer_config_dir",
        default_value=os.path.join(base_pkg, "config"),
        description="Directory containing the Cartographer .lua config file.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "cartographer_config_basename",
        default_value="cartographer.lua",
        description="Cartographer configuration .lua file name.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "do_top_lidar",
        default_value="true",
        description="Use top LiDAR; if false, use only the cup LiDAR.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "do_rviz",
        default_value="true",
        description="Launch RViz if true.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "resolution",
        default_value="0.0508",
        description="Occupancy grid resolution in metres per pixel (5.08 cm).",
    ))
    ld.add_action(DeclareLaunchArgument(
        "urdf_file_name",
        default_value="sigyn.urdf.xacro",
        description="URDF/xacro file name inside sigyn_description/urdf/.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true.",
    ))

    ld.add_action(OpaqueFunction(
        function=_launch_robot_state_publisher,
        args=[LaunchConfiguration("urdf_file_name"), use_sim_time, do_top_lidar],
    ))

    ld.add_action(Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(use_sim_time),
    ))

    # Launch the top lidar directly — bypass the scan_to_scan_filter_chain which
    # is a no-op for the top lidar (full 360° view) and was the source of
    # duplicate-timestamp scan messages seen by Cartographer.  Cartographer will
    # subscribe to /raw_scan directly via the remapping below.
    ld.add_action(Node(
        condition=UnlessCondition(use_sim_time),
        package='wr_ldlidar',
        executable='wr_ldlidar',
        name='top_ldlidar',
        output='screen',
        parameters=[
            {'serial_port': '/dev/lidar_top'},
            {'topic_name': 'scan'},
            {'lidar_frame': 'lidar_frame_top_lidar'},
            {'range_threshold': 0.0},
        ],
        remappings=[('scan', 'raw_scan')],
    ))

    # EKF — fuses wheel odometry (/sigyn/wheel_odom) and IMU to produce /odom.
    # Cartographer consumes /odom for dead-reckoning between scan matches
    # (use_odometry = true in cartographer.lua).  Without this, the pose
    # extrapolator has nothing to work with and the map smears as the robot moves.
    ekf_config_path = os.path.join(
        get_package_share_directory("base"), "config/ekf.yaml"
    )
    ld.add_action(launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        condition=UnlessCondition(use_sim_time),
        name="ekf_filter_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            ekf_config_path,
        ],
        remappings=[
            ("/odometry/filtered", "odom"),
            ("/odom/unfiltered", "/sigyn/wheel_odom"),
        ],
    ))

    # Teensy bridge — provides wheel odometry and accepts cmd_vel so the robot
    # can be driven around while the map is being built.
    def _launch_teensy_bridge(context, *args, **kwargs):
        if use_sim_time.perform(context).lower() == 'true':
            return []
        pkg = get_package_share_directory('sigyn_to_teensy')
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'sigyn_to_teensy.launch.py')
            ),
            launch_arguments={'namespace': 'sigyn'}.items(),
        )]
    ld.add_action(OpaqueFunction(function=_launch_teensy_bridge))

    # Twist multiplexer — merges keyboard / joystick / autonomy cmd_vel streams.
    multiplexer_config = os.path.join(
        get_package_share_directory('wr_twist_multiplexer'),
        'config',
        'wr_twist_multiplexer.yaml',
    )
    ld.add_action(Node(
        package='wr_twist_multiplexer',
        executable='wr_twist_multiplexer',
        name='wr_twist_multiplexer_node',
        arguments=['--config', multiplexer_config],
        emulate_tty=True,
        respawn=True,
        output='screen',
        condition=UnlessCondition(use_sim_time),
    ))

    # Optional bluetooth joystick for driving during mapping.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sigyn_bluetooth_joystick'),
                'launch',
                'sigyn_bluetooth_joystick.launch.py',
            )
        ),
        condition=IfCondition(do_joystick),
    ))

    ld.add_action(Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        # Subscribe to raw_scan directly — no filter chain in the middle.
        remappings=[("scan", "raw_scan")],
        arguments=[
            "-configuration_directory", cartographer_config_dir,
            "-configuration_basename", cartographer_config_basename,
        ],
    ))

    ld.add_action(Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-resolution", resolution],
    ))

    ld.add_action(Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(do_rviz),
        arguments=["-d", rviz_config_path],
    ))

    return ld
