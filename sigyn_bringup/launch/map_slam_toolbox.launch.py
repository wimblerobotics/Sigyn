# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Wimble Robotics
#
# map_slam_toolbox.launch.py — Create a new map using SLAM Toolbox (online async).
#
# Usage:
#   ros2 launch sigyn_bringup map_slam_toolbox.launch.py
#   ros2 launch sigyn_bringup map_slam_toolbox.launch.py use_sim_time:=true
#   ros2 launch sigyn_bringup map_slam_toolbox.launch.py slam_params_file:=/path/to/params.yaml
#
# When done mapping, save the map using the map_saver_server launched below:
#   ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
#       "{map_url: '$HOME/maps/my_map', image_format: 'pgm', map_mode: 'trinary',
#         free_thresh: 0.25, occupied_thresh: 0.65}"
#
# SLAM Toolbox can also serialize its internal pose graph:
#   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
#       "{name: {data: '/tmp/my_map'}}"

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    base_pkg = get_package_share_directory("sigyn_bringup")
    rviz_config_path = os.path.join(
        get_package_share_directory("rviz"), "config", "config.rviz"
    )

    do_joystick      = LaunchConfiguration("do_joystick")
    do_top_lidar     = LaunchConfiguration("do_top_lidar")
    do_rviz          = LaunchConfiguration("do_rviz")
    slam_params_file = LaunchConfiguration("slam_params_file")
    urdf_file_name   = LaunchConfiguration("urdf_file_name")
    use_sim_time     = LaunchConfiguration("use_sim_time")

    ld.add_action(DeclareLaunchArgument(
        "do_joystick",
        default_value="false",
        description="Launch bluetooth joystick if true.",
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
        "slam_params_file",
        default_value=os.path.join(base_pkg, "config", "mapper_params_online_async.yaml"),
        description="Full path to the SLAM Toolbox parameters file.",
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

    # Top lidar feeds /scan directly — no filter chain in the path so there
    # is no risk of the filter re-stamping and creating duplicate header times.
    # SLAM Toolbox subscribes to /scan per mapper_params_online_async.yaml.
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
        # No remapping: driver publishes directly to /scan.
    ))

    # Teensy bridge — must start before the EKF so that /sigyn/wheel_odom
    # is available when the EKF initialises.
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

    # EKF: SLAM Toolbox needs the odom frame.  The Teensy bridge above
    # provides /sigyn/wheel_odom which the EKF fuses into /odom.
    ld.add_action(Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        condition=UnlessCondition(use_sim_time),
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            os.path.join(base_pkg, "config", "ekf.yaml"),
        ],
        remappings=[
            ("/odometry/filtered", "odom"),
            ("/odom/unfiltered", "/sigyn/wheel_odom"),
        ],
    ))

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

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            )
        ]),
        launch_arguments={
            "use_lifecycle_manager": "False",
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
    ))

    ld.add_action(Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(do_rviz),
        arguments=["-d", rviz_config_path],
    ))

    # Map saver — provides /map_saver/save_map service so you can save the
    # current occupancy grid on demand without stopping the mapping session.
    ld.add_action(Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'save_map_timeout': 5.0},
            {'free_thresh_default': 0.25},
            {'occupied_thresh_default': 0.65},
        ],
    ))
    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_saver',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_saver']},
        ],
    ))

    return ld
