#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory('perimeter_roamer_v3')

    # Launch arguments
    waypoint_database_arg = DeclareLaunchArgument(
        'waypoint_database_path',
        default_value='data/patrol_waypoints.db',
        description='Path to the waypoints SQLite database file (relative to ~/sigyn_ws/src/Sigyn/)'
    )
    
    loop_waypoints_arg = DeclareLaunchArgument(
        'loop_waypoints', 
        default_value='false',
        description='Whether to continuously loop through waypoints after completing all'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'perimeter_roamer_params.yaml'),
        description='Full path to the configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    start_waypoint_capture_arg = DeclareLaunchArgument(
        'start_waypoint_capture',
        default_value='false',
        description='Start waypoint capture node to record waypoints'
    )

    # Construct behavior tree file path
    bt_xml_file = PathJoinSubstitution([
        FindPackageShare('perimeter_roamer_v3'),
        'bt_xml',
        'follow_waypoints.xml'
    ])

    # Battery simulator node (conditional - starts when use_sim_time is true)
    battery_simulator = Node(
        package='perimeter_roamer_v3',
        executable='battery_simulator.py',
        name='battery_simulator',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )

    # Waypoint capture node (conditional)
    waypoint_capture = Node(
        package='perimeter_roamer_v3',
        executable='capture_waypoints.py',
        name='waypoint_capture',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('start_waypoint_capture'))
    )

    # Main perimeter roamer node with waypoint following
    perimeter_roamer_node = Node(
        package='perimeter_roamer_v3',
        executable='perimeter_roamer',
        name='perimeter_roamer_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'bt_xml_filename': bt_xml_file,
                'waypoint_database_path': LaunchConfiguration('waypoint_database_path'),
                'loop_waypoints': LaunchConfiguration('loop_waypoints'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[
            # These should match your simulation topics
            ('/scan', '/scan'),
            ('/battery_state', '/sigyn/teensy_bridge/battery/status'),
            ('/navigate_to_pose', '/navigate_to_pose'),
            ('/odom', '/odom'),
            ('/cmd_vel', '/cmd_vel_nav'),
        ]
    )

    return LaunchDescription([
        waypoint_database_arg,
        loop_waypoints_arg,
        config_file_arg,
        use_sim_time_arg,
        start_waypoint_capture_arg,
        battery_simulator,
        waypoint_capture,
        perimeter_roamer_node
    ])
