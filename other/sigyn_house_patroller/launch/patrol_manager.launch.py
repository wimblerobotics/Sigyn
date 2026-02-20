import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('sigyn_house_patroller')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for patrol manager'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sigyn_house_patroller'),
            'config',
            'patrol_config.yaml'
        ]),
        description='Path to patrol configuration file'
    )
    
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sigyn_house_patroller'),
            'config',
            'waypoints.yaml'
        ]),
        description='Path to waypoint configuration file'
    )
    
    room_file_arg = DeclareLaunchArgument(
        'room_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sigyn_house_patroller'),
            'config',
            'rooms.yaml'
        ]),
        description='Path to room configuration file'
    )
    
    # Main patrol manager node
    patrol_manager_node = Node(
        package='sigyn_house_patroller',
        executable='sigyn_house_patroller_node',
        name='patrol_manager',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'patrol_config_file': LaunchConfiguration('config_file')},
            {'waypoint_config_file': LaunchConfiguration('waypoint_file')},
            {'room_config_file': LaunchConfiguration('room_file')},
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            # Remap topics to match your robot's topics
            ('/battery_state', '/battery_state'),
            ('/temperature', '/temperature'),
            ('/scan', '/scan'),
            ('/camera/depth/points', '/camera/depth/points'),
            ('/camera/color/image_raw', '/camera/color/image_raw'),
        ]
    )
    
    # Feature recognizer nodes (optional, can be launched separately)
    battery_monitor_node = Node(
        package='sigyn_house_patroller',
        executable='battery_monitor_node',
        name='battery_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=lambda context: 'true'  # Always launch for now
    )
    
    temperature_monitor_node = Node(
        package='sigyn_house_patroller',
        executable='temperature_monitor_node',
        name='temperature_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=lambda context: 'true'
    )
    
    door_monitor_node = Node(
        package='sigyn_house_patroller',
        executable='door_monitor_node',
        name='door_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=lambda context: 'true'
    )
    
    change_detector_node = Node(
        package='sigyn_house_patroller',
        executable='change_detector_node',
        name='change_detector',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=lambda context: 'true'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        log_level_arg,
        config_file_arg,
        waypoint_file_arg,
        room_file_arg,
        patrol_manager_node,
        battery_monitor_node,
        temperature_monitor_node,
        door_monitor_node,
        change_detector_node,
    ])
