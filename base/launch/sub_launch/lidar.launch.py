import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    nodes = [
        DeclareLaunchArgument(
            name='range_threshold', 
            default_value='0.0',
            description='Range Threshold'
        ),
    
        Node(
            condition=IfCondition(str(os.path.exists("/dev/lidar_top"))),
            package='ldlidar',
            executable='ldlidar',
            name='top_ldlidar',
            output='screen',
            parameters=[
                {'serial_port': "/dev/lidar_top"},
                {'topic_name': "scan"},
                {'lidar_frame': "lidar_frame_top_lidar"},
                {'range_threshold': LaunchConfiguration("range_threshold")}
            ],
            remappings=[('scan', 'raw_scan')]
        ),
        
        Node(
            condition=IfCondition(str(os.path.exists("/dev/lidar_cup"))),
            package='ldlidar',
            executable='ldlidar',
            name='cup_ldlidar',
            output='screen',
            parameters=[
                {'serial_port': "/dev/lidar_cup"},
                {'topic_name': "scan_cup"},
                {'lidar_frame': "lidar_frame_cup_lidar"},
                {'range_threshold': LaunchConfiguration("range_threshold")}
                ],
            remappings=[('scan', 'scan_cup')]
        ),
        
        Node(
          package="laser_filters",
          executable="scan_to_scan_filter_chain",
          name="scan_to_scan_filter_chain",
          output="screen",
          parameters=[
            PathJoinSubstitution([
                get_package_share_directory("base"),
                "config", "laser_filters_angular.yaml",
            ])],
            remappings=[('scan', 'raw_scan'),
                        ('scan_filtered', 'scan')]


        )
    ]
    
    return LaunchDescription(nodes)
