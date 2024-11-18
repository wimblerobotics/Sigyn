from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='serial_port', 
            default_value='/dev/lidar_front_center',
            description='LD06 Serial Port'
        ),
        DeclareLaunchArgument(
            name='range_threshold', 
            default_value='0.0',
            description='Range Threshold'
        ),
        Node(
            package='ldlidar',
            executable='ldlidar',
            name='ldlidar',
            output='screen',
            #prefix=['xterm -e gdb -ex run --args'],
            parameters=[
                {'serial_port': LaunchConfiguration("serial_port")},
                {'topic_name': "scan"},
                {'lidar_frame': "lidar_frame_top_lidar"},
                {'range_threshold': LaunchConfiguration("range_threshold")}
            ],
            remappings=[('scan', 'raw_scan')]
        ),
        # See: http://wiki.ros.org/laser_filters
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

    ])
