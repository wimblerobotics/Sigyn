from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='serial_port', 
            default_value='/dev/ttyUSB0',
            description='LD06 Serial Port'
        ),
        DeclareLaunchArgument(
            name='topic_name', 
            default_value='scan_right_rear',
            description='LD06 Topic Name'
        ),
        DeclareLaunchArgument(
            name='lidar_frame', 
            default_value='laser',
            description='Lidar Frame ID'
        ),
        DeclareLaunchArgument(
            name='range_threshold', 
            default_value='0.2',
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
                {'topic_name': LaunchConfiguration("topic_name")},
                {'lidar_frame': LaunchConfiguration("lidar_frame")},
                {'range_threshold': LaunchConfiguration("range_threshold")}
            ]
        )
    ])
