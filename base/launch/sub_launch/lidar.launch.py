from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
                {'lidar_frame': "scan"},
                {'range_threshold': LaunchConfiguration("range_threshold")}
            ]
        ),

    ])
