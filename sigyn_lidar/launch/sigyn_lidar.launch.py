from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(get_package_share_directory('sigyn_lidar'), 'config', 'example.yaml')
    return LaunchDescription([
        Node(
            package='sigyn_lidar',
            executable='modular_sigyn_lidar_node',
            name='modular_sigyn_lidar',
            parameters=[config_file],
            remappings=[
                ('scan_fused', '/scan')
            ],
            output='screen'
        )
    ])
