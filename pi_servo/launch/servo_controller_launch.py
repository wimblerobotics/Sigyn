import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pi_servo'),
        'config',
        'ros2_control_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config],
            output='screen'
        ),
        Node(
            package='pi_servo',
            executable='servo_controller',
            name='servo_controller',
            output='screen'
        )
    ])
