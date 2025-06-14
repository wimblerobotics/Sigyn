from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sigyn_to_sensor',
            executable='sigyn_to_sensor',
            name='sigyn_to_sensor',
            output='screen'
        )
    ])