from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sigyn_to_teensy',
            executable='teensy_bridge',
            name='teensy_bridge',
            output='screen'
        )
    ])