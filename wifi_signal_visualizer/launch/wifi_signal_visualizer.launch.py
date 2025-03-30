from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    enable_interpolation = LaunchConfiguration('enable_interpolation')
    ld.add_action(
        DeclareLaunchArgument(
            'enable_interpolation',
            default_value='true',
            description='Enable or disable interpolation'
        )
    )

    generate_new_data = LaunchConfiguration('generate_new_data')

    ld.add_action(
        DeclareLaunchArgument(
            'generate_new_data',
            default_value='false',
            description='Generate new data or use existing data'
        )
    )

    max_interpolation_distance = LaunchConfiguration('max_interpolation_distance')
    ld.add_action(
        DeclareLaunchArgument(
            'max_interpolation_distance',
            default_value='2.0',
            description='Maximum distance for interpolation'
        )
    )

    wifi_node = Node(
            package='wifi_signal_visualizer',  # Replace with the actual package name
            executable='wifi_signal_visualizer_node',  # Replace with the actual node name
            name='wifi_signal_visualizer',
            output='screen',
            parameters=[
                {'max_interpolation_distance': max_interpolation_distance},
                {'enable_interpolation': enable_interpolation},
                {'generate_new_data': generate_new_data}
            ]
        )
    ld.add_action(wifi_node)
    
    return ld