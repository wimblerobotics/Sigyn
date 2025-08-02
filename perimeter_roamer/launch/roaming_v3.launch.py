from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Create the roaming node
    roaming_node = Node(
        package='perimeter_roamer',
        executable='roaming_node_v3',
        name='roaming_node_v3',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        roaming_node
    ])
