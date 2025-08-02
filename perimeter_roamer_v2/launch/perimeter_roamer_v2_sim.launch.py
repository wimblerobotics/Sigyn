import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('perimeter_roamer_v2')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Path to the parameter file
    param_file = os.path.join(pkg_dir, 'config', 'perimeter_roamer_v2_sim_params.yaml')
    
    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='perimeter_roamer_v2',
            executable='perimeter_roamer',
            name='perimeter_roamer_v2',
            output='screen',
            parameters=[
                param_file,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('~/cmd_vel', '/cmd_vel'),
                ('~/scan', '/scan'),
                ('~/odom', '/odom'),
            ]
        )
    ]) 