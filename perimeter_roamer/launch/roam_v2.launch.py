import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('perimeter_roamer')
    
    # Path to the parameter file
    param_file = os.path.join(pkg_dir, 'config', 'roamer_v2_params.yaml')
    
    return LaunchDescription([
        Node(
            package='perimeter_roamer',
            executable='roaming_node_v2',
            name='perimeter_roamer_v2',
            output='screen',
            parameters=[param_file],
            remappings=[
                ('~/cmd_vel', '/cmd_vel'),
                ('~/scan', '/scan'),
                ('~/odom', '/odom'),
            ]
        )
    ])
