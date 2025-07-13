import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('wall_finder'),
        'config',
        'wall_finder.yaml'
    )

    ld = LaunchDescription()

    wall_finder_node = Node(
        package='wall_finder',
        executable='wall_finder',
        name='wall_finder',
        arguments=['--config', config_file_path],
        output='screen',
        emulate_tty=True,
        respawn=False
    )
    
    ld.add_action(wall_finder_node)
    return ld
