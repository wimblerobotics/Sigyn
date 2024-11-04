import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
import launch_ros.actions
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    configFilePath = os.path.join(
        get_package_share_directory('experiments'),
        'config',
        'motor_characterization.yaml'
    )

    ld = LaunchDescription()

    node = Node(
        emulate_tty=True,
        executable='motor_characterization',
        package='experiments',
        arguments=['--config', configFilePath],
        #prefix=['xterm -e gdb -ex run --args'],
        respawn=False,
        output='screen')
    ld.add_action(node)
    return ld
