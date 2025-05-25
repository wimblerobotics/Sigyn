import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node

def generate_launch_description():
    configFilePath = os.path.join(
        get_package_share_directory('bluetooth_joystick'),
        'config',
        'bluetooth_joystick.yaml'
    )
    with open(configFilePath, 'r') as file:
        configParams = yaml.safe_load(file)['bluetooth_joystick']['ros__parameters']

    ld = LaunchDescription()

    bluetooth_joystick_node = Node(
        emulate_tty=True,
        executable='bluetooth_joystick',
        name='bluetooth_joystick_node',
        package='bluetooth_joystick',
        parameters=[configParams],
        #prefix=['xterm -e gdb -ex run --args'],
        output='screen')
    ld.add_action(bluetooth_joystick_node)
    return ld
