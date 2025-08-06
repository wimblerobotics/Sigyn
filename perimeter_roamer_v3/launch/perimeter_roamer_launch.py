# Copyright 2025 Wimble Robotics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory('perimeter_roamer_v3')

    # Launch arguments
    bt_xml_filename_arg = DeclareLaunchArgument(
        'bt_xml_filename',
        default_value='simple_test.xml',
        description='Behavior tree XML filename (without path)')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'perimeter_roamer_params.yaml'),
        description='Full path to the configuration file')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Construct behavior tree file path from parameter
    bt_xml_file = PathJoinSubstitution([
        FindPackageShare('perimeter_roamer_v3'),
        'bt_xml',
        LaunchConfiguration('bt_xml_filename')
    ])

    # Node configuration
    perimeter_roamer_node = Node(
        package='perimeter_roamer_v3',
        executable='perimeter_roamer',
        name='perimeter_roamer_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'bt_xml_filename': bt_xml_file,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[
            # Remap to actual topics on your robot
            ('/scan', '/scan'),
            ('/battery_state', '/battery_state'),
            ('/navigate_to_pose', '/navigate_to_pose'),
        ]
    )

    # Battery simulator for simulation only
    battery_simulator_node = Node(
        package='perimeter_roamer_v3',
        executable='battery_simulator.py',
        name='battery_simulator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )

    # Delay perimeter_roamer start to ensure Nav2 action server is available
    delayed_roamer = TimerAction(
        period=5.0,
        actions=[perimeter_roamer_node]
    )
    return LaunchDescription([
        bt_xml_filename_arg,
        config_file_arg,
        use_sim_time_arg,
        battery_simulator_node,
        delayed_roamer
    ])