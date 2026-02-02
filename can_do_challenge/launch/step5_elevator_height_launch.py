import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    can_do_pkg = get_package_share_directory('can_do_challenge')

    # Behavior Tree Node
    can_do_node = Node(
        package='can_do_challenge',
        executable='can_do_challenge_node_real',
        name='can_do_challenge_node',
        output='screen',
        parameters=[{
            "use_sim_time": False,
            "enable_groot_monitoring": True,
            "groot_port": 1667,
            "bt_xml_filename": os.path.join(can_do_pkg, "bt_xml", "step5_elevator_height.xml"),
            # Parameter for the target Y pixel coordinate (user requested roughly 342)
            "pi_target_y": 342.0,
            "pi_target_tolerance": 20.0
        }],
    )
    
    return LaunchDescription([
        can_do_node,
    ])
