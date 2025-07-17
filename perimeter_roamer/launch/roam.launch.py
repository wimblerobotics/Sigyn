import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'perimeter_roamer'
    share_dir = get_package_share_directory(pkg_name)

    # Configuration file
    params_file = os.path.join(share_dir, 'config', 'roamer_params.yaml')

    # Node configuration
    roamer_node = Node(
        package=pkg_name,
        executable='roamer',
        name='perimeter_roamer', # Match the name in the Python script
        output='screen',
        parameters=[params_file],
        # Optional: Remap topics if needed
        remappings=[
        #     ('/scan', '/robot/scan'),
        #     ('/odom', '/robot/odom'),
            ('/cmd_vel', '/cmd_vel_nav'),
        #     ('/local_costmap/costmap', '/costmap_node/costmap')
        ]
    )

    return LaunchDescription([
        roamer_node
    ])