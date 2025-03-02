# Options:
# do_joint_state_gui (false) - Flag to enable joint_state_publisher_gui.
# do_rviz (true) - Launch RViz if true.

import os
import platform

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    description_pkg = get_package_share_directory("description")

    rviz_directory_path = get_package_share_directory("rviz")
    rviz_config_path = os.path.join(rviz_directory_path, "config", "config.rviz")

    do_joint_state_gui = LaunchConfiguration("do_joint_state_gui")
    ld.add_action(
        DeclareLaunchArgument(
            name="do_joint_state_gui",
            default_value="False",
            description="Flag to enable joint_state_publisher_gui",
        )
    )

    do_rviz = LaunchConfiguration("do_rviz")
    ld.add_action(
        DeclareLaunchArgument(
            name="do_rviz", default_value="true", description="Launch RViz if true"
        )
    )

    log_info_action = LogInfo(
        msg=[
            "do_joint_state_gui: [",
            do_joint_state_gui,
            "], do_rviz: [",
            do_rviz,
            "]",
        ]
    )
    ld.add_action(log_info_action)
    
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(description_pkg, "launch", "description.launch.py")]
        ),
    ))

    # Launch the joint state publisher GUI
    ld.add_action(
        Node(
            condition=IfCondition(do_joint_state_gui),
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
        )
    )

    controller_params_file = PathJoinSubstitution(
        [description_pkg, "config", "my_controllers.yaml"])
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file],
        output="both",
        remappings=[("~/robot_description", "/robot_description")],
    )
    ld.add_action(controller_manager)

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    ld.add_action(joint_broad_spawner)
    
    fwcommand_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", controller_params_file],
    )
    ld.add_action(fwcommand_spawner)

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(LaunchConfiguration("do_rviz")),
        arguments=["-d", rviz_config_path],
    )
    ld.add_action(rviz_node)

    return ld
