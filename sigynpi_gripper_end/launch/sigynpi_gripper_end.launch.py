import os
import xacro

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node


def generate_launch_description():
    package_name='sigynpi_gripper_end'
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         # {'robot_description': robot_description},
    #                 controller_params_file]
    # )
    # ld.add_action(controller_manager)

    # delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # # diff_drive_spawner = Node(
    # #     package="controller_manager",
    # #     executable="spawner",
    # #     arguments=["diff_cont"],
    # # )

    # # delayed_diff_drive_spawner = RegisterEventHandler(
    # #     event_handler=OnProcessStart(
    # #         target_action=controller_manager,
    # #         on_start=[diff_drive_spawner],
    # #     )
    # # )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_broad"],
    # )
    # ld.add_action(joint_broad_spawner)

    # delayed_joint_broad_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager,
    #         on_start=[joint_broad_spawner],
    #     )
    # )
    # ld.add_action(delayed_controller_manager)
    
    return ld