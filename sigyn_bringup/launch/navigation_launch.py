# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Wimble Robotics
# Derived from nav2_bringup (Copyright 2018 Intel Corporation, Apache-2.0)

"""navigation_launch.py — Nav2 navigation-server bringup.

Launches the controller, planner, behaviour, BT navigator, waypoint follower,
and velocity smoother — either as standalone nodes or loaded into a shared
component container when use_composition=true.

This file is included by nav2_bringup.launch.py; it is not typically invoked
directly.

Launch arguments:
  autostart       (true)          Auto-transition lifecycle nodes to active.
  container_name  (nav2_container) Component container to load into.
  log_level       (info)          ROS 2 log level.
  namespace       ('')            ROS namespace prefix.
  params_file     (nav2 default)  Absolute path to navigation YAML.
  use_composition (true)          Use component container; false = standalone nodes.
  use_respawn     (false)         Respawn standalone nodes on crash.
  use_sim_time    (false)         Use /clock from Gazebo.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_pkg  = get_package_share_directory("sigyn_bringup")
    default_params = os.path.join(bringup_pkg, "config", "navigation.yaml")

    # ---------------------------------------------------------------------------
    # Declare all arguments up-front.
    # ---------------------------------------------------------------------------
    declared_args = [
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically transition Nav2 lifecycle nodes to active",
        ),
        DeclareLaunchArgument(
            "container_name",
            default_value="nav2_container",
            description="Name of the component container when use_composition=true",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS 2 log level for all Nav2 nodes",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Top-level ROS namespace for Nav2 nodes",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Absolute path to the Nav2 parameters YAML file",
        ),
        DeclareLaunchArgument(
            "use_composition",
            default_value="true",
            description="Load Nav2 servers into a shared component container",
        ),
        DeclareLaunchArgument(
            "use_respawn",
            default_value="false",
            description="Respawn standalone nodes on crash (use_composition=false only)",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock from /clock topic instead of wall clock",
        ),
    ]

    # ---------------------------------------------------------------------------
    # LaunchConfiguration handles.
    # ---------------------------------------------------------------------------
    autostart       = LaunchConfiguration("autostart")
    container_name  = LaunchConfiguration("container_name")
    log_level       = LaunchConfiguration("log_level")
    namespace       = LaunchConfiguration("namespace")
    params_file     = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn     = LaunchConfiguration("use_respawn")
    use_sim_time    = LaunchConfiguration("use_sim_time")

    # Full container name: prepend namespace when non-empty.
    container_name_full = (namespace, "/", container_name)

    # ---------------------------------------------------------------------------
    # Parameter file with runtime substitutions.
    # ---------------------------------------------------------------------------

    # Disable realtime scheduling in simulation — the process won't have
    # the necessary RT permissions and would otherwise emit a warning on start.
    realtime_priority = PythonExpression(
        ["'False' if '", use_sim_time, "'.lower() == 'true' else 'True'"]
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                "use_sim_time":  use_sim_time,
                "autostart":     autostart,
                # Disable SCHED_FIFO in simulation (no RT kernel permissions needed).
                "velocity_smoother.ros__parameters.use_realtime_priority": realtime_priority,
            },
            # Do not convert types — preserves nav2 plugin name strings as-is.
            convert_types=False,
        ),
    )

    # TF remappings applied to every Nav2 node so namespaced deployments work.
    tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Nav2 lifecycle nodes managed by the lifecycle manager.
    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "bt_navigator",
        "waypoint_follower",
        # "collision_monitor",  # Uncomment to enable collision monitoring.
        "docking_server",     # Enabled for AprilTag-based charging dock
    ]

    # ---------------------------------------------------------------------------
    # Standalone (non-composed) nodes.
    # Launched when use_composition=false.
    # ---------------------------------------------------------------------------
    standalone_nodes = GroupAction(
        condition=IfCondition(PythonExpression(
            ["not '", use_composition, "'.lower() == 'true'"]
        )),
        actions=[
            # SetParameter broadcasts use_sim_time to every node in this group,
            # preventing clock-mismatch TF errors if the YAML omits the key.
            SetParameter("use_sim_time", use_sim_time),

            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=tf_remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=tf_remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=tf_remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=tf_remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=tf_remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=tf_remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=tf_remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="opennav_docking",
                executable="opennav_docking",
                name="docking_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=tf_remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[{"autostart": autostart, "node_names": lifecycle_nodes}],
            ),
        ],
    )

    # ---------------------------------------------------------------------------
    # Composable nodes.
    # Loaded into the shared container when use_composition=true.
    # ---------------------------------------------------------------------------
    composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package="nav2_controller",
                        plugin="nav2_controller::ControllerServer",
                        name="controller_server",
                        parameters=[configured_params],
                        remappings=tf_remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    ComposableNode(
                        package="nav2_smoother",
                        plugin="nav2_smoother::SmootherServer",
                        name="smoother_server",
                        parameters=[configured_params],
                        remappings=tf_remappings,
                    ),
                    ComposableNode(
                        package="nav2_planner",
                        plugin="nav2_planner::PlannerServer",
                        name="planner_server",
                        parameters=[configured_params],
                        remappings=tf_remappings,
                    ),
                    ComposableNode(
                        package="nav2_behaviors",
                        plugin="behavior_server::BehaviorServer",
                        name="behavior_server",
                        parameters=[configured_params],
                        remappings=tf_remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    ComposableNode(
                        package="nav2_bt_navigator",
                        plugin="nav2_bt_navigator::BtNavigator",
                        name="bt_navigator",
                        parameters=[configured_params],
                        remappings=tf_remappings,
                    ),
                    ComposableNode(
                        package="nav2_waypoint_follower",
                        plugin="nav2_waypoint_follower::WaypointFollower",
                        name="waypoint_follower",
                        parameters=[configured_params],
                        remappings=tf_remappings,
                    ),
                    ComposableNode(
                        package="nav2_velocity_smoother",
                        plugin="nav2_velocity_smoother::VelocitySmoother",
                        name="velocity_smoother",
                        parameters=[configured_params],
                        remappings=tf_remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    # ComposableNode(
                    #     package="nav2_collision_monitor",
                    #     plugin="nav2_collision_monitor::CollisionMonitor",
                    #     name="collision_monitor",
                    #     parameters=[configured_params],
                    #     remappings=tf_remappings,
                    # ),
                    ComposableNode(
                        package="opennav_docking",
                        plugin="opennav_docking::DockingServer",
                        name="docking_server",
                        parameters=[configured_params],
                        remappings=tf_remappings,
                    ),
                    ComposableNode(
                        package="nav2_lifecycle_manager",
                        plugin="nav2_lifecycle_manager::LifecycleManager",
                        name="lifecycle_manager_navigation",
                        parameters=[{
                            "autostart":   autostart,
                            "node_names":  lifecycle_nodes,
                        }],
                    ),
                ],
            ),
        ],
    )

    return LaunchDescription([
        # Flush log lines immediately — aids debugging with journald / screen output.
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),

        *declared_args,
        standalone_nodes,
        composable_nodes,
    ])
