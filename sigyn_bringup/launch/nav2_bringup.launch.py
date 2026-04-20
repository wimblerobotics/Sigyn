# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Wimble Robotics
# Derived from nav2_bringup (Copyright 2018 Intel Corporation, Apache-2.0)

"""nav2_bringup.launch.py — Nav2 full bringup for the Sigyn robot.

Orchestrates SLAM or AMCL localisation plus the full Nav2 navigation stack.
Supports both composed (component container) and standalone node deployment.

Launch arguments:
  autostart        (true)   Auto-transition lifecycle nodes to active.
  bt_xml           ('')     Override BT XML path; empty → nav_through_poses default.
  container_name   (nav2_container)  Component container name when use_composition=true.
  log_level        (info)   ROS 2 log level for all Nav2 nodes.
  map              ('')     Absolute path to the map YAML file.
  namespace        ('')     Top-level namespace for all Nav2 nodes.
  params_file      (nav2_bringup default)  Absolute path to navigation YAML.
  slam             (false)  Run SLAM instead of AMCL localisation.
  use_composition  (true)   Load Nav2 servers into a shared component container.
  use_localization (true)   Enable AMCL or SLAM; disable for nav-only deployments.
  use_namespace    (false)  Push the ROS namespace onto all Nav2 nodes.
  use_range_sensors (auto)  Enable VL53 range sensors: 'auto'|'true'|'false'.
  use_respawn      (false)  Respawn standalone nodes on crash.
  use_sim_time     (false)  Use /clock from Gazebo instead of wall clock.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    bringup_pkg  = get_package_share_directory("sigyn_bringup")
    nav2_pkg     = get_package_share_directory("nav2_bringup")
    nav2_launch  = os.path.join(nav2_pkg, "launch")
    default_params = os.path.join(bringup_pkg, "config", "navigation.yaml")

    # ---------------------------------------------------------------------------
    # Declare all arguments up-front so they are resolvable by all actions below.
    # ---------------------------------------------------------------------------
    declared_args = [
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically transition Nav2 lifecycle nodes to active",
        ),
        DeclareLaunchArgument(
            "bt_xml",
            default_value="",
            description=(
                "Absolute path to the BT XML to use. "
                "Empty string selects the default nav_through_poses tree."
            ),
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
            "map",
            default_value="",
            description="Absolute path to the map YAML file",
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
            "slam",
            default_value="false",
            description="Run SLAM Toolbox instead of AMCL localisation",
        ),
        DeclareLaunchArgument(
            "use_composition",
            default_value="true",
            description="Load Nav2 servers into a shared component container",
        ),
        DeclareLaunchArgument(
            "use_localization",
            default_value="true",
            description="Enable localisation (AMCL or SLAM); false for nav-only mode",
        ),
        DeclareLaunchArgument(
            "use_namespace",
            default_value="false",
            description="Push the ROS namespace onto all Nav2 nodes",
        ),
        DeclareLaunchArgument(
            "use_range_sensors",
            default_value="auto",
            description=(
                "Enable VL53 range-sensor costmap layer: "
                "'auto' (on for real robot, off in sim), 'true', or 'false'"
            ),
        ),
        DeclareLaunchArgument(
            "use_respawn",
            default_value="false",
            description="Respawn standalone Nav2 nodes on crash (composition=false only)",
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
    autostart        = LaunchConfiguration("autostart")
    bt_xml           = LaunchConfiguration("bt_xml")
    container_name   = LaunchConfiguration("container_name")
    log_level        = LaunchConfiguration("log_level")
    map_yaml_file    = LaunchConfiguration("map")
    namespace        = LaunchConfiguration("namespace")
    params_file      = LaunchConfiguration("params_file")
    slam             = LaunchConfiguration("slam")
    use_composition  = LaunchConfiguration("use_composition")
    use_localization = LaunchConfiguration("use_localization")
    use_namespace    = LaunchConfiguration("use_namespace")
    use_range_sensors = LaunchConfiguration("use_range_sensors")
    use_respawn      = LaunchConfiguration("use_respawn")
    use_sim_time     = LaunchConfiguration("use_sim_time")

    # ---------------------------------------------------------------------------
    # Parameter substitutions applied to the Nav2 YAML at launch time.
    # ---------------------------------------------------------------------------

    # When use_namespace=true, replace the '<robot_namespace>' token in multi-robot
    # param files so that topic names are correctly prefixed.
    params_file = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": ("/", namespace)},
        condition=IfCondition(use_namespace),
    )

    default_bt_xml = os.path.join(bringup_pkg, "config", "nav_through_poses.xml")

    # Resolve the BT XML path: use the caller-supplied value when non-empty,
    # otherwise fall back to the Sigyn default tree.
    bt_xml_resolved = PythonExpression(
        ["'", bt_xml, "' if '", bt_xml, "' else '", default_bt_xml, "'"]
    )

    # Range sensor layer: 'auto' → enabled on real robot, disabled in simulation.
    range_sensor_enabled = PythonExpression([
        "'True' if '",  use_range_sensors, "'.lower() == 'true' else (",
        "'False' if '", use_range_sensors, "'.lower() == 'false' else (",
        "'False' if '", use_sim_time,      "'.lower() == 'true' else 'True'))",
    ])

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                "bt_navigator.ros__parameters.default_nav_through_poses_bt_xml": bt_xml_resolved,
                "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml":       bt_xml_resolved,
                # Disable the VL53 range-sensor layer in simulation (no Teensy topics).
                "local_costmap.local_costmap.ros__parameters.range_sensor_layer.enabled":
                    range_sensor_enabled,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    # TF remappings applied to every Nav2 node so namespaced deployments work.
    tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # ---------------------------------------------------------------------------
    # Bringup group — namespace push + container + localisation + navigation.
    # ---------------------------------------------------------------------------
    bringup_group = GroupAction([
        PushROSNamespace(condition=IfCondition(use_namespace), namespace=namespace),

        # Component container — all Nav2 servers load into this process when
        # use_composition=true, dramatically reducing IPC overhead.
        Node(
            package="rclcpp_components",
            executable="component_container_isolated",
            name="nav2_container",
            condition=IfCondition(use_composition),
            output="screen",
            parameters=[configured_params, {"autostart": autostart}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=tf_remappings,
        ),

        # SLAM path: run SLAM Toolbox instead of AMCL.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch, "slam_launch.py")
            ),
            condition=IfCondition(PythonExpression([
                "'", slam, "'.lower() == 'true'",
                " and '", use_localization, "'.lower() == 'true'",
            ])),
            launch_arguments={
                "namespace":     namespace,
                "use_sim_time":  use_sim_time,
                "autostart":     autostart,
                "use_respawn":   use_respawn,
                "params_file":   params_file,
            }.items(),
        ),

        # AMCL path: standard map-based localisation.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch, "localization_launch.py")
            ),
            condition=IfCondition(PythonExpression([
                "not '", slam, "'.lower() == 'true'",
                " and '", use_localization, "'.lower() == 'true'",
            ])),
            launch_arguments={
                "namespace":        namespace,
                "map":              map_yaml_file,
                "use_sim_time":     use_sim_time,
                "autostart":        autostart,
                "params_file":      params_file,
                "use_composition":  use_composition,
                "use_respawn":      use_respawn,
                "container_name":   "nav2_container",
            }.items(),
        ),

        # Navigation servers (controller, planner, BT navigator, etc.).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_pkg, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "namespace":        namespace,
                "use_sim_time":     use_sim_time,
                "autostart":        autostart,
                "params_file":      params_file,
                "use_composition":  use_composition,
                "use_respawn":      use_respawn,
                "container_name":   "nav2_container",
            }.items(),
        ),
    ])

    return LaunchDescription([
        # Flush log lines immediately — aids debugging with journald / screen output.
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),

        *declared_args,
        bringup_group,
    ])
