# USB Bandwidth for interesting devices

<div style="border: 3px solid #2c3e50; border-radius: 8px; padding: 15px; margin: 10px 0; background-color: #f8f9fa; box-shadow: 0 2px 4px rgba(0,0,0,0.1);">

<table style="width: 100%; border-collapse: collapse; color: #000000; font-weight: 500;">
<thead>
<tr style="border-bottom: 3px solid #2c3e50;">
<th style="border: 1px solid #ccc; padding: 12px; text-align: left; background-color: #e9ecef; font-weight: 600;">Device</th>
<th style="border: 1px solid #ccc; padding: 12px; text-align: left; background-color: #e9ecef; font-weight: 600;">To Device</th>
<th style="border: 1px solid #ccc; padding: 12px; text-align: left; background-color: #e9ecef; font-weight: 600;">From Device</th>
</tr>
</thead>
<tbody>
<tr>
<td style="border: 1px solid #ccc; padding: 10px;">board1</td>
<td style="border: 1px solid #ccc; padding: 10px;">1.88 kb/s</td>
<td style="border: 1px solid #ccc; padding: 10px;">6.12 kb/s</td>
</tr>
<tr>
<td style="border: 1px solid #ccc; padding: 10px;">board2</td>
<td style="border: 1px solid #ccc; padding: 10px;">0.90 kb/s</td>
<td style="border: 1px solid #ccc; padding: 10px;">3.32 kb/s</td>
</tr>
<tr>
<td style="border: 1px solid #ccc; padding: 10px;">LIDAR</td>
<td style="border: 1px solid #ccc; padding: 10px;">4.52 kb/s</td>
<td style="border: 1px solid #ccc; padding: 10px;">22.20 kb/s</td>
</tr>
</tbody>
</table>

</div>

# ROS2 Topic Analysis Report

📊 **Total Topics:** 100
🟢 **Active Topics:** 35
📡 **Total Bandwidth:** 2.1 MB/s

## Topic Details

| Topic | Type | Rate (Hz) | Bandwidth | QoS First Pub | Publishers | Subscribers |
|-------|------|-----------|-----------|---------------|------------|-------------|
| `/amcl/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | amcl | — |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | amcl | — |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | N/A | N/A | RELIABLE / VOLATILE | — | perimeter_roamer_node |
| `/behavior_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | behavior_server | — |
| `/behavior_tree_log` | `nav2_msgs/msg/BehaviorTreeLog` | N/A | N/A | RELIABLE / VOLATILE | bt_navigator_navigate_to_pose_rclcpp_node<br>bt_navigator_navigate_through_poses_rclcpp_node | — |
| `/bluetoothJoystick` | `msgs/msg/BluetoothJoystick` | N/A | N/A | RELIABLE / VOLATILE | — | twist_multiplexer_node |
| `/bt_navigator/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | bt_navigator | — |
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | N/A | N/A | RELIABLE / VOLATILE | rviz2 | — |
| `/clock` | `rosgraph_msgs/msg/Clock` | N/A | N/A | BEST_EFFORT / VOLATILE | — | perimeter_roamer_node<br>amcl |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | N/A | N/A | RELIABLE / VOLATILE | twist_multiplexer_node | teensy_bridge |
| `/cmd_vel_joystick` | `geometry_msgs/msg/Twist` | N/A | N/A | RELIABLE / VOLATILE | — | twist_multiplexer_node |
| `/cmd_vel_keyboard` | `geometry_msgs/msg/Twist` | N/A | N/A | RELIABLE / VOLATILE | — | twist_multiplexer_node |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | N/A | N/A | RELIABLE / VOLATILE | controller_server<br>behavior_server<br>behavior_server<br>behavior_server<br>behavior_server<br>behavior_server | velocity_smoother |
| `/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | N/A | N/A | RELIABLE / VOLATILE | velocity_smoother | twist_multiplexer_node |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | N/A | N/A | RELIABLE / VOLATILE | — | behavior_server |
| `/controller_selector` | `std_msgs/msg/String` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | — | bt_navigator_navigate_to_pose_rclcpp_node<br>bt_navigator_navigate_through_poses_rclcpp_node |
| `/controller_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | controller_server | — |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 3.12 | N/A | RELIABLE / VOLATILE | ekf_filter_node<br>lifecycle_manager_localization<br>lifecycle_manager_navigation | — |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 0.83 | 324.5 KB/s | RELIABLE / TRANSIENT_LOCAL | global_costmap | rviz2 |
| `/global_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | 0.89 | 335.9 KB/s | RELIABLE / TRANSIENT_LOCAL | global_costmap | smoother_server |
| `/global_costmap/costmap_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | global_costmap | smoother_server |
| `/global_costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | global_costmap | rviz2 |
| `/global_costmap/footprint` | `geometry_msgs/msg/Polygon` | N/A | N/A | RELIABLE / VOLATILE | — | global_costmap |
| `/global_costmap/global_costmap/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | global_costmap | — |
| `/global_costmap/obstacle_layer` | `nav_msgs/msg/OccupancyGrid` | 0.91 | 340.9 KB/s | RELIABLE / TRANSIENT_LOCAL | global_costmap | — |
| `/global_costmap/obstacle_layer_raw` | `nav2_msgs/msg/Costmap` | 0.86 | 320.3 KB/s | RELIABLE / TRANSIENT_LOCAL | global_costmap | — |
| `/global_costmap/obstacle_layer_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | global_costmap | — |
| `/global_costmap/obstacle_layer_updates` | `map_msgs/msg/OccupancyGridUpdate` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | global_costmap | — |
| `/global_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 5.00 | 1.1 KB/s | RELIABLE / VOLATILE | global_costmap | smoother_server |
| `/global_costmap/static_layer` | `nav_msgs/msg/OccupancyGrid` | 0.83 | 298.4 KB/s | RELIABLE / TRANSIENT_LOCAL | global_costmap | — |
| `/global_costmap/static_layer_raw` | `nav2_msgs/msg/Costmap` | 0.86 | 317.1 KB/s | RELIABLE / TRANSIENT_LOCAL | global_costmap | — |
| `/global_costmap/static_layer_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | global_costmap | — |
| `/global_costmap/static_layer_updates` | `map_msgs/msg/OccupancyGridUpdate` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | global_costmap | — |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | N/A | N/A | BEST_EFFORT / VOLATILE | — | rviz2<br>bt_navigator |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | N/A | N/A | RELIABLE / VOLATILE | rviz2 | amcl |
| `/joint_states` | `sensor_msgs/msg/JointState` | 10.00 | 3.9 KB/s | RELIABLE / VOLATILE | joint_state_publisher | robot_state_publisher |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | local_costmap | rviz2 |
| `/local_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | local_costmap | behavior_server |
| `/local_costmap/costmap_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | 4.02 | 14.8 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | behavior_server |
| `/local_costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | 4.14 | 15.2 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | rviz2 |
| `/local_costmap/footprint` | `geometry_msgs/msg/Polygon` | N/A | N/A | RELIABLE / VOLATILE | — | local_costmap |
| `/local_costmap/lidar_layer` | `nav_msgs/msg/OccupancyGrid` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/lidar_layer_raw` | `nav2_msgs/msg/Costmap` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/lidar_layer_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | 4.09 | 15.2 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/lidar_layer_updates` | `map_msgs/msg/OccupancyGridUpdate` | 4.18 | 14.8 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/local_costmap/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | local_costmap | — |
| `/local_costmap/oakd_top_layer` | `nav_msgs/msg/OccupancyGrid` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/oakd_top_layer_raw` | `nav2_msgs/msg/Costmap` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/oakd_top_layer_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | 4.21 | 15.0 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/oakd_top_layer_updates` | `map_msgs/msg/OccupancyGridUpdate` | 4.14 | 14.9 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 15.00 | 3.3 KB/s | RELIABLE / VOLATILE | local_costmap | rviz2<br>behavior_server |
| `/local_costmap/static_layer` | `nav_msgs/msg/OccupancyGrid` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/static_layer_raw` | `nav2_msgs/msg/Costmap` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/static_layer_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | 4.19 | 15.3 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/static_layer_updates` | `map_msgs/msg/OccupancyGridUpdate` | 4.33 | 14.7 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/map` | `nav_msgs/msg/OccupancyGrid` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | map_server | rviz2<br>amcl<br>local_costmap<br>global_costmap |
| `/map_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | map_server | — |
| `/map_updates` | `map_msgs/msg/OccupancyGridUpdate` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | — | rviz2 |
| `/odom` | `nav_msgs/msg/Odometry` | 19.19 | 14.0 KB/s | RELIABLE / VOLATILE | ekf_filter_node | rviz2<br>controller_server<br>bt_navigator |
| `/optimal_trajectory` | `nav_msgs/msg/Path` | N/A | N/A | RELIABLE / VOLATILE | controller_server | — |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | N/A | N/A | RELIABLE / VOLATILE | scan_to_scan_filter_chain<br>MoveAShortDistanceAheadActionServer<br>SaySomethingActionServer<br>ldlidar<br>launch_ros_7298<br>_ros2cli_daemon_0_8e63fc833863472bbfb81cb8e4c77390<br>perimeter_roamer_node<br>joint_state_publisher<br>rviz2<br>rviz_navigation_dialog_action_client<br>robot_state_publisher<br>teensy_bridge<br>_ros2cli_daemon_0_3b43d3bea2d74e2aa222020e98bcd0c3<br>ekf_filter_node<br>map_server<br>amcl<br>lifecycle_manager_localization<br>controller_server<br>local_costmap<br>smoother_server<br>planner_server<br>global_costmap<br>behavior_server<br>bt_navigator<br>waypoint_follower<br>velocity_smoother<br>lifecycle_manager_navigation<br>bt_navigator_navigate_to_pose_rclcpp_node<br>bt_navigator_navigate_through_poses_rclcpp_node<br>twist_multiplexer_node | scan_to_scan_filter_chain<br>MoveAShortDistanceAheadActionServer<br>SaySomethingActionServer<br>ldlidar<br>perimeter_roamer_node<br>rviz2<br>transform_listener_impl_62282c398b70<br>rviz_navigation_dialog_action_client<br>transform_listener_impl_62282c7d73c0<br>robot_state_publisher<br>robot_state_publisher<br>teensy_bridge<br>ekf_filter_node<br>transform_listener_impl_58a8767ef410<br>nav2_container<br>map_server<br>amcl<br>lifecycle_manager_localization<br>transform_listener_impl_71181c002ae0<br>controller_server<br>local_costmap<br>smoother_server<br>planner_server<br>global_costmap<br>behavior_server<br>bt_navigator<br>waypoint_follower<br>velocity_smoother<br>lifecycle_manager_navigation<br>transform_listener_impl_711808007950<br>transform_listener_impl_7118000031f0<br>transform_listener_impl_711804009120<br>transform_listener_impl_7117f8002460<br>bt_navigator_navigate_to_pose_rclcpp_node<br>bt_navigator_navigate_through_poses_rclcpp_node<br>twist_multiplexer_node |
| `/particle_cloud` | `nav2_msgs/msg/ParticleCloud` | N/A | N/A | BEST_EFFORT / VOLATILE | amcl | — |
| `/plan` | `nav_msgs/msg/Path` | N/A | N/A | RELIABLE / VOLATILE | planner_server | rviz2 |
| `/plan_smoothed` | `nav_msgs/msg/Path` | N/A | N/A | RELIABLE / VOLATILE | smoother_server | — |
| `/planner_selector` | `std_msgs/msg/String` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | — | bt_navigator_navigate_to_pose_rclcpp_node<br>bt_navigator_navigate_through_poses_rclcpp_node |
| `/planner_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | planner_server | — |
| `/preempt_teleop` | `std_msgs/msg/Empty` | N/A | N/A | RELIABLE / VOLATILE | — | behavior_server |
| `/raw_scan` | `sensor_msgs/msg/LaserScan` | 10.00 | 34.4 KB/s | RELIABLE / VOLATILE | ldlidar | scan_to_scan_filter_chain |
| `/robot_description` | `std_msgs/msg/String` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | robot_state_publisher | joint_state_publisher |
| `/rosout` | `rcl_interfaces/msg/Log` | 4.67 | 1.0 KB/s | RELIABLE / TRANSIENT_LOCAL | scan_to_scan_filter_chain<br>MoveAShortDistanceAheadActionServer<br>SaySomethingActionServer<br>ldlidar<br>launch_ros_7298<br>_ros2cli_daemon_0_8e63fc833863472bbfb81cb8e4c77390<br>perimeter_roamer_node<br>joint_state_publisher<br>rviz2<br>transform_listener_impl_62282c398b70<br>rviz_navigation_dialog_action_client<br>transform_listener_impl_62282c7d73c0<br>robot_state_publisher<br>teensy_bridge<br>_ros2cli_daemon_0_3b43d3bea2d74e2aa222020e98bcd0c3<br>ekf_filter_node<br>transform_listener_impl_58a8767ef410<br>nav2_container<br>map_server<br>amcl<br>lifecycle_manager_localization<br>transform_listener_impl_71181c002ae0<br>controller_server<br>local_costmap<br>smoother_server<br>planner_server<br>global_costmap<br>behavior_server<br>bt_navigator<br>waypoint_follower<br>velocity_smoother<br>lifecycle_manager_navigation<br>transform_listener_impl_711808007950<br>transform_listener_impl_7118000031f0<br>transform_listener_impl_711804009120<br>transform_listener_impl_7117f8002460<br>bt_navigator_navigate_to_pose_rclcpp_node<br>bt_navigator_navigate_through_poses_rclcpp_node<br>twist_multiplexer_node | — |
| `/scan` | `sensor_msgs/msg/LaserScan` | 10.00 | 19.0 KB/s | RELIABLE / VOLATILE | scan_to_scan_filter_chain | perimeter_roamer_node<br>rviz2<br>amcl<br>local_costmap<br>global_costmap |
| `/set_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | N/A | N/A | RELIABLE / VOLATILE | — | ekf_filter_node |
| `/sigyn/teensy_bridge/battery/status` | `sensor_msgs/msg/BatteryState` | 5 | 3.78 KB/s | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/commands/config` | `std_msgs/msg/String` | N/A | N/A | RELIABLE / VOLATILE | — | teensy_bridge |
| `/sigyn/teensy_bridge/commands/estop` | `std_msgs/msg/Bool` | N/A | N/A | RELIABLE / VOLATILE | — | teensy_bridge |
| `/sigyn/teensy_bridge/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 65.81 | 6.8 KB/s | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/imu/sensor_0` | `sensor_msgs/msg/Imu` | 10 | 3.25 KB/s | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/imu/sensor_1` | `sensor_msgs/msg/Imu` | 10 | 3.25 KB/s | RELIABLE / VOLATILE | teensy_bridge | ekf_filter_node |
| `/sigyn/teensy_bridge/range/vl53l0x_0` | `sensor_msgs/msg/Range` | 4.62 | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/range/vl53l0x_1` | `sensor_msgs/msg/Range` | 4.76 | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/range/vl53l0x_2` | `sensor_msgs/msg/Range` | 2.89 | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/range/vl53l0x_3` | `sensor_msgs/msg/Range` | 4.77 | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/range/vl53l0x_4` | `sensor_msgs/msg/Range` | 4.78 | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/range/vl53l0x_5` | `sensor_msgs/msg/Range` | 4.77 | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/range/vl53l0x_6` | `sensor_msgs/msg/Range` | 4.77 | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/range/vl53l0x_7` | `sensor_msgs/msg/Range` | 4.74 | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/safety/estop_status` | `std_msgs/msg/Bool` | N/A | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/temperature/motor_0` | `sensor_msgs/msg/Temperature` | 1 | 45 B/s | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/temperature/motor_1` | `sensor_msgs/msg/Temperature` | 1 | 45 B/s | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/wheel_odom` | `nav_msgs/msg/Odometry` | 20.45 | 15.1 KB/s | RELIABLE / VOLATILE | teensy_bridge | ekf_filter_node |
| `/smoother_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | smoother_server | — |
| `/speed_limit` | `nav2_msgs/msg/SpeedLimit` | N/A | N/A | RELIABLE / VOLATILE | — | controller_server |
| `/stereo/points2` | `sensor_msgs/msg/LaserScan` | N/A | N/A | BEST_EFFORT / VOLATILE | — | local_costmap |
| `/tf` | `tf2_msgs/msg/TFMessage` | 39.28 | 10.8 KB/s | RELIABLE / VOLATILE | robot_state_publisher<br>ekf_filter_node<br>amcl | transform_listener_impl_62282c398b70<br>transform_listener_impl_62282c7d73c0<br>transform_listener_impl_58a8767ef410<br>transform_listener_impl_71181c002ae0<br>transform_listener_impl_711808007950<br>transform_listener_impl_7118000031f0<br>transform_listener_impl_711804009120<br>transform_listener_impl_7117f8002460<br>bt_navigator |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | robot_state_publisher | transform_listener_impl_62282c398b70<br>transform_listener_impl_62282c7d73c0<br>transform_listener_impl_58a8767ef410<br>transform_listener_impl_71181c002ae0<br>transform_listener_impl_711808007950<br>transform_listener_impl_7118000031f0<br>transform_listener_impl_711804009120<br>transform_listener_impl_7117f8002460<br>bt_navigator |
| `/trajectories` | `visualization_msgs/msg/MarkerArray` | N/A | N/A | RELIABLE / VOLATILE | controller_server | — |
| `/transformed_global_plan` | `nav_msgs/msg/Path` | N/A | N/A | RELIABLE / VOLATILE | controller_server | — |
| `/velocity_smoother/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | velocity_smoother | — |
| `/waypoint_follower/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | waypoint_follower | — |
| `/waypoints` | `visualization_msgs/msg/MarkerArray` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | rviz_navigation_dialog_action_client | rviz2 |

## Active Topics Summary

| Topic | Rate (Hz) | Bandwidth |
|-------|-----------|-----------|
| `/sigyn/teensy_bridge/diagnostics` | 65.81 | 6.8 KB/s |
| `/tf` | 39.28 | 10.8 KB/s |
| `/sigyn/wheel_odom` | 20.45 | 15.1 KB/s |
| `/odom` | 19.19 | 14.0 KB/s |
| `/local_costmap/published_footprint` | 15.00 | 3.3 KB/s |
| `/joint_states` | 10.00 | 3.9 KB/s |
| `/raw_scan` | 10.00 | 34.4 KB/s |
| `/scan` | 10.00 | 19.0 KB/s |
| `/global_costmap/published_footprint` | 5.00 | 1.1 KB/s |
| `/sigyn/teensy_bridge/range/vl53l0x_4` | 4.78 | N/A |
| `/sigyn/teensy_bridge/range/vl53l0x_3` | 4.77 | N/A |
| `/sigyn/teensy_bridge/range/vl53l0x_5` | 4.77 | N/A |
| `/sigyn/teensy_bridge/range/vl53l0x_6` | 4.77 | N/A |
| `/sigyn/teensy_bridge/range/vl53l0x_1` | 4.76 | N/A |
| `/sigyn/teensy_bridge/range/vl53l0x_7` | 4.74 | N/A |
| `/rosout` | 4.67 | 1.0 KB/s |
| `/sigyn/teensy_bridge/range/vl53l0x_0` | 4.62 | N/A |
| `/local_costmap/static_layer_updates` | 4.33 | 14.7 KB/s |
| `/local_costmap/oakd_top_layer_raw_updates` | 4.21 | 15.0 KB/s |
| `/local_costmap/static_layer_raw_updates` | 4.19 | 15.3 KB/s |
| `/local_costmap/lidar_layer_updates` | 4.18 | 14.8 KB/s |
| `/local_costmap/costmap_updates` | 4.14 | 15.2 KB/s |
| `/local_costmap/oakd_top_layer_updates` | 4.14 | 14.9 KB/s |
| `/local_costmap/lidar_layer_raw_updates` | 4.09 | 15.2 KB/s |
| `/local_costmap/costmap_raw_updates` | 4.02 | 14.8 KB/s |
| `/diagnostics` | 3.12 | N/A |
| `/sigyn/teensy_bridge/range/vl53l0x_2` | 2.89 | N/A |
| `/sigyn/teensy_bridge/temperature/motor_0` | 1.46 | N/A |
| `/sigyn/teensy_bridge/temperature/motor_1` | 0.99 | N/A |
| `/global_costmap/obstacle_layer` | 0.91 | 340.9 KB/s |
| `/global_costmap/costmap_raw` | 0.89 | 335.9 KB/s |
| `/global_costmap/obstacle_layer_raw` | 0.86 | 320.3 KB/s |
| `/global_costmap/static_layer_raw` | 0.86 | 317.1 KB/s |
| `/global_costmap/costmap` | 0.83 | 324.5 KB/s |
| `/global_costmap/static_layer` | 0.83 | 298.4 KB/s |

## High Bandwidth Topics (>1KB/s)

- `/global_costmap/obstacle_layer`: 340.9 KB/s
- `/global_costmap/costmap_raw`: 335.9 KB/s
- `/global_costmap/costmap`: 324.5 KB/s
- `/global_costmap/obstacle_layer_raw`: 320.3 KB/s
- `/global_costmap/static_layer_raw`: 317.1 KB/s
- `/global_costmap/static_layer`: 298.4 KB/s
- `/raw_scan`: 34.4 KB/s
- `/scan`: 19.0 KB/s
- `/local_costmap/static_layer_raw_updates`: 15.3 KB/s
- `/local_costmap/costmap_updates`: 15.2 KB/s

## QoS Profile Summary

- **RELIABLE / VOLATILE**: 59 topics
- **RELIABLE / TRANSIENT_LOCAL**: 37 topics
- **BEST_EFFORT / VOLATILE**: 4 topics
