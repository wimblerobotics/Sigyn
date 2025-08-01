# ROS2 Topic Analysis Report

📊 **Total Topics:** 99
🟢 **Active Topics:** 29
📡 **Total Bandwidth:** 2.1 MB/s

## Topic Details

| Topic | Type | Rate (Hz) | Bandwidth | QoS First Pub | Publishers | Subscribers |
|-------|------|-----------|-----------|---------------|------------|-------------|
| `/amcl/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | amcl | — |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | amcl | — |
| `/behavior_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | behavior_server | — |
| `/behavior_tree_log` | `nav2_msgs/msg/BehaviorTreeLog` | N/A | N/A | RELIABLE / VOLATILE | bt_navigator_navigate_to_pose_rclcpp_node<br>bt_navigator_navigate_through_poses_rclcpp_node | — |
| `/bluetoothJoystick` | `msgs/msg/BluetoothJoystick` | N/A | N/A | RELIABLE / VOLATILE | — | twist_multiplexer_node |
| `/bt_navigator/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | bt_navigator | — |
| `/clicked_point` | `geometry_msgs/msg/PointStamped` | N/A | N/A | RELIABLE / VOLATILE | rviz2 | — |
| `/clock` | `rosgraph_msgs/msg/Clock` | N/A | N/A | BEST_EFFORT / VOLATILE | — | amcl |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | N/A | N/A | RELIABLE / VOLATILE | twist_multiplexer_node | teensy_bridge |
| `/cmd_vel_joystick` | `geometry_msgs/msg/Twist` | N/A | N/A | RELIABLE / VOLATILE | — | twist_multiplexer_node |
| `/cmd_vel_keyboard` | `geometry_msgs/msg/Twist` | N/A | N/A | RELIABLE / VOLATILE | — | twist_multiplexer_node |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | N/A | N/A | RELIABLE / VOLATILE | controller_server<br>behavior_server<br>behavior_server<br>behavior_server<br>behavior_server<br>behavior_server | velocity_smoother |
| `/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | N/A | N/A | RELIABLE / VOLATILE | velocity_smoother | twist_multiplexer_node |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | N/A | N/A | BEST_EFFORT / VOLATILE | — | behavior_server |
| `/controller_selector` | `std_msgs/msg/String` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | — | bt_navigator_navigate_to_pose_rclcpp_node<br>bt_navigator_navigate_through_poses_rclcpp_node |
| `/controller_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | controller_server | — |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 3.21 | N/A | RELIABLE / VOLATILE | lifecycle_manager_localization<br>lifecycle_manager_navigation<br>ekf_filter_node | — |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 0.83 | 327.1 KB/s | RELIABLE / TRANSIENT_LOCAL | global_costmap | rviz2 |
| `/global_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | 0.88 | 320.1 KB/s | RELIABLE / TRANSIENT_LOCAL | global_costmap | smoother_server |
| `/global_costmap/costmap_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | global_costmap | smoother_server |
| `/global_costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | global_costmap | rviz2 |
| `/global_costmap/footprint` | `geometry_msgs/msg/Polygon` | N/A | N/A | BEST_EFFORT / VOLATILE | — | global_costmap |
| `/global_costmap/global_costmap/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | global_costmap | — |
| `/global_costmap/obstacle_layer` | `nav_msgs/msg/OccupancyGrid` | 0.87 | 342.1 KB/s | RELIABLE / TRANSIENT_LOCAL | global_costmap | — |
| `/global_costmap/obstacle_layer_raw` | `nav2_msgs/msg/Costmap` | 0.91 | 308.2 KB/s | RELIABLE / TRANSIENT_LOCAL | global_costmap | — |
| `/global_costmap/obstacle_layer_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | global_costmap | — |
| `/global_costmap/obstacle_layer_updates` | `map_msgs/msg/OccupancyGridUpdate` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | global_costmap | — |
| `/global_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 5.00 | 1.1 KB/s | RELIABLE / TRANSIENT_LOCAL | global_costmap | smoother_server |
| `/global_costmap/static_layer` | `nav_msgs/msg/OccupancyGrid` | 0.91 | 336.9 KB/s | RELIABLE / TRANSIENT_LOCAL | global_costmap | — |
| `/global_costmap/static_layer_raw` | `nav2_msgs/msg/Costmap` | 0.93 | 318.0 KB/s | RELIABLE / TRANSIENT_LOCAL | global_costmap | — |
| `/global_costmap/static_layer_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | global_costmap | — |
| `/global_costmap/static_layer_updates` | `map_msgs/msg/OccupancyGridUpdate` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | global_costmap | — |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | N/A | N/A | BEST_EFFORT / VOLATILE | — | bt_navigator |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | N/A | N/A | RELIABLE / VOLATILE | rviz2 | amcl |
| `/joint_states` | `sensor_msgs/msg/JointState` | 10.00 | 3.9 KB/s | RELIABLE / VOLATILE | joint_state_publisher | robot_state_publisher |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | local_costmap | rviz2 |
| `/local_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | local_costmap | behavior_server |
| `/local_costmap/costmap_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | 4.09 | 14.7 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | behavior_server |
| `/local_costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | 3.79 | 14.6 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | rviz2 |
| `/local_costmap/footprint` | `geometry_msgs/msg/Polygon` | N/A | N/A | BEST_EFFORT / VOLATILE | — | local_costmap |
| `/local_costmap/lidar_layer` | `nav_msgs/msg/OccupancyGrid` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/lidar_layer_raw` | `nav2_msgs/msg/Costmap` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/lidar_layer_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | 3.91 | 15.2 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/lidar_layer_updates` | `map_msgs/msg/OccupancyGridUpdate` | 3.96 | 15.3 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/local_costmap/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | local_costmap | — |
| `/local_costmap/oakd_top_layer` | `nav_msgs/msg/OccupancyGrid` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/oakd_top_layer_raw` | `nav2_msgs/msg/Costmap` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/oakd_top_layer_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | 3.92 | 14.1 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/oakd_top_layer_updates` | `map_msgs/msg/OccupancyGridUpdate` | 4.00 | 14.2 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 15.00 | 3.3 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | behavior_server<br>rviz2 |
| `/local_costmap/static_layer` | `nav_msgs/msg/OccupancyGrid` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/static_layer_raw` | `nav2_msgs/msg/Costmap` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/static_layer_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | 4.19 | 14.2 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/local_costmap/static_layer_updates` | `map_msgs/msg/OccupancyGridUpdate` | 4.04 | 15.4 KB/s | RELIABLE / TRANSIENT_LOCAL | local_costmap | — |
| `/map` | `nav_msgs/msg/OccupancyGrid` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | map_server | amcl<br>local_costmap<br>global_costmap<br>rviz2 |
| `/map_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | map_server | — |
| `/map_updates` | `map_msgs/msg/OccupancyGridUpdate` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | — | rviz2 |
| `/odom` | `nav_msgs/msg/Odometry` | 19.84 | 14.6 KB/s | RELIABLE / VOLATILE | ekf_filter_node | controller_server<br>bt_navigator<br>rviz2 |
| `/optimal_trajectory` | `nav_msgs/msg/Path` | N/A | N/A | RELIABLE / VOLATILE | controller_server | — |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | N/A | N/A | RELIABLE / VOLATILE | launch_ros_36179<br>robot_state_publisher<br>map_server<br>amcl<br>lifecycle_manager_localization<br>controller_server<br>local_costmap<br>smoother_server<br>planner_server<br>global_costmap<br>behavior_server<br>bt_navigator<br>waypoint_follower<br>velocity_smoother<br>lifecycle_manager_navigation<br>bt_navigator_navigate_to_pose_rclcpp_node<br>bt_navigator_navigate_through_poses_rclcpp_node<br>twist_multiplexer_node<br>ldlidar<br>scan_to_scan_filter_chain<br>ekf_filter_node<br>joint_state_publisher<br>SaySomethingActionServer<br>MoveAShortDistanceAheadActionServer<br>teensy_bridge<br>rviz2<br>rviz_navigation_dialog_action_client<br>_ros2cli_daemon_0_e7bfc79031a04057a5ebd947f79a9105 | robot_state_publisher<br>robot_state_publisher<br>nav2_container<br>map_server<br>amcl<br>lifecycle_manager_localization<br>transform_listener_impl_734a8c002080<br>controller_server<br>local_costmap<br>smoother_server<br>planner_server<br>global_costmap<br>behavior_server<br>bt_navigator<br>waypoint_follower<br>velocity_smoother<br>lifecycle_manager_navigation<br>transform_listener_impl_734a78002170<br>transform_listener_impl_734a700020b0<br>transform_listener_impl_734a74002080<br>transform_listener_impl_734a680020b0<br>bt_navigator_navigate_to_pose_rclcpp_node<br>bt_navigator_navigate_through_poses_rclcpp_node<br>twist_multiplexer_node<br>ldlidar<br>scan_to_scan_filter_chain<br>ekf_filter_node<br>transform_listener_impl_55b544f5d480<br>SaySomethingActionServer<br>MoveAShortDistanceAheadActionServer<br>teensy_bridge<br>rviz2<br>transform_listener_impl_5fc2f4b5fea0<br>rviz_navigation_dialog_action_client<br>transform_listener_impl_5fc2f5340480 |
| `/particle_cloud` | `nav2_msgs/msg/ParticleCloud` | N/A | N/A | BEST_EFFORT / VOLATILE | amcl | — |
| `/plan` | `nav_msgs/msg/Path` | N/A | N/A | RELIABLE / VOLATILE | planner_server | rviz2 |
| `/plan_smoothed` | `nav_msgs/msg/Path` | N/A | N/A | RELIABLE / VOLATILE | smoother_server | — |
| `/planner_selector` | `std_msgs/msg/String` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | — | bt_navigator_navigate_to_pose_rclcpp_node<br>bt_navigator_navigate_through_poses_rclcpp_node |
| `/planner_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | planner_server | — |
| `/preempt_teleop` | `std_msgs/msg/Empty` | N/A | N/A | BEST_EFFORT / VOLATILE | — | behavior_server |
| `/raw_scan` | `sensor_msgs/msg/LaserScan` | 10.00 | 37.1 KB/s | RELIABLE / VOLATILE | ldlidar | scan_to_scan_filter_chain |
| `/robot_description` | `std_msgs/msg/String` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | robot_state_publisher | joint_state_publisher |
| `/rosout` | `rcl_interfaces/msg/Log` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | launch_ros_36179<br>robot_state_publisher<br>nav2_container<br>map_server<br>amcl<br>lifecycle_manager_localization<br>transform_listener_impl_734a8c002080<br>controller_server<br>local_costmap<br>smoother_server<br>planner_server<br>global_costmap<br>behavior_server<br>bt_navigator<br>waypoint_follower<br>velocity_smoother<br>lifecycle_manager_navigation<br>transform_listener_impl_734a78002170<br>transform_listener_impl_734a700020b0<br>transform_listener_impl_734a74002080<br>transform_listener_impl_734a680020b0<br>bt_navigator_navigate_to_pose_rclcpp_node<br>bt_navigator_navigate_through_poses_rclcpp_node<br>twist_multiplexer_node<br>ldlidar<br>scan_to_scan_filter_chain<br>ekf_filter_node<br>transform_listener_impl_55b544f5d480<br>joint_state_publisher<br>SaySomethingActionServer<br>MoveAShortDistanceAheadActionServer<br>teensy_bridge<br>rviz2<br>transform_listener_impl_5fc2f4b5fea0<br>rviz_navigation_dialog_action_client<br>transform_listener_impl_5fc2f5340480<br>_ros2cli_daemon_0_e7bfc79031a04057a5ebd947f79a9105 | — |
| `/scan` | `sensor_msgs/msg/LaserScan` | 10.00 | 18.4 KB/s | RELIABLE / VOLATILE | scan_to_scan_filter_chain | amcl<br>local_costmap<br>global_costmap<br>rviz2 |
| `/set_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | N/A | N/A | RELIABLE / VOLATILE | — | ekf_filter_node |
| `/sigyn/teensy_bridge/battery/status` | `sensor_msgs/msg/BatteryState` | 4.99 | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/commands/config` | `std_msgs/msg/String` | N/A | N/A | RELIABLE / VOLATILE | — | teensy_bridge |
| `/sigyn/teensy_bridge/commands/estop` | `std_msgs/msg/Bool` | N/A | N/A | RELIABLE / VOLATILE | — | teensy_bridge |
| `/sigyn/teensy_bridge/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 7.09 | 1.6 KB/s | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/imu/sensor_0` | `sensor_msgs/msg/Imu` | 7.43 | 2.4 KB/s | RELIABLE / VOLATILE | teensy_bridge | ekf_filter_node<br>rviz2 |
| `/sigyn/teensy_bridge/imu/sensor_1` | `sensor_msgs/msg/Imu` | 6.95 | 3.2 KB/s | RELIABLE / VOLATILE | teensy_bridge | ekf_filter_node<br>rviz2 |
| `/sigyn/teensy_bridge/range/vl53l0x_0` | `sensor_msgs/msg/Range` | N/A | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/range/vl53l0x_1` | `sensor_msgs/msg/Range` | N/A | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/range/vl53l0x_2` | `sensor_msgs/msg/Range` | N/A | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/range/vl53l0x_3` | `sensor_msgs/msg/Range` | N/A | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/range/vl53l0x_4` | `sensor_msgs/msg/Range` | N/A | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/range/vl53l0x_5` | `sensor_msgs/msg/Range` | N/A | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/range/vl53l0x_6` | `sensor_msgs/msg/Range` | N/A | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/range/vl53l0x_7` | `sensor_msgs/msg/Range` | N/A | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/safety/estop_status` | `std_msgs/msg/Bool` | N/A | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/temperature/motor_0` | `sensor_msgs/msg/Temperature` | 1.49 | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/teensy_bridge/temperature/motor_1` | `sensor_msgs/msg/Temperature` | 0.99 | N/A | RELIABLE / VOLATILE | teensy_bridge | — |
| `/sigyn/wheel_odom` | `nav_msgs/msg/Odometry` | 21.58 | 16.7 KB/s | RELIABLE / VOLATILE | teensy_bridge | ekf_filter_node |
| `/smoother_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | smoother_server | — |
| `/speed_limit` | `nav2_msgs/msg/SpeedLimit` | N/A | N/A | RELIABLE / VOLATILE | — | controller_server |
| `/stereo/points2` | `sensor_msgs/msg/LaserScan` | N/A | N/A | BEST_EFFORT / VOLATILE | — | local_costmap |
| `/tf` | `tf2_msgs/msg/TFMessage` | 36.74 | 11.0 KB/s | RELIABLE / VOLATILE | robot_state_publisher<br>amcl<br>ekf_filter_node | transform_listener_impl_734a8c002080<br>transform_listener_impl_734a78002170<br>transform_listener_impl_734a700020b0<br>transform_listener_impl_734a74002080<br>transform_listener_impl_734a680020b0<br>bt_navigator<br>transform_listener_impl_55b544f5d480<br>transform_listener_impl_5fc2f4b5fea0<br>transform_listener_impl_5fc2f5340480 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | robot_state_publisher | transform_listener_impl_734a8c002080<br>transform_listener_impl_734a78002170<br>transform_listener_impl_734a700020b0<br>transform_listener_impl_734a74002080<br>transform_listener_impl_734a680020b0<br>bt_navigator<br>transform_listener_impl_55b544f5d480<br>transform_listener_impl_5fc2f4b5fea0<br>transform_listener_impl_5fc2f5340480 |
| `/trajectories` | `visualization_msgs/msg/MarkerArray` | N/A | N/A | RELIABLE / VOLATILE | controller_server | — |
| `/transformed_global_plan` | `nav_msgs/msg/Path` | N/A | N/A | RELIABLE / VOLATILE | controller_server | — |
| `/velocity_smoother/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | velocity_smoother | — |
| `/waypoint_follower/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | N/A | N/A | RELIABLE / VOLATILE | waypoint_follower | — |
| `/waypoints` | `visualization_msgs/msg/MarkerArray` | N/A | N/A | RELIABLE / TRANSIENT_LOCAL | rviz_navigation_dialog_action_client | — |

## Active Topics Summary

| Topic | Rate (Hz) | Bandwidth |
|-------|-----------|-----------|
| `/tf` | 36.74 | 11.0 KB/s |
| `/sigyn/wheel_odom` | 21.58 | 16.7 KB/s |
| `/odom` | 19.84 | 14.6 KB/s |
| `/local_costmap/published_footprint` | 15.00 | 3.3 KB/s |
| `/joint_states` | 10.00 | 3.9 KB/s |
| `/raw_scan` | 10.00 | 37.1 KB/s |
| `/scan` | 10.00 | 18.4 KB/s |
| `/sigyn/teensy_bridge/imu/sensor_0` | 7.43 | 2.4 KB/s |
| `/sigyn/teensy_bridge/diagnostics` | 7.09 | 1.6 KB/s |
| `/sigyn/teensy_bridge/imu/sensor_1` | 6.95 | 3.2 KB/s |
| `/global_costmap/published_footprint` | 5.00 | 1.1 KB/s |
| `/sigyn/teensy_bridge/battery/status` | 4.99 | N/A |
| `/local_costmap/static_layer_raw_updates` | 4.19 | 14.2 KB/s |
| `/local_costmap/costmap_raw_updates` | 4.09 | 14.7 KB/s |
| `/local_costmap/static_layer_updates` | 4.04 | 15.4 KB/s |
| `/local_costmap/oakd_top_layer_updates` | 4.00 | 14.2 KB/s |
| `/local_costmap/lidar_layer_updates` | 3.96 | 15.3 KB/s |
| `/local_costmap/oakd_top_layer_raw_updates` | 3.92 | 14.1 KB/s |
| `/local_costmap/lidar_layer_raw_updates` | 3.91 | 15.2 KB/s |
| `/local_costmap/costmap_updates` | 3.79 | 14.6 KB/s |
| `/diagnostics` | 3.21 | N/A |
| `/sigyn/teensy_bridge/temperature/motor_0` | 1.49 | N/A |
| `/sigyn/teensy_bridge/temperature/motor_1` | 0.99 | N/A |
| `/global_costmap/static_layer_raw` | 0.93 | 318.0 KB/s |
| `/global_costmap/obstacle_layer_raw` | 0.91 | 308.2 KB/s |
| `/global_costmap/static_layer` | 0.91 | 336.9 KB/s |
| `/global_costmap/costmap_raw` | 0.88 | 320.1 KB/s |
| `/global_costmap/obstacle_layer` | 0.87 | 342.1 KB/s |
| `/global_costmap/costmap` | 0.83 | 327.1 KB/s |

## High Bandwidth Topics (>1KB/s)

- `/global_costmap/obstacle_layer`: 342.1 KB/s
- `/global_costmap/static_layer`: 336.9 KB/s
- `/global_costmap/costmap`: 327.1 KB/s
- `/global_costmap/costmap_raw`: 320.1 KB/s
- `/global_costmap/static_layer_raw`: 318.0 KB/s
- `/global_costmap/obstacle_layer_raw`: 308.2 KB/s
- `/raw_scan`: 37.1 KB/s
- `/scan`: 18.4 KB/s
- `/sigyn/wheel_odom`: 16.7 KB/s
- `/local_costmap/static_layer_updates`: 15.4 KB/s

## QoS Profile Summary

- **RELIABLE / VOLATILE**: 52 topics
- **RELIABLE / TRANSIENT_LOCAL**: 39 topics
- **BEST_EFFORT / VOLATILE**: 8 topics
