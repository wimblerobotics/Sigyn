<style>
@page {
  size: A4 landscape;
  margin: 1.0cm 0.3cm 1.0cm 0.3cm;
}
/* Override pandoc standalone template's narrow body max-width */
body { font-size: 9pt; max-width: none !important; padding: 0 !important; margin: 0 !important; }
#content, .content, main, article, div { max-width: none !important; }
table {
  font-size: 7pt;
  border-collapse: collapse;
  width: 100%;
  table-layout: fixed;
}
th {
  white-space: nowrap;
  padding: 2px 4px;
  vertical-align: bottom;
  overflow: hidden;
}
td {
  padding: 2px 4px;
  vertical-align: top;
  overflow-wrap: break-word;
  word-wrap: break-word;
  word-break: break-all;
}
/* 8-column summary table — fixed widths summing to 100% */
/* col 1=Topic, 2=BW, 3=Hz, 4=AvgMsg, 5=Type, 6=Pub, 7=Sub, 8=Health */
table th:nth-child(1), table td:nth-child(1) { width: 22%; }
table th:nth-child(2), table td:nth-child(2) { width:  8%; }
table th:nth-child(3), table td:nth-child(3) { width:  7%; }
table th:nth-child(4), table td:nth-child(4) { width:  6%; }
table th:nth-child(5), table td:nth-child(5) { width: 15%; }
table th:nth-child(6), table td:nth-child(6) { width: 19%; }
table th:nth-child(7), table td:nth-child(7) { width: 17%; }
table th:nth-child(8), table td:nth-child(8) { width:  6%; }
code { font-size: 6.5pt; }
/* Repeat table header row on every printed page */
thead { display: table-header-group; }
tbody { display: table-row-group; }
</style>

# Sigyn — ROS 2 Topic Analysis

> **Robot:** Sigyn  
> **Host:** `sigyn7900a`  
> **ROS Distro:** jazzy  
> **Generated:** 2026-04-21T20:08:39.770700+00:00  
> **Sampling window:** 15.0 s (all topics measured in parallel)  
> **Source data:** `topic_data_described.json`  
> **Update descriptions:** edit `topic_descriptions.json`, re-run `describe.py`, re-run `report.py`

---

## Table of Contents

1. [Top 20 Topics by Bandwidth](#top-20-topics-by-bandwidth)
2. [Full Topic Summary Table](#full-topic-summary-table)
3. [Topic Details by Namespace](#topic-details-by-namespace)
   - [Root Topics](#root-topics)
   - [AMCL Localisation](#amcl-localisation)
   - [Assisted_Teleop](#assisted_teleop)
   - [Backup](#backup)
   - [Behavior Server](#behavior-server)
   - [BT Navigator](#bt-navigator)
   - [Compute_Path_Through_Poses](#compute_path_through_poses)
   - [Compute_Path_To_Pose](#compute_path_to_pose)
   - [Controller Server](#controller-server)
   - [Drive_On_Heading](#drive_on_heading)
   - [Follow_Gps_Waypoints](#follow_gps_waypoints)
   - [Follow_Path](#follow_path)
   - [Follow_Waypoints](#follow_waypoints)
   - [Global Costmap](#global-costmap)
   - [Global_Costmap / Global_Costmap](#global_costmap--global_costmap)
   - [Gripper / Elevator](#gripper--elevator)
   - [Local Costmap](#local-costmap)
   - [Local_Costmap / Local_Costmap](#local_costmap--local_costmap)
   - [Map Server](#map-server)
   - [Navigate_Through_Poses](#navigate_through_poses)
   - [Navigate_To_Pose](#navigate_to_pose)
   - [OAK-D Detection](#oak-d-detection)
   - [Oakd / Annotated_Image](#oakd--annotated_image)
   - [OAK-D Top Camera](#oak-d-top-camera)
   - [Oakd_Top / Color](#oakd_top--color)
   - [Oakd_Top / Out](#oakd_top--out)
   - [Planner Server](#planner-server)
   - [Sigyn (Other)](#sigyn-other)
   - [Power](#power)
   - [Sigyn / Roboclaw](#sigyn--roboclaw)
   - [Safety System](#safety-system)
   - [Sigyn Sensors](#sigyn-sensors)
   - [Sigyn / Stepper](#sigyn--stepper)
   - [Sigyn / Teensy_Bridge](#sigyn--teensy_bridge)
   - [Smooth_Path](#smooth_path)
   - [Smoother Server](#smoother-server)
   - [Spin](#spin)
   - [Stereo](#stereo)
   - [Teensy Bridge](#teensy-bridge)
   - [Velocity Smoother](#velocity-smoother)
   - [Wait](#wait)
   - [Waypoint Follower](#waypoint-follower)
4. [Silent Topics (Publisher Exists, No Messages Observed)](#silent-topics)

---

## Top 20 Topics by Bandwidth

Topics ordered by measured bandwidth (highest first). Silent topics excluded.

| # | Topic | Bandwidth | Hz | Avg Msg Size | Type |
|---|-------|-----------|-----|--------------|------|
| 1 | [`/oakd_top/depth_raw`](#oakd_top-depth_raw) | **258.47 MB/s** | 7.7 Hz (3.8–9.4) | 4.15 MB | `sensor_msgs/msg/Image` |
| 2 | [`/oakd_top/points`](#oakd_top-points) | **91.92 MB/s** | 7.4 Hz (3.7–9.9) | 1.54 MB | `sensor_msgs/msg/PointCloud2` |
| 3 | [`/oakd_top/rgb_preview`](#oakd_top-rgb_preview) | **33.48 MB/s** | 8.0 Hz (4.1–109.4) | 519.2 kB | `sensor_msgs/msg/Image` |
| 4 | [`/oakd/annotated_image`](#oakd-annotated_image) | **33.20 MB/s** | 7.9 Hz (4.0–161.3) | 519.2 kB | `sensor_msgs/msg/Image` |
| 5 | [`/robot_description`](#robot_description) | **19.26 MB/s** | 61.4 Hz (18.4–243.5) | 32.7 kB | `std_msgs/msg/String` |
| 6 | [`/oakd_top/depth_image`](#oakd_top-depth_image) | **5.19 MB/s** | 1.6 Hz (1.5–1.7) | 388.9 kB | `sensor_msgs/msg/Image` |
| 7 | [`/global_costmap/costmap_raw`](#global_costmap-costmap_raw) | **2.77 MB/s** | 1.0 Hz (0.8–1.6) | 337.6 kB | `nav2_msgs/msg/Costmap` |
| 8 | [`/global_costmap/costmap`](#global_costmap-costmap) | **2.77 MB/s** | 1.0 Hz (0.8–1.6) | 337.6 kB | `nav_msgs/msg/OccupancyGrid` |
| 9 | [`/global_costmap/obstacle_layer`](#global_costmap-obstacle_layer) | **2.70 MB/s** | 0.9 Hz (0.8–1.0) | 337.6 kB | `nav_msgs/msg/OccupancyGrid` |
| 10 | [`/global_costmap/obstacle_layer_raw`](#global_costmap-obstacle_layer_raw) | **2.70 MB/s** | 0.9 Hz (0.8–1.0) | 337.6 kB | `nav2_msgs/msg/Costmap` |
| 11 | [`/global_costmap/static_layer_raw`](#global_costmap-static_layer_raw) | **2.70 MB/s** | 0.9 Hz (0.8–1.0) | 337.6 kB | `nav2_msgs/msg/Costmap` |
| 12 | [`/map`](#map) | **2.70 MB/s** | — | 337.6 kB | `nav_msgs/msg/OccupancyGrid` |
| 13 | [`/global_costmap/static_layer`](#global_costmap-static_layer) | **2.70 MB/s** | 0.9 Hz (0.8–1.0) | 337.6 kB | `nav_msgs/msg/OccupancyGrid` |
| 14 | [`/tf_static`](#tf_static) | **2.20 MB/s** | 58.2 Hz (16.8–270.1) | 3.9 kB | `tf2_msgs/msg/TFMessage` |
| 15 | [`/oakd/annotated_image/compressed`](#oakd-annotated_image-compressed) | **1.30 MB/s** | 7.9 Hz (4.0–153.4) | 20.3 kB | `sensor_msgs/msg/CompressedImage` |
| 16 | [`/tf`](#tf) | **600.8 kB/s** | 130.1 Hz (20.0–415.7) | 577 B | `tf2_msgs/msg/TFMessage` |
| 17 | [`/local_costmap/clearing_endpoints`](#local_costmap-clearing_endpoints) | **558.4 kB/s** | 15.0 Hz (13.2–17.2) | 4.6 kB | `sensor_msgs/msg/PointCloud2` |
| 18 | [`/scan_cup`](#scan_cup) | **290.6 kB/s** | 9.8 Hz (5.0–11.5) | 3.7 kB | `sensor_msgs/msg/LaserScan` |
| 19 | [`/raw_scan`](#raw_scan) | **290.0 kB/s** | 9.8 Hz (5.0–11.7) | 3.7 kB | `sensor_msgs/msg/LaserScan` |
| 20 | [`/scan`](#scan) | **289.7 kB/s** | 9.8 Hz (5.0–11.7) | 3.7 kB | `sensor_msgs/msg/LaserScan` |

---

## Full Topic Summary Table

All topics, sorted by topic name. <span style="color:green">&#9679;</span> OK &nbsp;<span style="color:#bb8800">&#9679;</span> slightly low &nbsp;<span style="color:red">&#9679;</span> low rate &nbsp;<span style="color:orange">&#9679;</span> silent (has publisher) &nbsp;(no publisher)

| Topic | Bandwidth | Hz | Avg Msg | Type | Publishers | Subscribers | Status |
|-------|----------:|---:|--------:|------|-----------|------------|--------|
| [`/amcl/transition_event`](#amcl-transition_event) | — | — | — | `lifecycle_msgs/TransitionEvent` | `/amcl` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/amcl_pose`](#amcl_pose) | 2.9 kB/s | — | 364 B | `geometry_msgs/PoseWithCovarianceStamped` | `/amcl` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/assisted_teleop/_action/feedback`](#assisted_teleop-_action-feedback) | — | — | — | `nav2_msgs/action/AssistedTeleop_FeedbackMessage` | `/behavior_server` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/assisted_teleop/_action/status`](#assisted_teleop-_action-status) | — | — | — | `action_msgs/GoalStatusArray` | `/behavior_server` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/backup/_action/feedback`](#backup-_action-feedback) | — | — | — | `nav2_msgs/action/BackUp_FeedbackMessage` | `/behavior_server` | `/bt_navigator_navigate_to_pose_rclcpp_node`<br>`/bt_navigator_navigate_through_poses_rclcpp_node` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/backup/_action/status`](#backup-_action-status) | — | — | — | `action_msgs/GoalStatusArray` | `/behavior_server` | `/bt_navigator_navigate_to_pose_rclcpp_node`<br>`/bt_navigator_navigate_through_poses_rclcpp_node` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/battery_overlay_text`](#battery_overlay_text) | — | — | — | `rviz_2d_overlay_msgs/OverlayText` | `/battery_overlay_publisher` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/behavior_server/transition_event`](#behavior_server-transition_event) | — | — | — | `lifecycle_msgs/TransitionEvent` | `/behavior_server` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/behavior_tree_log`](#behavior_tree_log) | — | — | — | `nav2_msgs/BehaviorTreeLog` | `/bt_navigator_navigate_to_pose_rclcpp_node`<br>`/bt_navigator_navigate_through_poses_rclcpp_node` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/bond`](#bond) | 132.0 kB/s | 179.2 Hz | 92 B | `bond/Status` | `/map_server`<br>`/lifecycle_manager_localization`<br>`/amcl`<br>`/lifecycle_manager_localization`<br>`/controller_server`<br>`/lifecycle_manager_navigation`<br>`/smoother_server`<br>`/lifecycle_manager_navigation`<br>`/planner_server`<br>`/lifecycle_manager_navigation`<br>`/behavior_server`<br>`/lifecycle_manager_navigation`<br>`/velocity_smoother`<br>`/lifecycle_manager_navigation`<br>`/bt_navigator`<br>`/lifecycle_manager_navigation`<br>`/waypoint_follower`<br>`/lifecycle_manager_navigation` | `/map_server`<br>`/lifecycle_manager_localization`<br>`/amcl`<br>`/lifecycle_manager_localization`<br>`/controller_server`<br>`/lifecycle_manager_navigation`<br>`/smoother_server`<br>`/lifecycle_manager_navigation`<br>`/planner_server`<br>`/lifecycle_manager_navigation`<br>`/behavior_server`<br>`/lifecycle_manager_navigation`<br>`/velocity_smoother`<br>`/lifecycle_manager_navigation`<br>`/bt_navigator`<br>`/lifecycle_manager_navigation`<br>`/waypoint_follower`<br>`/lifecycle_manager_navigation` | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/bt_navigator/transition_event`](#bt_navigator-transition_event) | — | — | — | `lifecycle_msgs/TransitionEvent` | `/bt_navigator` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/clock`](#clock) | — | — | — | `rosgraph_msgs/Clock` | NONE | `/map_server`<br>`/amcl` | (no publisher) |
| [`/cmd_vel`](#cmd_vel) | — | — | — | `geometry_msgs/Twist` | `/wr_teleop_twist_keyboard`<br>`/wr_twist_multiplexer_node` | `/teensy_bridge` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/cmd_vel_gripper`](#cmd_vel_gripper) | — | — | — | `geometry_msgs/Twist` | NONE | `/teensy_bridge` | (no publisher) |
| [`/cmd_vel_joystick`](#cmd_vel_joystick) | — | — | — | `geometry_msgs/Twist` | NONE | `/wr_twist_multiplexer_node` | (no publisher) |
| [`/cmd_vel_keyboard`](#cmd_vel_keyboard) | — | — | — | `geometry_msgs/Twist` | NONE | `/wr_twist_multiplexer_node` | (no publisher) |
| [`/cmd_vel_nav`](#cmd_vel_nav) | — | — | — | `geometry_msgs/Twist` | `/controller_server`<br>`/behavior_server`<br>`/behavior_server`<br>`/behavior_server`<br>`/behavior_server`<br>`/behavior_server` | `/wr_twist_multiplexer_node`<br>`/velocity_smoother` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/cmd_vel_smoothed`](#cmd_vel_smoothed) | — | — | — | `geometry_msgs/Twist` | `/velocity_smoother` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/cmd_vel_teleop`](#cmd_vel_teleop) | — | — | — | `geometry_msgs/Twist` | NONE | `/behavior_server` | (no publisher) |
| [`/compute_path_through_poses/_action/feedback`](#compute_path_through_poses-_action-feedback) | — | — | — | `nav2_msgs/action/ComputePathThroughPoses_FeedbackMessage` | `/planner_server` | `/bt_navigator_navigate_through_poses_rclcpp_node` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/compute_path_through_poses/_action/status`](#compute_path_through_poses-_action-status) | — | — | — | `action_msgs/GoalStatusArray` | `/planner_server` | `/bt_navigator_navigate_through_poses_rclcpp_node` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/compute_path_to_pose/_action/feedback`](#compute_path_to_pose-_action-feedback) | — | — | — | `nav2_msgs/action/ComputePathToPose_FeedbackMessage` | `/planner_server` | `/bt_navigator_navigate_to_pose_rclcpp_node` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/compute_path_to_pose/_action/status`](#compute_path_to_pose-_action-status) | — | — | — | `action_msgs/GoalStatusArray` | `/planner_server` | `/bt_navigator_navigate_to_pose_rclcpp_node` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/controller_selector`](#controller_selector) | — | — | — | `std_msgs/String` | NONE | `/bt_navigator_navigate_to_pose_rclcpp_node`<br>`/bt_navigator_navigate_through_poses_rclcpp_node` | (no publisher) |
| [`/controller_server/transition_event`](#controller_server-transition_event) | — | — | — | `lifecycle_msgs/TransitionEvent` | `/controller_server` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/diagnostics`](#diagnostics) | 38.9 kB/s | 22.4 Hz | 217 B | `diagnostic_msgs/DiagnosticArray` | `/scan_to_scan_filter_chain`<br>`/teensy_bridge`<br>`/lifecycle_manager_localization`<br>`/lifecycle_manager_navigation`<br>`/ekf_filter_node` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/drive_on_heading/_action/feedback`](#drive_on_heading-_action-feedback) | — | — | — | `nav2_msgs/action/DriveOnHeading_FeedbackMessage` | `/behavior_server` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/drive_on_heading/_action/status`](#drive_on_heading-_action-status) | — | — | — | `action_msgs/GoalStatusArray` | `/behavior_server` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/follow_gps_waypoints/_action/feedback`](#follow_gps_waypoints-_action-feedback) | — | — | — | `nav2_msgs/action/FollowGPSWaypoints_FeedbackMessage` | `/waypoint_follower` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/follow_gps_waypoints/_action/status`](#follow_gps_waypoints-_action-status) | — | — | — | `action_msgs/GoalStatusArray` | `/waypoint_follower` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/follow_path/_action/feedback`](#follow_path-_action-feedback) | — | — | — | `nav2_msgs/action/FollowPath_FeedbackMessage` | `/controller_server` | `/bt_navigator_navigate_to_pose_rclcpp_node`<br>`/bt_navigator_navigate_through_poses_rclcpp_node` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/follow_path/_action/status`](#follow_path-_action-status) | — | — | — | `action_msgs/GoalStatusArray` | `/controller_server` | `/bt_navigator_navigate_to_pose_rclcpp_node`<br>`/bt_navigator_navigate_through_poses_rclcpp_node` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/follow_waypoints/_action/feedback`](#follow_waypoints-_action-feedback) | — | — | — | `nav2_msgs/action/FollowWaypoints_FeedbackMessage` | `/waypoint_follower` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/follow_waypoints/_action/status`](#follow_waypoints-_action-status) | — | — | — | `action_msgs/GoalStatusArray` | `/waypoint_follower` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/global_costmap/costmap`](#global_costmap-costmap) | 2.77 MB/s | 1.0 Hz | 337.6 kB | `nav_msgs/OccupancyGrid` | `/global_costmap/global_costmap` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/global_costmap/costmap_raw`](#global_costmap-costmap_raw) | 2.77 MB/s | 1.0 Hz | 337.6 kB | `nav2_msgs/Costmap` | `/global_costmap/global_costmap` | `/smoother_server` | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/global_costmap/costmap_raw_updates`](#global_costmap-costmap_raw_updates) | — | — | — | `nav2_msgs/CostmapUpdate` | `/global_costmap/global_costmap` | `/smoother_server` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/global_costmap/costmap_updates`](#global_costmap-costmap_updates) | — | — | — | `map_msgs/OccupancyGridUpdate` | `/global_costmap/global_costmap` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/global_costmap/footprint`](#global_costmap-footprint) | — | — | — | `geometry_msgs/Polygon` | NONE | `/global_costmap/global_costmap` | (no publisher) |
| [`/global_costmap/global_costmap/transition_event`](#global_costmap-global_costmap-transition_event) | — | — | — | `lifecycle_msgs/TransitionEvent` | `/global_costmap/global_costmap` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/global_costmap/obstacle_layer`](#global_costmap-obstacle_layer) | 2.70 MB/s | 0.9 Hz | 337.6 kB | `nav_msgs/OccupancyGrid` | `/global_costmap/global_costmap` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/global_costmap/obstacle_layer_raw`](#global_costmap-obstacle_layer_raw) | 2.70 MB/s | 0.9 Hz | 337.6 kB | `nav2_msgs/Costmap` | `/global_costmap/global_costmap` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/global_costmap/obstacle_layer_raw_updates`](#global_costmap-obstacle_layer_raw_updates) | — | — | — | `nav2_msgs/CostmapUpdate` | `/global_costmap/global_costmap` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/global_costmap/obstacle_layer_updates`](#global_costmap-obstacle_layer_updates) | — | — | — | `map_msgs/OccupancyGridUpdate` | `/global_costmap/global_costmap` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/global_costmap/published_footprint`](#global_costmap-published_footprint) | 8.8 kB/s | 5.0 Hz | 216 B | `geometry_msgs/PolygonStamped` | `/global_costmap/global_costmap` | `/smoother_server` | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/global_costmap/static_layer`](#global_costmap-static_layer) | 2.70 MB/s | 0.9 Hz | 337.6 kB | `nav_msgs/OccupancyGrid` | `/global_costmap/global_costmap` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/global_costmap/static_layer_raw`](#global_costmap-static_layer_raw) | 2.70 MB/s | 0.9 Hz | 337.6 kB | `nav2_msgs/Costmap` | `/global_costmap/global_costmap` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/global_costmap/static_layer_raw_updates`](#global_costmap-static_layer_raw_updates) | — | — | — | `nav2_msgs/CostmapUpdate` | `/global_costmap/global_costmap` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/global_costmap/static_layer_updates`](#global_costmap-static_layer_updates) | — | — | — | `map_msgs/OccupancyGridUpdate` | `/global_costmap/global_costmap` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/goal_pose`](#goal_pose) | — | — | — | `geometry_msgs/PoseStamped` | NONE | `/bt_navigator` | (no publisher) |
| [`/gripper/home`](#gripper-home) | — | — | — | `std_msgs/Empty` | NONE | `/teensy_bridge` | (no publisher) |
| [`/gripper/move_elevator/_action/feedback`](#gripper-move_elevator-_action-feedback) | — | — | — | `sigyn_interfaces/action/MoveElevator_FeedbackMessage` | `/teensy_bridge` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/gripper/move_elevator/_action/status`](#gripper-move_elevator-_action-status) | — | — | — | `action_msgs/GoalStatusArray` | `/teensy_bridge` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/gripper/move_extender/_action/feedback`](#gripper-move_extender-_action-feedback) | — | — | — | `sigyn_interfaces/action/MoveExtender_FeedbackMessage` | `/teensy_bridge` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/gripper/move_extender/_action/status`](#gripper-move_extender-_action-status) | — | — | — | `action_msgs/GoalStatusArray` | `/teensy_bridge` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/gripper/position/command`](#gripper-position-command) | — | — | — | `sigyn_interfaces/GripperPositionCommand` | NONE | `/teensy_bridge` | (no publisher) |
| [`/gripper/status`](#gripper-status) | 6.3 kB/s | 10.0 Hz | 78 B | `sigyn_interfaces/GripperStatus` | `/teensy_bridge` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/initialpose`](#initialpose) | — | — | — | `geometry_msgs/PoseWithCovarianceStamped` | NONE | `/amcl` | (no publisher) |
| [`/joint_states`](#joint_states) | 217.1 kB/s | 69.9 Hz | 388 B | `sensor_msgs/JointState` | `/joint_state_publisher`<br>`/joint_state_publisher`<br>`/joint_state_publisher`<br>`/joint_state_publisher`<br>`/joint_state_publisher`<br>`/joint_state_publisher`<br>`/joint_state_publisher` | `/robot_state_publisher`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/robot_state_publisher` | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/local_costmap/clearing_endpoints`](#local_costmap-clearing_endpoints) | 558.4 kB/s | 15.0 Hz | 4.6 kB | `sensor_msgs/PointCloud2` | `/local_costmap/local_costmap` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/local_costmap/costmap`](#local_costmap-costmap) | 49.5 kB/s | — | 6.2 kB | `nav_msgs/OccupancyGrid` | `/local_costmap/local_costmap` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/local_costmap/costmap_raw`](#local_costmap-costmap_raw) | 49.7 kB/s | — | 6.2 kB | `nav2_msgs/Costmap` | `/local_costmap/local_costmap` | `/behavior_server` | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/local_costmap/costmap_raw_updates`](#local_costmap-costmap_raw_updates) | 180.9 kB/s | 4.2 Hz | 5.3 kB | `nav2_msgs/CostmapUpdate` | `/local_costmap/local_costmap` | `/behavior_server` | <span style="color:red;font-size:9pt">&#9679;</span> low (4.2 Hz, exp 15.0) |
| [`/local_costmap/costmap_updates`](#local_costmap-costmap_updates) | 178.3 kB/s | 4.1 Hz | 5.3 kB | `map_msgs/OccupancyGridUpdate` | `/local_costmap/local_costmap` | NONE | <span style="color:red;font-size:9pt">&#9679;</span> low (4.1 Hz, exp 15.0) |
| [`/local_costmap/footprint`](#local_costmap-footprint) | — | — | — | `geometry_msgs/Polygon` | NONE | `/local_costmap/local_costmap` | (no publisher) |
| [`/local_costmap/local_costmap/transition_event`](#local_costmap-local_costmap-transition_event) | — | — | — | `lifecycle_msgs/TransitionEvent` | `/local_costmap/local_costmap` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/local_costmap/oakd_top_layer`](#local_costmap-oakd_top_layer) | — | — | — | `nav_msgs/OccupancyGrid` | `/local_costmap/local_costmap` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/local_costmap/oakd_top_layer_raw`](#local_costmap-oakd_top_layer_raw) | — | — | — | `nav2_msgs/Costmap` | `/local_costmap/local_costmap` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/local_costmap/oakd_top_layer_raw_updates`](#local_costmap-oakd_top_layer_raw_updates) | 178.3 kB/s | 4.1 Hz | 5.3 kB | `nav2_msgs/CostmapUpdate` | `/local_costmap/local_costmap` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/local_costmap/oakd_top_layer_updates`](#local_costmap-oakd_top_layer_updates) | 178.3 kB/s | 4.1 Hz | 5.3 kB | `map_msgs/OccupancyGridUpdate` | `/local_costmap/local_costmap` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/local_costmap/published_footprint`](#local_costmap-published_footprint) | 26.5 kB/s | 15.0 Hz | 220 B | `geometry_msgs/PolygonStamped` | `/local_costmap/local_costmap` | `/behavior_server` | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/local_costmap/range_sensor_layer`](#local_costmap-range_sensor_layer) | — | — | — | `nav_msgs/OccupancyGrid` | `/local_costmap/local_costmap` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/local_costmap/range_sensor_layer_raw`](#local_costmap-range_sensor_layer_raw) | — | — | — | `nav2_msgs/Costmap` | `/local_costmap/local_costmap` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/local_costmap/range_sensor_layer_raw_updates`](#local_costmap-range_sensor_layer_raw_updates) | 178.3 kB/s | 4.1 Hz | 5.3 kB | `nav2_msgs/CostmapUpdate` | `/local_costmap/local_costmap` | NONE | <span style="color:red;font-size:9pt">&#9679;</span> low (4.1 Hz, exp 15.0) |
| [`/local_costmap/range_sensor_layer_updates`](#local_costmap-range_sensor_layer_updates) | 178.3 kB/s | 4.1 Hz | 5.3 kB | `map_msgs/OccupancyGridUpdate` | `/local_costmap/local_costmap` | NONE | <span style="color:red;font-size:9pt">&#9679;</span> low (4.1 Hz, exp 15.0) |
| [`/local_costmap/voxel_layer`](#local_costmap-voxel_layer) | — | — | — | `nav_msgs/OccupancyGrid` | `/local_costmap/local_costmap` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/local_costmap/voxel_layer_raw`](#local_costmap-voxel_layer_raw) | — | — | — | `nav2_msgs/Costmap` | `/local_costmap/local_costmap` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/local_costmap/voxel_layer_raw_updates`](#local_costmap-voxel_layer_raw_updates) | 178.3 kB/s | 4.1 Hz | 5.3 kB | `nav2_msgs/CostmapUpdate` | `/local_costmap/local_costmap` | NONE | <span style="color:red;font-size:9pt">&#9679;</span> low (4.1 Hz, exp 15.0) |
| [`/local_costmap/voxel_layer_updates`](#local_costmap-voxel_layer_updates) | 178.3 kB/s | 4.1 Hz | 5.3 kB | `map_msgs/OccupancyGridUpdate` | `/local_costmap/local_costmap` | NONE | <span style="color:red;font-size:9pt">&#9679;</span> low (4.1 Hz, exp 15.0) |
| [`/map`](#map) | 2.70 MB/s | — | 337.6 kB | `nav_msgs/OccupancyGrid` | `/map_server` | `/amcl`<br>`/global_costmap/global_costmap` | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/map_server/transition_event`](#map_server-transition_event) | — | — | — | `lifecycle_msgs/TransitionEvent` | `/map_server` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/navigate_through_poses/_action/feedback`](#navigate_through_poses-_action-feedback) | — | — | — | `nav2_msgs/action/NavigateThroughPoses_FeedbackMessage` | `/bt_navigator` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/navigate_through_poses/_action/status`](#navigate_through_poses-_action-status) | — | — | — | `action_msgs/GoalStatusArray` | `/bt_navigator` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/navigate_to_pose/_action/feedback`](#navigate_to_pose-_action-feedback) | — | — | — | `nav2_msgs/action/NavigateToPose_FeedbackMessage` | `/bt_navigator` | `/bt_navigator`<br>`/waypoint_follower` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/navigate_to_pose/_action/status`](#navigate_to_pose-_action-status) | — | — | — | `action_msgs/GoalStatusArray` | `/bt_navigator` | `/bt_navigator`<br>`/waypoint_follower` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/oakd/annotated_image`](#oakd-annotated_image) | 33.20 MB/s | 7.9 Hz | 519.2 kB | `sensor_msgs/Image` | `/oakd_detector` | NONE | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (7.9 Hz) |
| [`/oakd/annotated_image/compressed`](#oakd-annotated_image-compressed) | 1.30 MB/s | 7.9 Hz | 20.3 kB | `sensor_msgs/CompressedImage` | `/oakd_detector` | NONE | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (7.9 Hz) |
| [`/oakd/can_detections`](#oakd-can_detections) | 3.3 kB/s | 7.9 Hz | 52 B | `sigyn_interfaces/OakdDetectionArray` | `/oakd_detector` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/oakd/object_detector_heartbeat`](#oakd-object_detector_heartbeat) | 3.3 kB/s | 7.9 Hz | 52 B | `vision_msgs/Detection2DArray` | `/oakd_detector` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/oakd_top/camera_info`](#oakd_top-camera_info) | 24.6 kB/s | 8.0 Hz | 381 B | `sensor_msgs/CameraInfo` | `/oakd_detector` | NONE | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (8.0 Hz) |
| [`/oakd_top/can_point_base`](#oakd_top-can_point_base) | — | — | — | `geometry_msgs/PointStamped` | `/oakd_detector` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/oakd_top/can_point_camera`](#oakd_top-can_point_camera) | — | — | — | `geometry_msgs/PointStamped` | `/oakd_detector` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/oakd_top/can_point_raw`](#oakd_top-can_point_raw) | — | — | — | `geometry_msgs/PointStamped` | `/oakd_detector` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/oakd_top/color/image/compressed`](#oakd_top-color-image-compressed) | — | — | — | `sensor_msgs/CompressedImage` | `/oakd_top/oakd_color_compressed_republisher` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/oakd_top/depth_image`](#oakd_top-depth_image) | 5.19 MB/s | 1.6 Hz | 388.9 kB | `sensor_msgs/Image` | `/oakd_detector` | NONE | <span style="color:red;font-size:9pt">&#9679;</span> low (1.6 Hz, exp 15.0) |
| [`/oakd_top/depth_raw`](#oakd_top-depth_raw) | 258.47 MB/s | 7.7 Hz | 4.15 MB | `sensor_msgs/Image` | `/oakd_detector` | NONE | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (7.7 Hz) |
| [`/oakd_top/depth_sample_base`](#oakd_top-depth_sample_base) | — | — | — | `geometry_msgs/PointStamped` | `/oakd_detector` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/oakd_top/depth_sample_camera`](#oakd_top-depth_sample_camera) | — | — | — | `geometry_msgs/PointStamped` | `/oakd_detector` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/oakd_top/out`](#oakd_top-out) | — | — | — | `sensor_msgs/Image` | `/oakd_top/oakd_color_compressed_republisher` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/oakd_top/out/compressedDepth`](#oakd_top-out-compressedDepth) | — | — | — | `sensor_msgs/CompressedImage` | `/oakd_top/oakd_color_compressed_republisher` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/oakd_top/out/theora`](#oakd_top-out-theora) | — | — | — | `theora_image_transport/Packet` | `/oakd_top/oakd_color_compressed_republisher` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/oakd_top/out/zstd`](#oakd_top-out-zstd) | — | — | — | `sensor_msgs/CompressedImage` | `/oakd_top/oakd_color_compressed_republisher` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/oakd_top/points`](#oakd_top-points) | 91.92 MB/s | 7.4 Hz | 1.54 MB | `sensor_msgs/PointCloud2` | `/oakd_detector` | NONE | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (7.4 Hz) |
| [`/oakd_top/rgb_preview`](#oakd_top-rgb_preview) | 33.48 MB/s | 8.0 Hz | 519.2 kB | `sensor_msgs/Image` | `/oakd_detector` | NONE | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (8.0 Hz) |
| [`/odom`](#odom) | 187.4 kB/s | 32.3 Hz | 724 B | `nav_msgs/Odometry` | `/ekf_filter_node` | `/controller_server`<br>`/bt_navigator` | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (32.3 Hz) |
| [`/optimal_trajectory`](#optimal_trajectory) | — | — | — | `nav_msgs/Path` | `/controller_server` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/parameter_events`](#parameter_events) | — | — | — | `rcl_interfaces/ParameterEvent` | `/joint_state_publisher`<br>`/battery_overlay_publisher`<br>`/robot_state_publisher`<br>`/wr_teleop_twist_keyboard`<br>`/scan_to_scan_filter_chain`<br>`/pointcloud_to_laserscan_node`<br>`/robot_state_publisher`<br>`/top_ldlidar`<br>`/joint_state_publisher`<br>`/wr_twist_multiplexer_node`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/teensy_bridge`<br>`/_ros2cli_daemon_0_d9b837226db446f08c14e93c332b5d5a`<br>`/map_server`<br>`/amcl`<br>`/lifecycle_manager_localization`<br>`/controller_server`<br>`/local_costmap/local_costmap`<br>`/smoother_server`<br>`/planner_server`<br>`/global_costmap/global_costmap`<br>`/behavior_server`<br>`/bt_navigator`<br>`/waypoint_follower`<br>`/velocity_smoother`<br>`/lifecycle_manager_navigation`<br>`/bt_navigator_navigate_to_pose_rclcpp_node`<br>`/bt_navigator_navigate_through_poses_rclcpp_node`<br>`/robot_state_publisher`<br>`/launch_ros_4535`<br>`/ekf_filter_node`<br>`/joint_state_publisher`<br>`/joint_state_publisher`<br>`/topic_analysis_discovery`<br>`/oakd_top/oakd_color_compressed_republisher`<br>`/rqt_gui_py_node_1352476`<br>`/cup_ldlidar`<br>`/oakd_detector`<br>`/joint_state_publisher`<br>`/joint_state_publisher`<br>`/robot_state_publisher`<br>`/joint_state_publisher`<br>`/sigyn_notifier` | `/robot_state_publisher`<br>`/robot_state_publisher`<br>`/scan_to_scan_filter_chain`<br>`/pointcloud_to_laserscan_node`<br>`/transform_listener_impl_5e865b2ef690`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/top_ldlidar`<br>`/wr_twist_multiplexer_node`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/teensy_bridge`<br>`/nav2_container`<br>`/map_server`<br>`/amcl`<br>`/lifecycle_manager_localization`<br>`/controller_server`<br>`/local_costmap/local_costmap`<br>`/smoother_server`<br>`/planner_server`<br>`/global_costmap/global_costmap`<br>`/behavior_server`<br>`/bt_navigator`<br>`/waypoint_follower`<br>`/velocity_smoother`<br>`/lifecycle_manager_navigation`<br>`/transform_listener_impl_71275c008a00`<br>`/transform_listener_impl_712754000fa0`<br>`/bt_navigator_navigate_to_pose_rclcpp_node`<br>`/bt_navigator_navigate_through_poses_rclcpp_node`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/ekf_filter_node`<br>`/transform_listener_impl_5f8a1e348bd0`<br>`/oakd_top/oakd_color_compressed_republisher`<br>`/oakd_top/oakd_color_compressed_republisher`<br>`/oakd_top/oakd_color_compressed_republisher`<br>`/oakd_top/oakd_color_compressed_republisher`<br>`/cup_ldlidar`<br>`/robot_state_publisher`<br>`/robot_state_publisher` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/particle_cloud`](#particle_cloud) | — | — | — | `nav2_msgs/ParticleCloud` | `/amcl` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/plan`](#plan) | — | — | — | `nav_msgs/Path` | `/planner_server` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/plan_smoothed`](#plan_smoothed) | — | — | — | `nav_msgs/Path` | `/smoother_server` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/planner_selector`](#planner_selector) | — | — | — | `std_msgs/String` | NONE | `/bt_navigator_navigate_to_pose_rclcpp_node`<br>`/bt_navigator_navigate_through_poses_rclcpp_node` | (no publisher) |
| [`/planner_server/transition_event`](#planner_server-transition_event) | — | — | — | `lifecycle_msgs/TransitionEvent` | `/planner_server` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/preempt_teleop`](#preempt_teleop) | — | — | — | `std_msgs/Empty` | NONE | `/behavior_server` | (no publisher) |
| [`/raw_scan`](#raw_scan) | 290.0 kB/s | 9.8 Hz | 3.7 kB | `sensor_msgs/LaserScan` | `/top_ldlidar` | `/scan_to_scan_filter_chain` | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/robot_description`](#robot_description) | 19.26 MB/s | 61.4 Hz | 32.7 kB | `std_msgs/String` | `/robot_state_publisher`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/robot_state_publisher` | `/joint_state_publisher`<br>`/joint_state_publisher`<br>`/joint_state_publisher`<br>`/joint_state_publisher`<br>`/joint_state_publisher`<br>`/joint_state_publisher`<br>`/joint_state_publisher` | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/rosout`](#rosout) | 1.6 kB/s | — | 204 B | `rcl_interfaces/Log` | `/joint_state_publisher`<br>`/battery_overlay_publisher`<br>`/robot_state_publisher`<br>`/wr_teleop_twist_keyboard`<br>`/scan_to_scan_filter_chain`<br>`/pointcloud_to_laserscan_node`<br>`/transform_listener_impl_5e865b2ef690`<br>`/robot_state_publisher`<br>`/top_ldlidar`<br>`/joint_state_publisher`<br>`/wr_twist_multiplexer_node`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/teensy_bridge`<br>`/_ros2cli_daemon_0_d9b837226db446f08c14e93c332b5d5a`<br>`/nav2_container`<br>`/map_server`<br>`/amcl`<br>`/lifecycle_manager_localization`<br>`/controller_server`<br>`/local_costmap/local_costmap`<br>`/smoother_server`<br>`/planner_server`<br>`/global_costmap/global_costmap`<br>`/behavior_server`<br>`/bt_navigator`<br>`/waypoint_follower`<br>`/velocity_smoother`<br>`/lifecycle_manager_navigation`<br>`/transform_listener_impl_71275c008a00`<br>`/transform_listener_impl_712754000fa0`<br>`/bt_navigator_navigate_to_pose_rclcpp_node`<br>`/bt_navigator_navigate_through_poses_rclcpp_node`<br>`/robot_state_publisher`<br>`/launch_ros_4535`<br>`/ekf_filter_node`<br>`/transform_listener_impl_5f8a1e348bd0`<br>`/joint_state_publisher`<br>`/joint_state_publisher`<br>`/topic_analysis_discovery`<br>`/oakd_top/oakd_color_compressed_republisher`<br>`/rqt_gui_py_node_1352476`<br>`/cup_ldlidar`<br>`/oakd_detector`<br>`/joint_state_publisher`<br>`/joint_state_publisher`<br>`/robot_state_publisher`<br>`/joint_state_publisher`<br>`/sigyn_notifier` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/scan`](#scan) | 289.7 kB/s | 9.8 Hz | 3.7 kB | `sensor_msgs/LaserScan` | `/scan_to_scan_filter_chain` | `/amcl`<br>`/global_costmap/global_costmap` | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/scan_cup`](#scan_cup) | 290.6 kB/s | 9.8 Hz | 3.7 kB | `sensor_msgs/LaserScan` | `/cup_ldlidar` | `/local_costmap/local_costmap` | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/set_pose`](#set_pose) | — | — | — | `geometry_msgs/PoseWithCovarianceStamped` | NONE | `/ekf_filter_node` | (no publisher) |
| [`/sigyn/power/battery`](#sigyn-power-battery) | 659 B/s | 1.0 Hz | 77 B | `sensor_msgs/BatteryState` | `/teensy_bridge` | `/sigyn_notifier` | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/sigyn/power/rail`](#sigyn-power-rail) | 1.0 kB/s | 3.1 Hz | 40 B | `wr_interfaces/PowerRailStatus` | `/teensy_bridge` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/sigyn/roboclaw/status`](#sigyn-roboclaw-status) | 3.0 kB/s | 5.0 Hz | 73 B | `wr_interfaces/RoboClawStatus` | `/teensy_bridge` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/sigyn/safety/fault_events`](#sigyn-safety-fault_events) | 1.0 kB/s | 1.7 Hz | 67 B | `wr_interfaces/FaultEvent` | `/teensy_bridge` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/sigyn/safety/fault_list`](#sigyn-safety-fault_list) | 2.1 kB/s | 10.0 Hz | 26 B | `wr_interfaces/ActiveFaultList` | `/teensy_bridge` | `/sigyn_notifier` | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/sigyn/safety/human_alert`](#sigyn-safety-human_alert) | — | — | — | `wr_interfaces/HumanAlert` | `/teensy_bridge` | `/sigyn_notifier` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/sigyn/safety/state`](#sigyn-safety-state) | 2.3 kB/s | 10.0 Hz | 28 B | `wr_interfaces/SafetyState` | `/teensy_bridge` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/sigyn/sensors/imu_front_right`](#sigyn-sensors-imu_front_right) | 78.9 kB/s | 28.9 Hz | 340 B | `sensor_msgs/Imu` | `/teensy_bridge` | NONE | <span style="color:red;font-size:9pt">&#9679;</span> low (28.9 Hz, exp 100.0) |
| [`/sigyn/sensors/imu_rear_left`](#sigyn-sensors-imu_rear_left) | 77.1 kB/s | 28.9 Hz | 332 B | `sensor_msgs/Imu` | `/teensy_bridge` | NONE | <span style="color:red;font-size:9pt">&#9679;</span> low (28.9 Hz, exp 100.0) |
| [`/sigyn/sensors/odom`](#sigyn-sensors-odom) | 187.3 kB/s | 32.3 Hz | 724 B | `nav_msgs/Odometry` | `/teensy_bridge` | `/ekf_filter_node` | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (32.3 Hz) |
| [`/sigyn/sensors/range/front_left_fwd`](#sigyn-sensors-range-front_left_fwd) | 12.4 kB/s | 25.8 Hz | 60 B | `sensor_msgs/Range` | `/teensy_bridge` | `/local_costmap/local_costmap`<br>`/rqt_gui_py_node_1352476` | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (25.8 Hz) |
| [`/sigyn/sensors/range/front_left_side`](#sigyn-sensors-range-front_left_side) | 12.9 kB/s | 25.2 Hz | 64 B | `sensor_msgs/Range` | `/teensy_bridge` | `/local_costmap/local_costmap` | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (25.2 Hz) |
| [`/sigyn/sensors/range/front_right_fwd`](#sigyn-sensors-range-front_right_fwd) | 13.2 kB/s | 25.7 Hz | 64 B | `sensor_msgs/Range` | `/teensy_bridge` | `/local_costmap/local_costmap` | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (25.7 Hz) |
| [`/sigyn/sensors/range/front_right_side`](#sigyn-sensors-range-front_right_side) | 13.0 kB/s | 25.4 Hz | 64 B | `sensor_msgs/Range` | `/teensy_bridge` | `/local_costmap/local_costmap` | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (25.4 Hz) |
| [`/sigyn/sensors/range/rear_left_bkwd`](#sigyn-sensors-range-rear_left_bkwd) | 12.4 kB/s | 25.7 Hz | 60 B | `sensor_msgs/Range` | `/teensy_bridge` | `/local_costmap/local_costmap` | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (25.7 Hz) |
| [`/sigyn/sensors/range/rear_left_side`](#sigyn-sensors-range-rear_left_side) | 12.2 kB/s | 25.4 Hz | 60 B | `sensor_msgs/Range` | `/teensy_bridge` | `/local_costmap/local_costmap` | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (25.4 Hz) |
| [`/sigyn/sensors/range/rear_right_bkwd`](#sigyn-sensors-range-rear_right_bkwd) | 13.1 kB/s | 25.5 Hz | 64 B | `sensor_msgs/Range` | `/teensy_bridge` | `/local_costmap/local_costmap` | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (25.5 Hz) |
| [`/sigyn/sensors/range/rear_right_side`](#sigyn-sensors-range-rear_right_side) | 13.0 kB/s | 25.2 Hz | 64 B | `sensor_msgs/Range` | `/teensy_bridge` | `/local_costmap/local_costmap` | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (25.2 Hz) |
| [`/sigyn/stepper/status`](#sigyn-stepper-status) | — | — | — | `wr_interfaces/StepperStatus` | `/teensy_bridge` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/sigyn/take_oakd_picture`](#sigyn-take_oakd_picture) | — | — | — | `std_msgs/Bool` | NONE | `/oakd_detector` | (no publisher) |
| [`/sigyn/teensy_bridge/battery/status`](#sigyn-teensy_bridge-battery-status) | — | — | — | `sensor_msgs/BatteryState` | NONE | `/battery_overlay_publisher` | (no publisher) |
| [`/smooth_path/_action/feedback`](#smooth_path-_action-feedback) | — | — | — | `nav2_msgs/action/SmoothPath_FeedbackMessage` | `/smoother_server` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/smooth_path/_action/status`](#smooth_path-_action-status) | — | — | — | `action_msgs/GoalStatusArray` | `/smoother_server` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/smoother_server/transition_event`](#smoother_server-transition_event) | — | — | — | `lifecycle_msgs/TransitionEvent` | `/smoother_server` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/speed_limit`](#speed_limit) | — | — | — | `nav2_msgs/SpeedLimit` | NONE | `/controller_server` | (no publisher) |
| [`/spin/_action/feedback`](#spin-_action-feedback) | — | — | — | `nav2_msgs/action/Spin_FeedbackMessage` | `/behavior_server` | `/bt_navigator_navigate_to_pose_rclcpp_node`<br>`/bt_navigator_navigate_through_poses_rclcpp_node` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/spin/_action/status`](#spin-_action-status) | — | — | — | `action_msgs/GoalStatusArray` | `/behavior_server` | `/bt_navigator_navigate_to_pose_rclcpp_node`<br>`/bt_navigator_navigate_through_poses_rclcpp_node` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/stereo/points2`](#stereo-points2) | — | — | — | `sensor_msgs/LaserScan` | `/pointcloud_to_laserscan_node` | `/local_costmap/local_costmap` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/teensy_bridge/status`](#teensy_bridge-status) | 305 B/s | 2.0 Hz | 18 B | `std_msgs/String` | `/teensy_bridge` | NONE | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/teensy_bridge/transition_event`](#teensy_bridge-transition_event) | — | — | — | `lifecycle_msgs/TransitionEvent` | `/teensy_bridge` | `/launch_ros_4535` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/tf`](#tf) | 600.8 kB/s | 130.1 Hz | 577 B | `tf2_msgs/TFMessage` | `/robot_state_publisher`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/amcl`<br>`/robot_state_publisher`<br>`/ekf_filter_node`<br>`/robot_state_publisher` | `/transform_listener_impl_5e865b2ef690`<br>`/amcl`<br>`/transform_listener_impl_71275c008a00`<br>`/smoother_server`<br>`/transform_listener_impl_712754000fa0`<br>`/behavior_server`<br>`/bt_navigator`<br>`/transform_listener_impl_5f8a1e348bd0`<br>`/oakd_detector` | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/tf_static`](#tf_static) | 2.20 MB/s | 58.2 Hz | 3.9 kB | `tf2_msgs/TFMessage` | `/robot_state_publisher`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/robot_state_publisher`<br>`/robot_state_publisher` | `/transform_listener_impl_5e865b2ef690`<br>`/amcl`<br>`/transform_listener_impl_71275c008a00`<br>`/smoother_server`<br>`/transform_listener_impl_712754000fa0`<br>`/behavior_server`<br>`/bt_navigator`<br>`/transform_listener_impl_5f8a1e348bd0`<br>`/oakd_detector` | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| [`/trajectories`](#trajectories) | — | — | — | `visualization_msgs/MarkerArray` | `/controller_server` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/transformed_global_plan`](#transformed_global_plan) | — | — | — | `nav_msgs/Path` | `/controller_server` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/velocity_smoother/transition_event`](#velocity_smoother-transition_event) | — | — | — | `lifecycle_msgs/TransitionEvent` | `/velocity_smoother` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/wait/_action/feedback`](#wait-_action-feedback) | — | — | — | `nav2_msgs/action/Wait_FeedbackMessage` | `/behavior_server` | `/bt_navigator_navigate_to_pose_rclcpp_node`<br>`/bt_navigator_navigate_through_poses_rclcpp_node` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/wait/_action/status`](#wait-_action-status) | — | — | — | `action_msgs/GoalStatusArray` | `/behavior_server` | `/bt_navigator_navigate_to_pose_rclcpp_node`<br>`/bt_navigator_navigate_through_poses_rclcpp_node` | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| [`/waypoint_follower/transition_event`](#waypoint_follower-transition_event) | — | — | — | `lifecycle_msgs/TransitionEvent` | `/waypoint_follower` | NONE | <span style="color:orange;font-size:9pt">&#9679;</span> silent |

---

## Topic Details by Namespace

### Root Topics

#### `/amcl_pose` {#amcl_pose}

**Best-estimate robot pose in the map frame, published by the AMCL localiser.**

AMCL (Adaptive Monte Carlo Localisation) publishes this PoseWithCovarianceStamped when the robot moves enough to trigger a filter update (configured by update_min_d=0.01 m and update_min_a=0.05 rad). The covariance ellipse reflects localiser certainty. Nav2 and EKF nodes consume this topic to correct the odom-to-map transform.

AMCL is a particle-filter localiser. It maintains a cloud of hypotheses (2000–5000 on Sigyn) and weights them by how well the current laser scan matches the static map. The /amcl_pose output is the weighted mean of the cloud, expressed as a PoseWithCovarianceStamped.

When to expect it: AMCL only publishes after the robot has moved at least update_min_d=0.01 m or rotated at least update_min_a≈3°. During idle the topic goes quiet — this is normal. The save_pose_rate=0.5 Hz parameter also throttles writes to the parameter server.

Relationship to /particle_cloud: /particle_cloud exposes the raw hypotheses; /amcl_pose is the summarised result. You can monitor cloud spread as a proxy for localisation confidence.

Relationship to /tf: AMCL simultaneously broadcasts the map→odom transform so downstream nodes see a consistent world frame. On Sigyn, tf_broadcast=true.

Custom use: Subscribe to this topic to detect kidnap (sudden large pose jump), trigger re-mapping, or log localisation drift over a mission. The covariance diagonal elements encode uncertainty in x, y, and yaw; a rule of thumb is to alert if cov[0]>0.05 (5 cm²) during autonomous navigation.

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/PoseWithCovarianceStamped` |
| **Bandwidth** | 2.9 kB/s |
| **Rate** | — |
| **Avg message size** | 364 B |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 1 |

**Publishers:**

- `/amcl` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/battery_overlay_text` {#battery_overlay_text}

**Human-readable battery status string for display overlays.**

A std_msgs/String published by Sigyn's battery monitoring node, intended for display in RViz2 overlays or heads-up dashboards. Contains a formatted summary of voltage, current, and state-of-charge derived from /sigyn/power/battery.

This topic is Sigyn-specific. Its producer reads /sigyn/power/battery (sensor_msgs/BatteryState) and formats a compact string such as '48.2V / 12.3A / 78% SOC'. The string is designed to fit in a RViz2 TextViewFacing overlay panel.

For book readers: this pattern — deriving a human-readable string topic from a machine-readable sensor topic — is common in field robots where operators monitor the robot via a dashboard rather than reading raw ROS topics. It decouples the display format from the sensor protocol. A downside is that the text is harder to plot over time; for analytics, always use the original /sigyn/power/battery.

| Property | Value |
|----------|-------|
| **Type** | `rviz_2d_overlay_msgs/msg/OverlayText` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/battery_overlay_publisher` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/behavior_tree_log` {#behavior_tree_log}

**BehaviorTree.CPP execution log, one entry per BT node state change.**

Published by bt_navigator at the BT tick rate (~100 Hz on Sigyn). Each message records which BT node changed state and to what (RUNNING/SUCCESS/FAILURE). Used for debugging navigation behavior and replaying missions with Groot2.

Nav2 uses BehaviorTree.CPP (BT.CPP) as its execution engine. On every tick (10 ms / 100 Hz on Sigyn) the BT engine evaluates the tree and any node whose status changes emits a transition record to this topic.

The message type is nav2_msgs/msg/BehaviorTreeLog. Each BehaviorTreeStatusChange entry records uid (node ID), node_name, previous_status, and current_status.

Relationship to Groot2: Groot2 connects directly to BT.CPP's ZMQ publisher port, which provides live visualisation. /behavior_tree_log is the ROS 2 bridge of the same data for rosbag recording and offline analysis.

Bandwidth note: At 100 Hz tick rate with a complex tree (Sigyn's navigate_to_pose_w_recovery.xml has ~40 nodes), this topic can emit hundreds of messages per second when a recovery sequence is active. During idle navigation (robot moving toward a goal on a clear path) the rate drops significantly because most nodes remain in RUNNING without transitioning.

Custom use: Subscribe to detect when the robot enters a recovery behavior (spin, backup) and correlate with obstacle sensor data. This is invaluable for diagnosing why the robot keeps recovering on a particular route.

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/BehaviorTreeLog` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/bt_navigator_navigate_to_pose_rclcpp_node` — RELIABLE / VOLATILE (depth 10)
- `/bt_navigator_navigate_through_poses_rclcpp_node` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/bond` {#bond}

**Lifecycle bond heartbeat between Nav2 servers and the lifecycle manager.**

Bond messages are internal to Nav2. The lifecycle manager uses them to detect crashed servers.

| Property | Value |
|----------|-------|
| **Type** | `bond/msg/Status` |
| **Bandwidth** | 132.0 kB/s |
| **Rate** | 179.2 Hz (43.0–443.8) |
| **Avg message size** | 92 B |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 2598 |

**Publishers:**

- `/map_server` — RELIABLE / VOLATILE (depth 5)
- `/lifecycle_manager_localization` — RELIABLE / VOLATILE (depth 5)
- `/amcl` — RELIABLE / VOLATILE (depth 5)
- `/lifecycle_manager_localization` — RELIABLE / VOLATILE (depth 5)
- `/controller_server` — RELIABLE / VOLATILE (depth 5)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 5)
- `/smoother_server` — RELIABLE / VOLATILE (depth 5)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 5)
- `/planner_server` — RELIABLE / VOLATILE (depth 5)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 5)
- `/behavior_server` — RELIABLE / VOLATILE (depth 5)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 5)
- `/velocity_smoother` — RELIABLE / VOLATILE (depth 5)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 5)
- `/bt_navigator` — RELIABLE / VOLATILE (depth 5)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 5)
- `/waypoint_follower` — RELIABLE / VOLATILE (depth 5)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 5)

**Subscribers:**

- `/map_server` — RELIABLE / VOLATILE (depth 100)
- `/lifecycle_manager_localization` — RELIABLE / VOLATILE (depth 100)
- `/amcl` — RELIABLE / VOLATILE (depth 100)
- `/lifecycle_manager_localization` — RELIABLE / VOLATILE (depth 100)
- `/controller_server` — RELIABLE / VOLATILE (depth 100)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 100)
- `/smoother_server` — RELIABLE / VOLATILE (depth 100)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 100)
- `/planner_server` — RELIABLE / VOLATILE (depth 100)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 100)
- `/behavior_server` — RELIABLE / VOLATILE (depth 100)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 100)
- `/velocity_smoother` — RELIABLE / VOLATILE (depth 100)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 100)
- `/bt_navigator` — RELIABLE / VOLATILE (depth 100)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 100)
- `/waypoint_follower` — RELIABLE / VOLATILE (depth 100)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 100)

---

#### `/clock` {#clock}

**Simulation or hardware clock source; used by all nodes when use_sim_time is true.**

Published by gazebo/simulator when use_sim_time=true. In real-hardware mode (use_sim_time=false) this topic may not be present or may be ignored. All time-stamped messages and ROS timers use this clock in simulation.

TODO: Expand with Sigyn-specific note that use_sim_time toggles between Gazebo (sim) and system clock (real hardware), and how to switch.

| Property | Value |
|----------|-------|
| **Type** | `rosgraph_msgs/msg/Clock` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/map_server` — BEST_EFFORT / VOLATILE (depth 1)
- `/amcl` — BEST_EFFORT / VOLATILE (depth 1)

---

#### `/cmd_vel` {#cmd_vel}

**Final commanded velocity sent to the motor controller after all safety filtering.**

The authoritative velocity command consumed by wr_ros_teensy's CmdVelController. It is produced by the collision_monitor node, which post-processes cmd_vel_smoothed. Sources include navigation (cmd_vel_nav), teleop (cmd_vel_joystick, cmd_vel_keyboard), and assisted teleop, arbitrated by the twist multiplexer before safety filtering.

cmd_vel is the single most important velocity topic on Sigyn. Its lineage is:
  cmd_vel_nav (controller_server at 20 Hz)
  → wr_twist_multiplexer (arbitrates with teleop sources)
  → velocity_smoother (smooths at 20 Hz, real-time priority)
  → cmd_vel_smoothed
  → collision_monitor (halts if imminent collision)
  → cmd_vel  ← this topic
  → CmdVelController (applies safety authority clamps, 500 Hz watchdog)
  → Teensy Board 1 (RoboClaw motor driver)

Safety: CmdVelController enforces:
  - NORMAL: full passthrough
  - DEGRADED: 0.5× clamp
  - CRITICAL/ESTOP: zero velocity
The 500 ms watchdog zeroes velocity if no message arrives, protecting against PC freeze or node crash.

Tuning: Sigyn is configured vx_max=0.5 m/s, wz_max=1.5 rad/s in the MPPI controller. The velocity_smoother then limits acceleration (max_accel=[2.5, 0, 3.2]) so the robot does not jerk. If you observe jerky motion, examine cmd_vel_smoothed vs cmd_vel to see if collision_monitor is the cause, then look at cmd_vel_nav to see if the planner is sending jerky commands.

Book note: This three-stage pipeline (plan → smooth → safety gate) is a best-practice architecture for mobile robots that must share spaces with people. Each stage has a clearly defined responsibility and can be tested independently.

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/Twist` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/wr_teleop_twist_keyboard` — RELIABLE / VOLATILE (depth 1)
- `/wr_twist_multiplexer_node` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

---

#### `/cmd_vel_gripper` {#cmd_vel_gripper}

**Velocity-style jog command for the elevator/gripper assembly.**

A geometry_msgs/Twist routed to CmdVelElevatorController, which translates linear.x into elevator incremental movement and angular.z into extender movement. Used by joystick jog mode for the gripper.

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/Twist` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

---

#### `/cmd_vel_joystick` {#cmd_vel_joystick}

**Velocity command from the Bluetooth joystick teleop node.**

Published by sigyn_bluetooth_joystick when the operator is in teleop mode. Arbitrated by wr_twist_multiplexer against cmd_vel_nav. The joystick takes priority over navigation when the operator is actively sending commands.

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/Twist` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/wr_twist_multiplexer_node` — RELIABLE / VOLATILE (depth 10)

---

#### `/cmd_vel_keyboard` {#cmd_vel_keyboard}

**Velocity command from the keyboard teleop node (wr_teleop_twist_keyboard).**

Published by wr_teleop_twist_keyboard. Lower priority than joystick in the twist multiplexer. Useful for bench testing and fine-grained repositioning during development.

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/Twist` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/wr_twist_multiplexer_node` — RELIABLE / VOLATILE (depth 10)

---

#### `/cmd_vel_nav` {#cmd_vel_nav}

**Raw velocity command from the Nav2 controller server before smoothing.**

Published by the MPPI controller (wrapped in RotationShimController) at controller_frequency=20 Hz. This is the 'ideal' velocity the planner wants; it has not yet been smoothed or safety-gated. Useful for diagnosing whether jerky motion originates in the planner or in downstream stages.

On Sigyn the controller pipeline is RotationShimController → MPPIController. The shim handles in-place rotation when heading error exceeds ~45° (angular_dist_threshold=0.785 rad), then hands off to MPPI for path following.

MPPI publishes at exactly controller_frequency=20 Hz. If you see cmd_vel_nav publish at a lower rate, the costmap update is too slow (costmap_update_timeout=0.30 s). Compare the rate of cmd_vel_nav with /local_costmap/costmap_updates to diagnose.

Related: /trajectories and /optimal_trajectory are the MPPI visualisation outputs. They are expensive to publish; set visualize=true only for debugging.

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/Twist` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/controller_server` — RELIABLE / VOLATILE (depth 1)
- `/behavior_server` — RELIABLE / VOLATILE (depth 1)
- `/behavior_server` — RELIABLE / VOLATILE (depth 1)
- `/behavior_server` — RELIABLE / VOLATILE (depth 1)
- `/behavior_server` — RELIABLE / VOLATILE (depth 1)
- `/behavior_server` — RELIABLE / VOLATILE (depth 1)

**Subscribers:**

- `/wr_twist_multiplexer_node` — RELIABLE / VOLATILE (depth 10)
- `/velocity_smoother` — RELIABLE / VOLATILE (depth 1)

---

#### `/cmd_vel_smoothed` {#cmd_vel_smoothed}

**Velocity after the velocity_smoother node has applied acceleration limits.**

The velocity_smoother runs at smoothing_frequency=20 Hz and clamps acceleration (max_accel=[2.5, 0, 3.2]) so the robot cannot jerk. The output is fed to the collision_monitor before reaching /cmd_vel.

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/Twist` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/velocity_smoother` — RELIABLE / VOLATILE (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/cmd_vel_teleop` {#cmd_vel_teleop}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/Twist` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/behavior_server` — RELIABLE / VOLATILE (depth 1)

---

#### `/controller_selector` {#controller_selector}

**String topic for selecting which controller plugin is active.**

Published to switch between controller plugins at runtime (e.g., from MPPI to RPP). On Sigyn only FollowPath (MPPI) is configured, so this topic is not normally used.

| Property | Value |
|----------|-------|
| **Type** | `std_msgs/msg/String` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/bt_navigator_navigate_to_pose_rclcpp_node` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/bt_navigator_navigate_through_poses_rclcpp_node` — RELIABLE / TRANSIENT_LOCAL (depth 1)

---

#### `/diagnostics` {#diagnostics}

**Aggregated diagnostics from all nodes, published as DiagnosticArray at ~1 Hz.**

The standard ROS 2 diagnostics topic. All nodes that implement diagnostic_updater publish to /diagnostics. A diagnostic aggregator collects them. wr_ros_teensy publishes Teensy board health, sensor status, and communication link quality here.

TODO: Expand with Sigyn-specific diagnostics keys from TopicPublisher.cpp and safety coordinator.

| Property | Value |
|----------|-------|
| **Type** | `diagnostic_msgs/msg/DiagnosticArray` |
| **Bandwidth** | 38.9 kB/s |
| **Rate** | 22.4 Hz (3.9–402.7) |
| **Avg message size** | 217 B |
| **Delay (end-to-end)** | 6.4 ms (0.1–26.8) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 324 |

**Publishers:**

- `/scan_to_scan_filter_chain` — RELIABLE / VOLATILE (depth 1)
- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)
- `/lifecycle_manager_localization` — RELIABLE / VOLATILE (depth 1)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 1)
- `/ekf_filter_node` — RELIABLE / VOLATILE (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/goal_pose` {#goal_pose}

**Navigation goal pose in the map frame, published by RViz2 or mission nodes.**

The standard topic for sending a single navigation goal. bt_navigator subscribes and dispatches a NavigateToPose action. Also published by the house_patroller when triggering individual waypoint navigation.

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/PoseStamped` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/bt_navigator` — RELIABLE / VOLATILE (depth 1)

---

#### `/initialpose` {#initialpose}

**Initial pose estimate for AMCL, published by RViz2 or programmatically at startup.**

PoseWithCovarianceStamped in the map frame. AMCL resets its particle cloud around this estimate. On Sigyn, AMCL is configured set_initial_pose=true so it auto-initialises from the initial_pose parameters in navigation.yaml — the /initialpose topic is used when the operator needs to manually correct localisation.

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/PoseWithCovarianceStamped` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/amcl` — RELIABLE / VOLATILE (depth 1)

---

#### `/joint_states` {#joint_states}

**Joint positions for all articulated joints (wheels, elevator, gripper) for TF and RViz.**

Published by joint_state_publisher or wr_ros_teensy. Drives robot_state_publisher which broadcasts the corresponding TF transforms. Essential for correct robot model display in RViz2.

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/JointState` |
| **Bandwidth** | 217.1 kB/s |
| **Rate** | 69.9 Hz (5.3–446.4) |
| **Avg message size** | 388 B |
| **Delay (end-to-end)** | 25.5 ms (1.8–307.5) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 1043 |

**Publishers:**

- `/joint_state_publisher` — RELIABLE / VOLATILE (depth 10)
- `/joint_state_publisher` — RELIABLE / VOLATILE (depth 10)
- `/joint_state_publisher` — RELIABLE / VOLATILE (depth 10)
- `/joint_state_publisher` — RELIABLE / VOLATILE (depth 10)
- `/joint_state_publisher` — RELIABLE / VOLATILE (depth 10)
- `/joint_state_publisher` — RELIABLE / VOLATILE (depth 10)
- `/joint_state_publisher` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/robot_state_publisher` — BEST_EFFORT / VOLATILE (depth 5)
- `/robot_state_publisher` — BEST_EFFORT / VOLATILE (depth 5)
- `/robot_state_publisher` — BEST_EFFORT / VOLATILE (depth 5)
- `/robot_state_publisher` — BEST_EFFORT / VOLATILE (depth 5)
- `/robot_state_publisher` — BEST_EFFORT / VOLATILE (depth 5)
- `/robot_state_publisher` — BEST_EFFORT / VOLATILE (depth 5)

---

#### `/map` {#map}

**The static occupancy grid map loaded from disk, published TRANSIENT_LOCAL at startup.**

Published by map_server with TRANSIENT_LOCAL durability so late-joining nodes receive it immediately. The map is loaded from the YAML+PNG file configured at launch time. On Sigyn this is the house floor plan used for AMCL localisation and global path planning.

| Property | Value |
|----------|-------|
| **Type** | `nav_msgs/msg/OccupancyGrid` |
| **Bandwidth** | 2.70 MB/s |
| **Rate** | — |
| **Avg message size** | 337.6 kB |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 1 |

**Publishers:**

- `/map_server` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

- `/amcl` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/global_costmap/global_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

---

#### `/odom` {#odom}

**EKF-fused odometry from robot_localization, combining wheel encoder and IMU data.**

Published by the robot_localization EKF node at ~50 Hz. It fuses /sigyn/sensors/odom (wheel encoder odometry) with IMU data to produce a smoother, drift-corrected estimate. This is the primary odometry topic consumed by Nav2.

Sigyn uses two-stage localisation: robot_localization EKF provides odom→base_link (continuous, dead-reckoning); AMCL provides map→odom (corrected when a laser scan update triggers). The /odom topic is the EKF output.

The EKF fusion improves on raw wheel odometry by:
  - Reducing orientation drift using IMU gyro integration
  - Handling motor slip events (sudden encoder jump without corresponding IMU rotation)

Relationship to /sigyn/sensors/odom: The raw wheel encoder topic is what comes from the Teensy at 50 Hz. /odom is the EKF-processed version at the same rate. If the EKF diverges (common with very slippery floors), the symptom is the robot's local costmap drifting relative to obstacles — diagnose by comparing /odom vs /sigyn/sensors/odom covariance.

Book note: Always use the EKF output (/odom) as your odometry source in navigation, not the raw encoder topic. The EKF provides a consistent coordinate frame even when individual sensors produce noisy measurements.

| Property | Value |
|----------|-------|
| **Type** | `nav_msgs/msg/Odometry` |
| **Bandwidth** | 187.4 kB/s |
| **Rate** | 32.3 Hz (17.8–302.9) |
| **Avg message size** | 724 B |
| **Delay (end-to-end)** | 13.5 ms (3.1–36.2) |
| **Health** | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (32.3 Hz) |
| **Sample count** | 467 |

**Publishers:**

- `/ekf_filter_node` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/controller_server` — RELIABLE / VOLATILE (depth 1)
- `/bt_navigator` — RELIABLE / VOLATILE (depth 1)

---

#### `/optimal_trajectory` {#optimal_trajectory}

**The single best MPPI trajectory (lowest cost) selected at each controller tick.**

Published at controller_frequency=20 Hz when visualize=true. Shows the actual path the robot intends to follow over the next time_steps × model_dt = 56 × 0.05 s = 2.8 s horizon.

| Property | Value |
|----------|-------|
| **Type** | `nav_msgs/msg/Path` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/controller_server` — RELIABLE / VOLATILE (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/parameter_events` {#parameter_events}

**ROS 2 parameter change events, published when any node's parameter is modified.**

Every ROS 2 node publishes to this topic when a parameter changes. Used by rqt_reconfigure and parameter monitoring tools. Not normally consumed by application code.

| Property | Value |
|----------|-------|
| **Type** | `rcl_interfaces/msg/ParameterEvent` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/joint_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/battery_overlay_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/wr_teleop_twist_keyboard` — RELIABLE / VOLATILE (depth 1000)
- `/scan_to_scan_filter_chain` — RELIABLE / VOLATILE (depth 1000)
- `/pointcloud_to_laserscan_node` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/top_ldlidar` — RELIABLE / VOLATILE (depth 1000)
- `/joint_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/wr_twist_multiplexer_node` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/teensy_bridge` — RELIABLE / VOLATILE (depth 1000)
- `/_ros2cli_daemon_0_d9b837226db446f08c14e93c332b5d5a` — RELIABLE / VOLATILE (depth 1000)
- `/map_server` — RELIABLE / VOLATILE (depth 1000)
- `/amcl` — RELIABLE / VOLATILE (depth 1000)
- `/lifecycle_manager_localization` — RELIABLE / VOLATILE (depth 1000)
- `/controller_server` — RELIABLE / VOLATILE (depth 1000)
- `/local_costmap/local_costmap` — RELIABLE / VOLATILE (depth 1000)
- `/smoother_server` — RELIABLE / VOLATILE (depth 1000)
- `/planner_server` — RELIABLE / VOLATILE (depth 1000)
- `/global_costmap/global_costmap` — RELIABLE / VOLATILE (depth 1000)
- `/behavior_server` — RELIABLE / VOLATILE (depth 1000)
- `/bt_navigator` — RELIABLE / VOLATILE (depth 1000)
- `/waypoint_follower` — RELIABLE / VOLATILE (depth 1000)
- `/velocity_smoother` — RELIABLE / VOLATILE (depth 1000)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 1000)
- `/bt_navigator_navigate_to_pose_rclcpp_node` — RELIABLE / VOLATILE (depth 1000)
- `/bt_navigator_navigate_through_poses_rclcpp_node` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/launch_ros_4535` — RELIABLE / VOLATILE (depth 1000)
- `/ekf_filter_node` — RELIABLE / VOLATILE (depth 1000)
- `/joint_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/joint_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/topic_analysis_discovery` — RELIABLE / VOLATILE (depth 1000)
- `/oakd_top/oakd_color_compressed_republisher` — RELIABLE / VOLATILE (depth 1000)
- `/rqt_gui_py_node_1352476` — RELIABLE / VOLATILE (depth 1000)
- `/cup_ldlidar` — RELIABLE / VOLATILE (depth 1000)
- `/oakd_detector` — RELIABLE / VOLATILE (depth 1000)
- `/joint_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/joint_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/joint_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/sigyn_notifier` — RELIABLE / VOLATILE (depth 1000)

**Subscribers:**

- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/scan_to_scan_filter_chain` — RELIABLE / VOLATILE (depth 1000)
- `/pointcloud_to_laserscan_node` — RELIABLE / VOLATILE (depth 1000)
- `/transform_listener_impl_5e865b2ef690` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/top_ldlidar` — RELIABLE / VOLATILE (depth 1000)
- `/wr_twist_multiplexer_node` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/teensy_bridge` — RELIABLE / VOLATILE (depth 1000)
- `/nav2_container` — RELIABLE / VOLATILE (depth 1000)
- `/map_server` — RELIABLE / VOLATILE (depth 1000)
- `/amcl` — RELIABLE / VOLATILE (depth 1000)
- `/lifecycle_manager_localization` — RELIABLE / VOLATILE (depth 1000)
- `/controller_server` — RELIABLE / VOLATILE (depth 1000)
- `/local_costmap/local_costmap` — RELIABLE / VOLATILE (depth 1000)
- `/smoother_server` — RELIABLE / VOLATILE (depth 1000)
- `/planner_server` — RELIABLE / VOLATILE (depth 1000)
- `/global_costmap/global_costmap` — RELIABLE / VOLATILE (depth 1000)
- `/behavior_server` — RELIABLE / VOLATILE (depth 1000)
- `/bt_navigator` — RELIABLE / VOLATILE (depth 1000)
- `/waypoint_follower` — RELIABLE / VOLATILE (depth 1000)
- `/velocity_smoother` — RELIABLE / VOLATILE (depth 1000)
- `/lifecycle_manager_navigation` — RELIABLE / VOLATILE (depth 1000)
- `/transform_listener_impl_71275c008a00` — RELIABLE / VOLATILE (depth 1000)
- `/transform_listener_impl_712754000fa0` — RELIABLE / VOLATILE (depth 1000)
- `/bt_navigator_navigate_to_pose_rclcpp_node` — RELIABLE / VOLATILE (depth 1000)
- `/bt_navigator_navigate_through_poses_rclcpp_node` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/ekf_filter_node` — RELIABLE / VOLATILE (depth 1000)
- `/transform_listener_impl_5f8a1e348bd0` — RELIABLE / VOLATILE (depth 1000)
- `/oakd_top/oakd_color_compressed_republisher` — RELIABLE / VOLATILE (depth 1000)
- `/oakd_top/oakd_color_compressed_republisher` — RELIABLE / VOLATILE (depth 1000)
- `/oakd_top/oakd_color_compressed_republisher` — RELIABLE / VOLATILE (depth 1000)
- `/oakd_top/oakd_color_compressed_republisher` — RELIABLE / VOLATILE (depth 1000)
- `/cup_ldlidar` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 1000)

---

#### `/particle_cloud` {#particle_cloud}

**AMCL particle cloud (hypothesis set) as a PoseArray, published at AMCL update rate.**

Visualises the full set of localisation hypotheses. A tight cluster indicates high-confidence localisation; a spread cloud indicates uncertainty. Published only when the robot moves enough to trigger an AMCL update.

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/ParticleCloud` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/amcl` — BEST_EFFORT / VOLATILE (depth 5)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/plan` {#plan}

**Global path from robot pose to goal, computed by NavFn planner.**

Published as nav_msgs/Path when a new navigation goal is accepted and a plan is computed. The path is in the map frame. The MPPI controller tracks this path via its PathAlignCritic and PathFollowCritic.

| Property | Value |
|----------|-------|
| **Type** | `nav_msgs/msg/Path` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/planner_server` — RELIABLE / VOLATILE (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/plan_smoothed` {#plan_smoothed}

**Smoothed version of /plan after the SmootherServer processes it.**

NavFn produces grid-quantised paths with sharp corners. The SmootherServer (SimpleSmoother, tolerance=1e-10, max_its=1000) produces a smoother path that reduces aggressive turns. The MPPI controller actually tracks the smoothed plan.

| Property | Value |
|----------|-------|
| **Type** | `nav_msgs/msg/Path` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/smoother_server` — RELIABLE / VOLATILE (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/planner_selector` {#planner_selector}

**String topic for selecting which planner plugin is active at runtime.**

Similar to /controller_selector but for the planner server. Only GridBased (NavFn) is configured on Sigyn.

| Property | Value |
|----------|-------|
| **Type** | `std_msgs/msg/String` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/bt_navigator_navigate_to_pose_rclcpp_node` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/bt_navigator_navigate_through_poses_rclcpp_node` — RELIABLE / TRANSIENT_LOCAL (depth 1)

---

#### `/preempt_teleop` {#preempt_teleop}

**Command to preempt the current teleop session and hand control back to navigation.**

Published by the assisted_teleop behavior or a supervisor node when the teleop session should be ended programmatically.

| Property | Value |
|----------|-------|
| **Type** | `std_msgs/msg/Empty` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/behavior_server` — RELIABLE / VOLATILE (depth 1)

---

#### `/raw_scan` {#raw_scan}

**Unfiltered raw output from the LD14P lidar driver before range/angle filtering.**

The wr_ldlidar driver publishes raw sensor output here. A filter node processes it to produce /scan and /scan_cup. Useful for diagnosing sensor faults or tuning the filter parameters.

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/LaserScan` |
| **Bandwidth** | 290.0 kB/s |
| **Rate** | 9.8 Hz (5.0–11.7) |
| **Avg message size** | 3.7 kB |
| **Delay (end-to-end)** | 46.5 ms (3.7–99.7) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 141 |

**Publishers:**

- `/top_ldlidar` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/scan_to_scan_filter_chain` — BEST_EFFORT / VOLATILE (depth 5)

---

#### `/robot_description` {#robot_description}

**URDF robot model as a string, published once at startup with TRANSIENT_LOCAL.**

Published by robot_state_publisher. Contains the full URDF XML. RViz2 and any node that needs the kinematic model subscribes here. Very large message, published only once.

| Property | Value |
|----------|-------|
| **Type** | `std_msgs/msg/String` |
| **Bandwidth** | 19.26 MB/s |
| **Rate** | 61.4 Hz (18.4–243.5) |
| **Avg message size** | 32.7 kB |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 6 |

**Publishers:**

- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

- `/joint_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/joint_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/joint_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/joint_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/joint_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/joint_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/joint_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)

---

#### `/rosout` {#rosout}

**Aggregated log output from all nodes (ROS 2 equivalent of stdout).**

All RCLCPP_INFO/WARN/ERROR calls in all nodes are published here as rcl_interfaces/msg/Log. The ros2 bag record of /rosout captures the full log stream for post-run analysis.

| Property | Value |
|----------|-------|
| **Type** | `rcl_interfaces/msg/Log` |
| **Bandwidth** | 1.6 kB/s |
| **Rate** | — |
| **Avg message size** | 204 B |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 1 |

**Publishers:**

- `/joint_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/battery_overlay_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/wr_teleop_twist_keyboard` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/scan_to_scan_filter_chain` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/pointcloud_to_laserscan_node` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/transform_listener_impl_5e865b2ef690` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/top_ldlidar` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/joint_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/wr_twist_multiplexer_node` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/teensy_bridge` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/_ros2cli_daemon_0_d9b837226db446f08c14e93c332b5d5a` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/nav2_container` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/map_server` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/amcl` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/lifecycle_manager_localization` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/controller_server` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/smoother_server` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/planner_server` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/global_costmap/global_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/behavior_server` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/bt_navigator` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/waypoint_follower` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/velocity_smoother` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/lifecycle_manager_navigation` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/transform_listener_impl_71275c008a00` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/transform_listener_impl_712754000fa0` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/bt_navigator_navigate_to_pose_rclcpp_node` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/bt_navigator_navigate_through_poses_rclcpp_node` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/launch_ros_4535` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/ekf_filter_node` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/transform_listener_impl_5f8a1e348bd0` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/joint_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/joint_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/topic_analysis_discovery` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/oakd_top/oakd_color_compressed_republisher` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/rqt_gui_py_node_1352476` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/cup_ldlidar` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/oakd_detector` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/joint_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/joint_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/joint_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1000)
- `/sigyn_notifier` — RELIABLE / TRANSIENT_LOCAL (depth 1000)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/scan` {#scan}

**Filtered 2D laser scan from the upper LD14P lidar, used by AMCL and global costmap.**

The primary laser scan topic. Produced by wr_ldlidar after filtering (the raw output is /raw_scan). At 10 Hz with 450 points/scan, this is the main sensor for localisation and global obstacle detection. AMCL's scan_topic=scan parameter links it directly to the localiser.

Sigyn has two lidars: upper (at ~80 cm height, scans the room walls → /scan) and lower cup lidar (at ~30 cm, scans near-ground obstacles → /scan_cup). The upper lidar is NOT used for the local costmap because it sees doorframes as obstacles — it operates at wall height and doorframe tops are within its scan plane.

LD14P characteristics: 360° scan, 10 Hz, ~450 points per revolution, max range 10 m. The /raw_scan output from the driver is passed through a filter node that applies angle masks and range limits to produce /scan.

AMCL integration: AMCL draws max_beams=360 evenly-spaced beams from each scan and updates the particle filter. The likelihood_field model computes the probability of each particle by comparing beam endpoints against the pre-computed map distance field. This is fast (O(n_particles × n_beams)) and accurate in structured environments like a house.

Global costmap: The obstacle_layer subscribes to /scan (not /scan_cup) for global marking because it needs longer range (up to 2.5 m raytrace). The upper lidar is better for this because it is less affected by floor reflections.

Book note: Two lidars at different heights is a common pattern for domestic robots. The upper lidar 'sees' the room structure; the lower lidar 'sees' table legs, chair bases, and other low obstacles. The trade-off is cost and complexity — single-lidar robots are simpler but have blind spots.

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/LaserScan` |
| **Bandwidth** | 289.7 kB/s |
| **Rate** | 9.8 Hz (5.0–11.7) |
| **Avg message size** | 3.7 kB |
| **Delay (end-to-end)** | 46.7 ms (3.7–99.8) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 141 |

**Publishers:**

- `/scan_to_scan_filter_chain` — RELIABLE / VOLATILE (depth 1000)

**Subscribers:**

- `/amcl` — BEST_EFFORT / VOLATILE (depth 5)
- `/global_costmap/global_costmap` — BEST_EFFORT / VOLATILE (depth 50)

---

#### `/scan_cup` {#scan_cup}

**2D laser scan from the lower cup-height LD14P lidar, used by the local costmap.**

The lower lidar at ~30 cm height scans near-ground obstacles (table legs, bags, pets). It is the primary sensor for the local costmap voxel layer. Called 'cup' because it operates at mug-on-floor height.

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/LaserScan` |
| **Bandwidth** | 290.6 kB/s |
| **Rate** | 9.8 Hz (5.0–11.5) |
| **Avg message size** | 3.7 kB |
| **Delay (end-to-end)** | 60.1 ms (3.2–100.8) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 141 |

**Publishers:**

- `/cup_ldlidar` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/local_costmap/local_costmap` — BEST_EFFORT / VOLATILE (depth 50)

---

#### `/set_pose` {#set_pose}

**Alternative pose-reset topic consumed by some Nav2 nodes.**

Some Nav2 nodes accept /set_pose as an alternative to /initialpose. On Sigyn this is wired to the AMCL pose reset path.

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/PoseWithCovarianceStamped` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/ekf_filter_node` — RELIABLE / VOLATILE (depth 1)

---

#### `/speed_limit` {#speed_limit}

**Speed limit command from the costmap speed-restriction filter.**

Published by the costmap filter when the robot enters a speed-restricted zone (defined in a speed-filter costmap YAML). The velocity_smoother respects this limit. Currently not configured on Sigyn but the topic will appear if the filter is enabled.

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/SpeedLimit` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/controller_server` — RELIABLE / VOLATILE (depth 10)

---

#### `/tf` {#tf}

**Dynamic TF2 transform broadcast; aggregates map→odom→base_link and sensor frame transforms.**

The core ROS 2 coordinate transform topic. Multiple nodes publish here: robot_localization (odom→base_footprint), AMCL (map→odom), joint_state_publisher (joint transforms), and wr_ros_teensy (if publish_odom_tf=true). The tf2 library buffers all incoming transforms so any node can query any frame-to-frame transform.

TODO: Add Sigyn-specific frame hierarchy diagram.

| Property | Value |
|----------|-------|
| **Type** | `tf2_msgs/msg/TFMessage` |
| **Bandwidth** | 600.8 kB/s |
| **Rate** | 130.1 Hz (20.0–415.7) |
| **Avg message size** | 577 B |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 1880 |

**Publishers:**

- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 100)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 100)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 100)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 100)
- `/amcl` — RELIABLE / VOLATILE (depth 100)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 100)
- `/ekf_filter_node` — RELIABLE / VOLATILE (depth 100)
- `/robot_state_publisher` — RELIABLE / VOLATILE (depth 100)

**Subscribers:**

- `/transform_listener_impl_5e865b2ef690` — RELIABLE / VOLATILE (depth 100)
- `/amcl` — RELIABLE / VOLATILE (depth 100)
- `/transform_listener_impl_71275c008a00` — RELIABLE / VOLATILE (depth 100)
- `/smoother_server` — RELIABLE / VOLATILE (depth 100)
- `/transform_listener_impl_712754000fa0` — RELIABLE / VOLATILE (depth 100)
- `/behavior_server` — RELIABLE / VOLATILE (depth 100)
- `/bt_navigator` — RELIABLE / VOLATILE (depth 100)
- `/transform_listener_impl_5f8a1e348bd0` — RELIABLE / VOLATILE (depth 100)
- `/oakd_detector` — RELIABLE / VOLATILE (depth 100)

---

#### `/tf_static` {#tf_static}

**Static TF2 transforms (sensor mounts, fixed frames) published once at startup with TRANSIENT_LOCAL.**

Published by robot_state_publisher and other nodes for frames that never change (lidar_link→base_link, camera_link→base_link, etc.). TRANSIENT_LOCAL durability means late-joining nodes receive the full static tree immediately.

| Property | Value |
|----------|-------|
| **Type** | `tf2_msgs/msg/TFMessage` |
| **Bandwidth** | 2.20 MB/s |
| **Rate** | 58.2 Hz (16.8–270.1) |
| **Avg message size** | 3.9 kB |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 6 |

**Publishers:**

- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/robot_state_publisher` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

- `/transform_listener_impl_5e865b2ef690` — RELIABLE / TRANSIENT_LOCAL (depth 100)
- `/amcl` — RELIABLE / TRANSIENT_LOCAL (depth 100)
- `/transform_listener_impl_71275c008a00` — RELIABLE / TRANSIENT_LOCAL (depth 100)
- `/smoother_server` — RELIABLE / TRANSIENT_LOCAL (depth 100)
- `/transform_listener_impl_712754000fa0` — RELIABLE / TRANSIENT_LOCAL (depth 100)
- `/behavior_server` — RELIABLE / TRANSIENT_LOCAL (depth 100)
- `/bt_navigator` — RELIABLE / TRANSIENT_LOCAL (depth 100)
- `/transform_listener_impl_5f8a1e348bd0` — RELIABLE / TRANSIENT_LOCAL (depth 100)
- `/oakd_detector` — RELIABLE / TRANSIENT_LOCAL (depth 100)

---

#### `/trajectories` {#trajectories}

**All MPPI candidate trajectories for visualisation in RViz2.**

When visualize=true in the MPPI controller, it publishes the full batch of 2000 sampled trajectories as a MarkerArray. This is computationally expensive and should only be enabled during tuning sessions.

| Property | Value |
|----------|-------|
| **Type** | `visualization_msgs/msg/MarkerArray` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/controller_server` — RELIABLE / VOLATILE (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/transformed_global_plan` {#transformed_global_plan}

**The portion of the global plan transformed into the local (odom) frame for the controller.**

The controller server takes the global plan (in map frame) and transforms it into the local frame (odom frame) within the current costmap window. This transformed plan is what the MPPI PathAlignCritic and PathFollowCritic actually optimise against.

| Property | Value |
|----------|-------|
| **Type** | `nav_msgs/msg/Path` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/controller_server` — RELIABLE / VOLATILE (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### AMCL Localisation

#### `/amcl/transition_event` {#amcl-transition_event}

**ROS 2 lifecycle state-change event from the AMCL node.**

Published when the AMCL lifecycle node transitions between states (unconfigured → inactive → active → finalized). Useful for diagnostics and for nodes that must wait until AMCL is fully active before sending a navigation goal.

ROS 2 lifecycle nodes publish to a ~/transition_event topic on every state change. For AMCL these events happen at startup (configure then activate) and shutdown. The message is lifecycle_msgs/msg/TransitionEvent.

In a production robot like Sigyn, the lifecycle manager (nav2_lifecycle_manager) orchestrates startup order, so you rarely need to subscribe to this directly. It is most useful in integration tests and in custom orchestrators that need to know exactly when AMCL becomes ready to accept a /initialpose reset. During normal operation this topic fires only a handful of times per session.

| Property | Value |
|----------|-------|
| **Type** | `lifecycle_msgs/msg/TransitionEvent` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/amcl` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Assisted_Teleop

#### `/assisted_teleop/_action/feedback` {#assisted_teleop-_action-feedback}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/action/AssistedTeleop_FeedbackMessage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/behavior_server` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/assisted_teleop/_action/status` {#assisted_teleop-_action-status}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `action_msgs/msg/GoalStatusArray` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/behavior_server` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Backup

#### `/backup/_action/feedback` {#backup-_action-feedback}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/action/BackUp_FeedbackMessage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/behavior_server` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/bt_navigator_navigate_to_pose_rclcpp_node` — RELIABLE / VOLATILE (depth 10)
- `/bt_navigator_navigate_through_poses_rclcpp_node` — RELIABLE / VOLATILE (depth 10)

---

#### `/backup/_action/status` {#backup-_action-status}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `action_msgs/msg/GoalStatusArray` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/behavior_server` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

- `/bt_navigator_navigate_to_pose_rclcpp_node` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/bt_navigator_navigate_through_poses_rclcpp_node` — RELIABLE / TRANSIENT_LOCAL (depth 1)

---

### Behavior Server

#### `/behavior_server/transition_event` {#behavior_server-transition_event}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `lifecycle_msgs/msg/TransitionEvent` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/behavior_server` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### BT Navigator

#### `/bt_navigator/transition_event` {#bt_navigator-transition_event}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `lifecycle_msgs/msg/TransitionEvent` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/bt_navigator` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Compute_Path_Through_Poses

#### `/compute_path_through_poses/_action/feedback` {#compute_path_through_poses-_action-feedback}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/action/ComputePathThroughPoses_FeedbackMessage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/planner_server` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/bt_navigator_navigate_through_poses_rclcpp_node` — RELIABLE / VOLATILE (depth 10)

---

#### `/compute_path_through_poses/_action/status` {#compute_path_through_poses-_action-status}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `action_msgs/msg/GoalStatusArray` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/planner_server` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

- `/bt_navigator_navigate_through_poses_rclcpp_node` — RELIABLE / TRANSIENT_LOCAL (depth 1)

---

### Compute_Path_To_Pose

#### `/compute_path_to_pose/_action/feedback` {#compute_path_to_pose-_action-feedback}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/action/ComputePathToPose_FeedbackMessage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/planner_server` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/bt_navigator_navigate_to_pose_rclcpp_node` — RELIABLE / VOLATILE (depth 10)

---

#### `/compute_path_to_pose/_action/status` {#compute_path_to_pose-_action-status}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `action_msgs/msg/GoalStatusArray` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/planner_server` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

- `/bt_navigator_navigate_to_pose_rclcpp_node` — RELIABLE / TRANSIENT_LOCAL (depth 1)

---

### Controller Server

#### `/controller_server/transition_event` {#controller_server-transition_event}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `lifecycle_msgs/msg/TransitionEvent` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/controller_server` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Drive_On_Heading

#### `/drive_on_heading/_action/feedback` {#drive_on_heading-_action-feedback}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/action/DriveOnHeading_FeedbackMessage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/behavior_server` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/drive_on_heading/_action/status` {#drive_on_heading-_action-status}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `action_msgs/msg/GoalStatusArray` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/behavior_server` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Follow_Gps_Waypoints

#### `/follow_gps_waypoints/_action/feedback` {#follow_gps_waypoints-_action-feedback}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/action/FollowGPSWaypoints_FeedbackMessage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/waypoint_follower` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/follow_gps_waypoints/_action/status` {#follow_gps_waypoints-_action-status}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `action_msgs/msg/GoalStatusArray` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/waypoint_follower` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Follow_Path

#### `/follow_path/_action/feedback` {#follow_path-_action-feedback}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/action/FollowPath_FeedbackMessage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/controller_server` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/bt_navigator_navigate_to_pose_rclcpp_node` — RELIABLE / VOLATILE (depth 10)
- `/bt_navigator_navigate_through_poses_rclcpp_node` — RELIABLE / VOLATILE (depth 10)

---

#### `/follow_path/_action/status` {#follow_path-_action-status}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `action_msgs/msg/GoalStatusArray` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/controller_server` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

- `/bt_navigator_navigate_to_pose_rclcpp_node` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/bt_navigator_navigate_through_poses_rclcpp_node` — RELIABLE / TRANSIENT_LOCAL (depth 1)

---

### Follow_Waypoints

#### `/follow_waypoints/_action/feedback` {#follow_waypoints-_action-feedback}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/action/FollowWaypoints_FeedbackMessage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/waypoint_follower` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/follow_waypoints/_action/status` {#follow_waypoints-_action-status}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `action_msgs/msg/GoalStatusArray` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/waypoint_follower` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Global Costmap

#### `/global_costmap/costmap` {#global_costmap-costmap}

**Full OccupancyGrid representation of the global costmap, published at 1 Hz.**

The global costmap is a static-frame grid (map frame) that combines the static map layer (loaded from YAML), obstacle layer (from /scan), and inflation layer. It is published at publish_frequency=1 Hz as a complete OccupancyGrid message, which at Sigyn's resolution (0.0508 m/pixel) over the full house map is a large message.

The global costmap has two publication channels: /global_costmap/costmap (full OccupancyGrid, 1 Hz) and /global_costmap/costmap_updates (incremental MapMetaData+data updates, 5 Hz). For bandwidth efficiency, subscribers that only need to track changes should use the _updates topic.

Layer architecture (Sigyn global costmap):
  1. static_layer — loads the occupancy grid map file. map_subscribe_transient_local=True so it receives the latched /map message at startup.
  2. obstacle_layer — marks and clears based on /scan laser returns (raytrace up to 3 m).
  3. inflation_layer — inflates obstacles by inflation_radius=0.35 m with cost_scaling_factor=8.0.

The robot_radius for global planning is 0.24 m (slightly smaller than the physical 0.28 m) to allow the planner to route through tight doorways. The local costmap uses 0.28 m for actual collision avoidance.

Relationship to /local_costmap/costmap: The global costmap covers the entire house at coarser temporal resolution (5 Hz update). The local costmap covers a 4×4 m rolling window at 15 Hz. The global costmap drives path planning (NavFn); the local drives real-time collision avoidance (MPPI).

| Property | Value |
|----------|-------|
| **Type** | `nav_msgs/msg/OccupancyGrid` |
| **Bandwidth** | 2.77 MB/s |
| **Rate** | 1.0 Hz (0.8–1.6) |
| **Avg message size** | 337.6 kB |
| **Delay (end-to-end)** | 2.6 ms (2.0–3.1) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 14 |

**Publishers:**

- `/global_costmap/global_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/global_costmap/costmap_raw` {#global_costmap-costmap_raw}

**Raw (un-inflated) global costmap, used by the behavior server for collision checks.**

The behavior server (spin, backup behaviors) uses the raw costmap so it can query the true obstacle cost without inflation bias. Published alongside the inflated version at the same rate.

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/Costmap` |
| **Bandwidth** | 2.77 MB/s |
| **Rate** | 1.0 Hz (0.8–1.6) |
| **Avg message size** | 337.6 kB |
| **Delay (end-to-end)** | 28.7 ms (1.8–368.5) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 14 |

**Publishers:**

- `/global_costmap/global_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

- `/smoother_server` — RELIABLE / TRANSIENT_LOCAL (depth 1)

---

#### `/global_costmap/costmap_raw_updates` {#global_costmap-costmap_raw_updates}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/CostmapUpdate` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/global_costmap/global_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

- `/smoother_server` — RELIABLE / TRANSIENT_LOCAL (depth 10)

---

#### `/global_costmap/costmap_updates` {#global_costmap-costmap_updates}

**Incremental global costmap update patches, published at 5 Hz.**

Instead of re-publishing the entire OccupancyGrid at every update cycle, Nav2 publishes only the changed cells as a nav2_msgs/OccupancyGridUpdate. This is far smaller than the full costmap and is the preferred topic for live visualisation and costmap-aware planning nodes.

| Property | Value |
|----------|-------|
| **Type** | `map_msgs/msg/OccupancyGridUpdate` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/global_costmap/global_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/global_costmap/footprint` {#global_costmap-footprint}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/Polygon` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/global_costmap/global_costmap` — RELIABLE / VOLATILE (depth 1)

---

#### `/global_costmap/obstacle_layer` {#global_costmap-obstacle_layer}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav_msgs/msg/OccupancyGrid` |
| **Bandwidth** | 2.70 MB/s |
| **Rate** | 0.9 Hz (0.8–1.0) |
| **Avg message size** | 337.6 kB |
| **Delay (end-to-end)** | 3.4 ms (2.2–4.3) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 13 |

**Publishers:**

- `/global_costmap/global_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/global_costmap/obstacle_layer_raw` {#global_costmap-obstacle_layer_raw}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/Costmap` |
| **Bandwidth** | 2.70 MB/s |
| **Rate** | 0.9 Hz (0.8–1.0) |
| **Avg message size** | 337.6 kB |
| **Delay (end-to-end)** | 11.0 ms (2.5–102.8) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 13 |

**Publishers:**

- `/global_costmap/global_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/global_costmap/obstacle_layer_raw_updates` {#global_costmap-obstacle_layer_raw_updates}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/CostmapUpdate` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/global_costmap/global_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/global_costmap/obstacle_layer_updates` {#global_costmap-obstacle_layer_updates}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `map_msgs/msg/OccupancyGridUpdate` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/global_costmap/global_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/global_costmap/published_footprint` {#global_costmap-published_footprint}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/PolygonStamped` |
| **Bandwidth** | 8.8 kB/s |
| **Rate** | 5.0 Hz (4.8–5.2) |
| **Avg message size** | 216 B |
| **Delay (end-to-end)** | 28.3 ms (11.9–48.0) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 73 |

**Publishers:**

- `/global_costmap/global_costmap` — RELIABLE / VOLATILE (depth 1)

**Subscribers:**

- `/smoother_server` — RELIABLE / VOLATILE (depth 1)

---

#### `/global_costmap/static_layer` {#global_costmap-static_layer}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav_msgs/msg/OccupancyGrid` |
| **Bandwidth** | 2.70 MB/s |
| **Rate** | 0.9 Hz (0.8–1.0) |
| **Avg message size** | 337.6 kB |
| **Delay (end-to-end)** | 3.3 ms (2.5–4.5) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 13 |

**Publishers:**

- `/global_costmap/global_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/global_costmap/static_layer_raw` {#global_costmap-static_layer_raw}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/Costmap` |
| **Bandwidth** | 2.70 MB/s |
| **Rate** | 0.9 Hz (0.8–1.0) |
| **Avg message size** | 337.6 kB |
| **Delay (end-to-end)** | 3.1 ms (2.2–4.2) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 13 |

**Publishers:**

- `/global_costmap/global_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/global_costmap/static_layer_raw_updates` {#global_costmap-static_layer_raw_updates}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/CostmapUpdate` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/global_costmap/global_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/global_costmap/static_layer_updates` {#global_costmap-static_layer_updates}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `map_msgs/msg/OccupancyGridUpdate` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/global_costmap/global_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Global_Costmap / Global_Costmap

#### `/global_costmap/global_costmap/transition_event` {#global_costmap-global_costmap-transition_event}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `lifecycle_msgs/msg/TransitionEvent` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/global_costmap/global_costmap` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Gripper / Elevator

#### `/gripper/home` {#gripper-home}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `std_msgs/msg/Empty` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

---

#### `/gripper/move_elevator/_action/feedback` {#gripper-move_elevator-_action-feedback}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `sigyn_interfaces/action/MoveElevator_FeedbackMessage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/gripper/move_elevator/_action/status` {#gripper-move_elevator-_action-status}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `action_msgs/msg/GoalStatusArray` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/gripper/move_extender/_action/feedback` {#gripper-move_extender-_action-feedback}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `sigyn_interfaces/action/MoveExtender_FeedbackMessage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/gripper/move_extender/_action/status` {#gripper-move_extender-_action-status}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `action_msgs/msg/GoalStatusArray` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/gripper/position/command` {#gripper-position-command}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `sigyn_interfaces/msg/GripperPositionCommand` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

---

#### `/gripper/status` {#gripper-status}

**sigyn_interfaces/GripperStatus with current elevator/extender position and homing state.**

Published by TopicPublisher at 10 Hz from GRIP3 firmware messages. This is the higher-level status topic (in SI units) vs the raw stepper status. Consumed by the MoveElevator and MoveExtender action servers and by any behavior tree node that needs gripper state.

| Property | Value |
|----------|-------|
| **Type** | `sigyn_interfaces/msg/GripperStatus` |
| **Bandwidth** | 6.3 kB/s |
| **Rate** | 10.0 Hz (9.0–11.3) |
| **Avg message size** | 78 B |
| **Delay (end-to-end)** | 2.9 ms (1.4–13.4) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 144 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Local Costmap

#### `/local_costmap/clearing_endpoints` {#local_costmap-clearing_endpoints}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/PointCloud2` |
| **Bandwidth** | 558.4 kB/s |
| **Rate** | 15.0 Hz (13.2–17.2) |
| **Avg message size** | 4.6 kB |
| **Delay (end-to-end)** | 134.9 ms (66.6–278.4) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 218 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/local_costmap/costmap` {#local_costmap-costmap}

**Full OccupancyGrid of the 4×4 m rolling local costmap, published at 5 Hz.**

The local costmap rolls with the robot in the odom frame. It fuses the voxel layer (low lidar /scan_cup), range sensor layer (8× VL53L0X sensors), and OAK-D top layer (currently disabled). The full grid is published at publish_frequency=5 Hz; incremental updates are published at update_frequency=15 Hz.

The local costmap is the primary real-time collision avoidance surface on Sigyn. Its layer stack:
  1. voxel_layer — subscribes to /scan_cup (lower lidar at ~30 cm height). Voxel resolution 0.2 m × 10 voxels = 2 m height. Clears and marks with raytracing. obstacle_max_range=2.5 m.
  2. oakd_top_layer — disabled (enabled=False) while diagnosing a full-red costmap issue. Will re-enable once OAK-D top camera calibration is verified.
  3. range_sensor_layer — 8 VL53L0X sensors at ±12.5° FOV each. mark_threshold=0.90 (marks within 45 cm), clear_threshold=0.98, clear_on_max_reading=true (out-of-range clears). min_range=0.10 m.
  4. inflation_layer — inflation_radius=0.35 m, cost_scaling_factor=3.5.

Resolution: 0.0508 m/pixel (2 inch grid, matching the house floor grid). Window: 4×4 m = ~6241 cells.

/costmap vs /costmap_updates: The full OccupancyGrid (5 Hz, ~25 kB serialised) is for visualisation. The incremental updates (15 Hz, typically <1 kB) feed the MPPI controller. Subscribe to the updates for performance-critical consumers.

A full-red costmap (every cell lethal) is a known Sigyn diagnostic pattern. Causes: (1) OAK-D top layer publishing bad depth — hence currently disabled; (2) range sensors all reading 0.00 m (hardware noise below min_range=0.10 m); (3) voxel layer receiving a corrupted scan.

| Property | Value |
|----------|-------|
| **Type** | `nav_msgs/msg/OccupancyGrid` |
| **Bandwidth** | 49.5 kB/s |
| **Rate** | — |
| **Avg message size** | 6.2 kB |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 1 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/local_costmap/costmap_raw` {#local_costmap-costmap_raw}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/Costmap` |
| **Bandwidth** | 49.7 kB/s |
| **Rate** | — |
| **Avg message size** | 6.2 kB |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 1 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

- `/behavior_server` — RELIABLE / TRANSIENT_LOCAL (depth 1)

---

#### `/local_costmap/costmap_raw_updates` {#local_costmap-costmap_raw_updates}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/CostmapUpdate` |
| **Bandwidth** | 180.9 kB/s |
| **Rate** | 4.2 Hz (3.5–32.6) |
| **Avg message size** | 5.3 kB |
| **Delay (end-to-end)** | 5.4 ms (1.3–176.1) |
| **Health** | <span style="color:red;font-size:9pt">&#9679;</span> low (4.2 Hz, exp 15.0) |
| **Sample count** | 61 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

- `/behavior_server` — RELIABLE / TRANSIENT_LOCAL (depth 10)

---

#### `/local_costmap/costmap_updates` {#local_costmap-costmap_updates}

**Incremental local costmap patches at 15 Hz; the primary costmap feed for the MPPI controller.**

At update_frequency=15 Hz the local costmap recomputes and publishes only the cells that changed. This is the data stream the MPPI controller actually uses for collision penalty computation via the CostCritic.

| Property | Value |
|----------|-------|
| **Type** | `map_msgs/msg/OccupancyGridUpdate` |
| **Bandwidth** | 178.3 kB/s |
| **Rate** | 4.1 Hz (3.5–5.0) |
| **Avg message size** | 5.3 kB |
| **Delay (end-to-end)** | 2.6 ms (1.4–7.9) |
| **Health** | <span style="color:red;font-size:9pt">&#9679;</span> low (4.1 Hz, exp 15.0) |
| **Sample count** | 60 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/local_costmap/footprint` {#local_costmap-footprint}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/Polygon` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/local_costmap/local_costmap` — RELIABLE / VOLATILE (depth 1)

---

#### `/local_costmap/oakd_top_layer` {#local_costmap-oakd_top_layer}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav_msgs/msg/OccupancyGrid` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/local_costmap/oakd_top_layer_raw` {#local_costmap-oakd_top_layer_raw}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/Costmap` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/local_costmap/oakd_top_layer_raw_updates` {#local_costmap-oakd_top_layer_raw_updates}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/CostmapUpdate` |
| **Bandwidth** | 178.3 kB/s |
| **Rate** | 4.1 Hz (3.5–5.0) |
| **Avg message size** | 5.3 kB |
| **Delay (end-to-end)** | 2.6 ms (1.6–7.9) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 60 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/local_costmap/oakd_top_layer_updates` {#local_costmap-oakd_top_layer_updates}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `map_msgs/msg/OccupancyGridUpdate` |
| **Bandwidth** | 178.3 kB/s |
| **Rate** | 4.1 Hz (3.5–5.0) |
| **Avg message size** | 5.3 kB |
| **Delay (end-to-end)** | 2.7 ms (1.6–7.9) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 60 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/local_costmap/published_footprint` {#local_costmap-published_footprint}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/PolygonStamped` |
| **Bandwidth** | 26.5 kB/s |
| **Rate** | 15.0 Hz (9.2–61.1) |
| **Avg message size** | 220 B |
| **Delay (end-to-end)** | 52.9 ms (7.9–92.3) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 217 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / VOLATILE (depth 1)

**Subscribers:**

- `/behavior_server` — RELIABLE / VOLATILE (depth 1)

---

#### `/local_costmap/range_sensor_layer` {#local_costmap-range_sensor_layer}

**Local costmap layer showing only VL53L0X proximity sensor contributions.**

Nav2 publishes per-layer OccupancyGrids when debug=True. This layer shows which cells were marked or cleared by the 8 VL53L0X time-of-flight sensors. Useful for diagnosing incorrect sensor readings.

Sigyn has 8 VL53L0X sensors arranged around the base: front_left_fwd, front_left_side, front_right_fwd, front_right_side, rear_left_bkwd, rear_left_side, rear_right_bkwd, rear_right_side. Each sensor has a 25° total FOV (±12.5°) and a reliable range of 0.10–0.50 m.

The RangeSensorLayer combines all 8 sensors using combination_method=1 (Max). When a sensor reads ≤0.45 m (mark_threshold=0.90 × 0.50 m), it marks the cell in front of it as occupied. When a sensor reads ≥0.49 m (clear_threshold=0.98) or max_range (out-of-range), it clears the cell. clear_on_max_reading=true means a VL53 reporting out-of-range actively clears the cell, which prevents stale marks when the robot moves away from an obstacle.

Safety integration: The same sensors feed wr_ros_teensy's SafetyCoordinator. When any front sensor reads < ~15 cm (Ring 3 threshold in safety_system.spec.md), a FAULT is raised, degrading or stopping velocity. The costmap and safety system thus respond to the same sensor data through two independent code paths — belt-and-suspenders.

Cross-reference: /local_costmap/voxel_layer shows the lidar contribution; /local_costmap/range_sensor_layer shows the VL53 contribution. Comparing the two helps identify which sensor is causing a spurious obstacle.

| Property | Value |
|----------|-------|
| **Type** | `nav_msgs/msg/OccupancyGrid` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/local_costmap/range_sensor_layer_raw` {#local_costmap-range_sensor_layer_raw}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/Costmap` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/local_costmap/range_sensor_layer_raw_updates` {#local_costmap-range_sensor_layer_raw_updates}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/CostmapUpdate` |
| **Bandwidth** | 178.3 kB/s |
| **Rate** | 4.1 Hz (3.5–5.0) |
| **Avg message size** | 5.3 kB |
| **Delay (end-to-end)** | 2.8 ms (1.8–8.0) |
| **Health** | <span style="color:red;font-size:9pt">&#9679;</span> low (4.1 Hz, exp 15.0) |
| **Sample count** | 60 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/local_costmap/range_sensor_layer_updates` {#local_costmap-range_sensor_layer_updates}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `map_msgs/msg/OccupancyGridUpdate` |
| **Bandwidth** | 178.3 kB/s |
| **Rate** | 4.1 Hz (3.5–5.0) |
| **Avg message size** | 5.3 kB |
| **Delay (end-to-end)** | 2.8 ms (1.7–8.0) |
| **Health** | <span style="color:red;font-size:9pt">&#9679;</span> low (4.1 Hz, exp 15.0) |
| **Sample count** | 60 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/local_costmap/voxel_layer` {#local_costmap-voxel_layer}

**Local costmap layer showing only the low lidar (scan_cup) contributions.**

The voxel layer subscribes to /scan_cup (the lower LD14P lidar at ~30 cm height). It uses 3D voxels (0.2 m resolution × 10 = 2 m height) to mark and clear obstacles via raytracing.

| Property | Value |
|----------|-------|
| **Type** | `nav_msgs/msg/OccupancyGrid` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/local_costmap/voxel_layer_raw` {#local_costmap-voxel_layer_raw}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/Costmap` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/local_costmap/voxel_layer_raw_updates` {#local_costmap-voxel_layer_raw_updates}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/msg/CostmapUpdate` |
| **Bandwidth** | 178.3 kB/s |
| **Rate** | 4.1 Hz (3.5–5.0) |
| **Avg message size** | 5.3 kB |
| **Delay (end-to-end)** | 2.8 ms (1.6–8.1) |
| **Health** | <span style="color:red;font-size:9pt">&#9679;</span> low (4.1 Hz, exp 15.0) |
| **Sample count** | 60 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/local_costmap/voxel_layer_updates` {#local_costmap-voxel_layer_updates}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `map_msgs/msg/OccupancyGridUpdate` |
| **Bandwidth** | 178.3 kB/s |
| **Rate** | 4.1 Hz (3.5–5.0) |
| **Avg message size** | 5.3 kB |
| **Delay (end-to-end)** | 2.9 ms (1.6–8.2) |
| **Health** | <span style="color:red;font-size:9pt">&#9679;</span> low (4.1 Hz, exp 15.0) |
| **Sample count** | 60 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Local_Costmap / Local_Costmap

#### `/local_costmap/local_costmap/transition_event` {#local_costmap-local_costmap-transition_event}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `lifecycle_msgs/msg/TransitionEvent` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/local_costmap/local_costmap` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Map Server

#### `/map_server/transition_event` {#map_server-transition_event}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `lifecycle_msgs/msg/TransitionEvent` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/map_server` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Navigate_Through_Poses

#### `/navigate_through_poses/_action/feedback` {#navigate_through_poses-_action-feedback}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/action/NavigateThroughPoses_FeedbackMessage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/bt_navigator` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/navigate_through_poses/_action/status` {#navigate_through_poses-_action-status}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `action_msgs/msg/GoalStatusArray` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/bt_navigator` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Navigate_To_Pose

#### `/navigate_to_pose/_action/feedback` {#navigate_to_pose-_action-feedback}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/action/NavigateToPose_FeedbackMessage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/bt_navigator` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/bt_navigator` — RELIABLE / VOLATILE (depth 10)
- `/waypoint_follower` — RELIABLE / VOLATILE (depth 10)

---

#### `/navigate_to_pose/_action/status` {#navigate_to_pose-_action-status}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `action_msgs/msg/GoalStatusArray` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/bt_navigator` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

- `/bt_navigator` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/waypoint_follower` — RELIABLE / TRANSIENT_LOCAL (depth 1)

---

### OAK-D Detection

#### `/oakd/annotated_image` {#oakd-annotated_image}

**RGB image with AI detection bounding boxes overlaid, published by sigyn_oakd_detection.**

The OAK-D camera's detections (YOLOv5 or similar) are drawn onto the colour image and published here. Used for operator monitoring and recording evidence of detections.

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/Image` |
| **Bandwidth** | 33.20 MB/s |
| **Rate** | 7.9 Hz (4.0–161.3) |
| **Avg message size** | 519.2 kB |
| **Delay (end-to-end)** | 5.2 ms (1.8–13.8) |
| **Health** | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (7.9 Hz) |
| **Sample count** | 113 |

**Publishers:**

- `/oakd_detector` — BEST_EFFORT / VOLATILE (depth 5)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/oakd/can_detections` {#oakd-can_detections}

**List of detected aluminium cans from the OAK-D front camera, published by sigyn_oakd_detection.**

Custom sigyn_interfaces detection message. Published when the OAK-D neural network identifies a can in the scene. Used by the can_do_challenge and house_patroller nodes to trigger approach behaviors.

| Property | Value |
|----------|-------|
| **Type** | `sigyn_interfaces/msg/OakdDetectionArray` |
| **Bandwidth** | 3.3 kB/s |
| **Rate** | 7.9 Hz (4.0–153.5) |
| **Avg message size** | 52 B |
| **Delay (end-to-end)** | 5.4 ms (1.6–13.3) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 115 |

**Publishers:**

- `/oakd_detector` — BEST_EFFORT / VOLATILE (depth 5)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/oakd/object_detector_heartbeat` {#oakd-object_detector_heartbeat}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `vision_msgs/msg/Detection2DArray` |
| **Bandwidth** | 3.3 kB/s |
| **Rate** | 7.9 Hz (4.0–153.6) |
| **Avg message size** | 52 B |
| **Delay (end-to-end)** | 5.4 ms (1.7–13.3) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 115 |

**Publishers:**

- `/oakd_detector` — BEST_EFFORT / VOLATILE (depth 5)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Oakd / Annotated_Image

#### `/oakd/annotated_image/compressed` {#oakd-annotated_image-compressed}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/CompressedImage` |
| **Bandwidth** | 1.30 MB/s |
| **Rate** | 7.9 Hz (4.0–153.4) |
| **Avg message size** | 20.3 kB |
| **Delay (end-to-end)** | 6.0 ms (2.4–14.0) |
| **Health** | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (7.9 Hz) |
| **Sample count** | 115 |

**Publishers:**

- `/oakd_detector` — BEST_EFFORT / VOLATILE (depth 5)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### OAK-D Top Camera

#### `/oakd_top/camera_info` {#oakd_top-camera_info}

**Calibration parameters for the OAK-D top camera.**

sensor_msgs/CameraInfo published at the camera frame rate. Contains intrinsic matrix K, distortion coefficients D, and frame dimensions. Required by any node that projects pixels to 3D or vice versa.

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/CameraInfo` |
| **Bandwidth** | 24.6 kB/s |
| **Rate** | 8.0 Hz (6.8–10.1) |
| **Avg message size** | 381 B |
| **Delay (end-to-end)** | 6.1 ms (3.9–13.9) |
| **Health** | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (8.0 Hz) |
| **Sample count** | 116 |

**Publishers:**

- `/oakd_detector` — BEST_EFFORT / VOLATILE (depth 5)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/oakd_top/can_point_base` {#oakd_top-can_point_base}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/PointStamped` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/oakd_detector` — BEST_EFFORT / VOLATILE (depth 5)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/oakd_top/can_point_camera` {#oakd_top-can_point_camera}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/PointStamped` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/oakd_detector` — BEST_EFFORT / VOLATILE (depth 5)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/oakd_top/can_point_raw` {#oakd_top-can_point_raw}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/PointStamped` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/oakd_detector` — BEST_EFFORT / VOLATILE (depth 5)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/oakd_top/depth_image` {#oakd_top-depth_image}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/Image` |
| **Bandwidth** | 5.19 MB/s |
| **Rate** | 1.6 Hz (1.5–1.7) |
| **Avg message size** | 388.9 kB |
| **Delay (end-to-end)** | 8.6 ms (2.1–12.9) |
| **Health** | <span style="color:red;font-size:9pt">&#9679;</span> low (1.6 Hz, exp 15.0) |
| **Sample count** | 23 |

**Publishers:**

- `/oakd_detector` — BEST_EFFORT / VOLATILE (depth 5)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/oakd_top/depth_raw` {#oakd_top-depth_raw}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/Image` |
| **Bandwidth** | 258.47 MB/s |
| **Rate** | 7.7 Hz (3.8–9.4) |
| **Avg message size** | 4.15 MB |
| **Delay (end-to-end)** | 8.9 ms (5.8–14.9) |
| **Health** | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (7.7 Hz) |
| **Sample count** | 112 |

**Publishers:**

- `/oakd_detector` — BEST_EFFORT / VOLATILE (depth 5)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/oakd_top/depth_sample_base` {#oakd_top-depth_sample_base}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/PointStamped` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/oakd_detector` — BEST_EFFORT / VOLATILE (depth 5)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/oakd_top/depth_sample_camera` {#oakd_top-depth_sample_camera}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `geometry_msgs/msg/PointStamped` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/oakd_detector` — BEST_EFFORT / VOLATILE (depth 5)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/oakd_top/out` {#oakd_top-out}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/Image` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/oakd_top/oakd_color_compressed_republisher` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/oakd_top/points` {#oakd_top-points}

**Dense stereo point cloud from the OAK-D top camera, in the camera's depth frame.**

sensor_msgs/PointCloud2. Used by the local costmap oakd_top_layer (currently disabled). Point cloud density is high (~300k points at VGA resolution) making this one of the highest-bandwidth topics on Sigyn.

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/PointCloud2` |
| **Bandwidth** | 91.92 MB/s |
| **Rate** | 7.4 Hz (3.7–9.9) |
| **Avg message size** | 1.54 MB |
| **Delay (end-to-end)** | 14.2 ms (6.1–20.6) |
| **Health** | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (7.4 Hz) |
| **Sample count** | 107 |

**Publishers:**

- `/oakd_detector` — BEST_EFFORT / VOLATILE (depth 5)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/oakd_top/rgb_preview` {#oakd_top-rgb_preview}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/Image` |
| **Bandwidth** | 33.48 MB/s |
| **Rate** | 8.0 Hz (4.1–109.4) |
| **Avg message size** | 519.2 kB |
| **Delay (end-to-end)** | 3.0 ms (1.7–8.7) |
| **Health** | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (8.0 Hz) |
| **Sample count** | 116 |

**Publishers:**

- `/oakd_detector` — BEST_EFFORT / VOLATILE (depth 5)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Oakd_Top / Color

#### `/oakd_top/color/image/compressed` {#oakd_top-color-image-compressed}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/CompressedImage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/oakd_top/oakd_color_compressed_republisher` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Oakd_Top / Out

#### `/oakd_top/out/compressedDepth` {#oakd_top-out-compressedDepth}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/CompressedImage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/oakd_top/oakd_color_compressed_republisher` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/oakd_top/out/theora` {#oakd_top-out-theora}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `theora_image_transport/msg/Packet` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/oakd_top/oakd_color_compressed_republisher` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/oakd_top/out/zstd` {#oakd_top-out-zstd}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/CompressedImage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/oakd_top/oakd_color_compressed_republisher` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Planner Server

#### `/planner_server/transition_event` {#planner_server-transition_event}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `lifecycle_msgs/msg/TransitionEvent` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/planner_server` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Sigyn (Other)

#### `/sigyn/take_oakd_picture` {#sigyn-take_oakd_picture}

**Command topic to trigger a still image capture from the OAK-D camera.**

std_msgs/Empty or similar trigger. Published by house_patroller or operator commands when the robot should photograph a scene (e.g., to document a detected hazard or for the human-alert evidence image).

| Property | Value |
|----------|-------|
| **Type** | `std_msgs/msg/Bool` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/oakd_detector` — RELIABLE / VOLATILE (depth 10)

---

### Power

#### `/sigyn/power/battery` {#sigyn-power-battery}

**Battery state (voltage, current, SoC, temperature) from Teensy Board 2.**

Published as sensor_msgs/BatteryState at ~1 Hz. Feeds battery_overlay_text for display and can trigger human alerts via the fault notification system when voltage drops below thresholds.

TODO: Add battery chemistry (LiFePO4?), cell count, voltage thresholds, charging integration notes.

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/BatteryState` |
| **Bandwidth** | 659 B/s |
| **Rate** | 1.0 Hz (1.0–1.0) |
| **Avg message size** | 77 B |
| **Delay (end-to-end)** | 2.8 ms (1.3–11.1) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 15 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/sigyn_notifier` — RELIABLE / VOLATILE (depth 10)

---

#### `/sigyn/power/rail` {#sigyn-power-rail}

**Status of individual power rail voltages (5 V, 12 V, 24 V, etc.) from Teensy Board 2.**

wr_interfaces/msg/PowerRailStatus message containing per-rail voltage and current. Used by diagnostics and by the safety system to detect under-voltage conditions on motor or PC rails.

| Property | Value |
|----------|-------|
| **Type** | `wr_interfaces/msg/PowerRailStatus` |
| **Bandwidth** | 1.0 kB/s |
| **Rate** | 3.1 Hz (1.0–589.8) |
| **Avg message size** | 40 B |
| **Delay (end-to-end)** | 5.8 ms (1.7–18.1) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 45 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Sigyn / Roboclaw

#### `/sigyn/roboclaw/status` {#sigyn-roboclaw-status}

**RoboClaw motor controller status: encoder counts, speed, current, temperature, errors.**

Published by wr_ros_teensy at ~5 Hz (RCLW firmware message). Contains raw M1/M2 encoder counts, measured velocities, motor currents, board temperature, and error register. Essential for diagnosing drive problems.

TODO: Add RoboClaw error bit meanings, relationship to odometry quality.

| Property | Value |
|----------|-------|
| **Type** | `wr_interfaces/msg/RoboClawStatus` |
| **Bandwidth** | 3.0 kB/s |
| **Rate** | 5.0 Hz (4.8–5.2) |
| **Avg message size** | 73 B |
| **Delay (end-to-end)** | 2.5 ms (1.3–10.7) |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 73 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Safety System

#### `/sigyn/safety/fault_events` {#sigyn-safety-fault_events}

**Edge-triggered fault event: published each time a new fault is raised or cleared.**

Unlike /sigyn/safety/fault_list (which is a periodic snapshot), /sigyn/safety/fault_events is published only when a fault transitions in or out. Used by sigyn_notifier to decide when to send a Telegram message.

| Property | Value |
|----------|-------|
| **Type** | `wr_interfaces/msg/FaultEvent` |
| **Bandwidth** | 1.0 kB/s |
| **Rate** | 1.7 Hz (0.3–10.1) |
| **Avg message size** | 67 B |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 11 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 50)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/sigyn/safety/fault_list` {#sigyn-safety-fault_list}

**List of all currently active safety faults, published at ≥10 Hz.**

wr_interfaces/msg/ActiveFaultList containing every fault currently in FaultRegistry. Useful for dashboards that need to show all active issues, not just the aggregate severity.

| Property | Value |
|----------|-------|
| **Type** | `wr_interfaces/msg/ActiveFaultList` |
| **Bandwidth** | 2.1 kB/s |
| **Rate** | 10.0 Hz (9.2–11.7) |
| **Avg message size** | 26 B |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 145 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/sigyn_notifier` — RELIABLE / VOLATILE (depth 10)

---

#### `/sigyn/safety/human_alert` {#sigyn-safety-human_alert}

**Human notification request published when a fault requires human awareness or intervention.**

wr_interfaces/msg/HumanAlert carries fault ID, board, instance, tier, and message text. sigyn_notifier subscribes and dispatches to Telegram. Latching faults require a human to call the /sigyn/safety/human_override service to clear.

TODO: Link to human_interaction.spec.md; describe Telegram tiers 1/2/3.

| Property | Value |
|----------|-------|
| **Type** | `wr_interfaces/msg/HumanAlert` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 20)

**Subscribers:**

- `/sigyn_notifier` — RELIABLE / VOLATILE (depth 10)

---

#### `/sigyn/safety/state` {#sigyn-safety-state}

**Aggregated robot safety state (NORMAL/DEGRADED/CRITICAL/EMERGENCY_STOP), published at ≥10 Hz.**

Published by SafetyCoordinator. The state is derived from all active faults in FaultRegistry using the authority chain: NORMAL < DEGRADED < CRITICAL < EMERGENCY_STOP. CmdVelController subscribes to this topic to apply velocity limits.

The safety state is the single most important output of the wr_ros_teensy safety subsystem. Its consumers:
  - CmdVelController: scales velocity by 1.0 (NORMAL), 0.5 (DEGRADED), or 0.0 (CRITICAL/ESTOP)
  - sigyn_notifier: triggers Telegram alerts at tier 1/2/3 depending on state
  - Behavior trees: can check this topic to decide whether to abort a goal

Publish rate: The SafetyCoordinator fires an eval_timer_ at 100 ms (10 Hz). Every cycle it calls InternalEvaluateAndPublish(), which:
  1. Asks FaultRegistry for the current aggregate severity
  2. Converts to NORMAL/DEGRADED/CRITICAL/ESTOP
  3. Publishes even if unchanged (so subscribers always get fresh data)

Related topics: /sigyn/safety/fault_list (active faults), /sigyn/safety/fault_events (edge-triggered new faults), /sigyn/safety/human_alert (human notification requests).

Design note: Publishing unconditionally at 10 Hz (not edge-triggered) is intentional. A subscriber that receives no message for >200 ms knows the safety coordinator is dead and should treat this as ESTOP. This 'heartbeat-as-safety' pattern is explained in detail in safety_system.spec.md.

| Property | Value |
|----------|-------|
| **Type** | `wr_interfaces/msg/SafetyState` |
| **Bandwidth** | 2.3 kB/s |
| **Rate** | 10.0 Hz (9.2–11.7) |
| **Avg message size** | 28 B |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 145 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Sigyn Sensors

#### `/sigyn/sensors/imu_front_right` {#sigyn-sensors-imu_front_right}

**BNO055 9-DOF IMU on the front-right corner of Sigyn's base, published at 100 Hz.**

Published by wr_ros_teensy from Board 2 sensor data. Provides orientation (quaternion), angular velocity, and linear acceleration. Used by robot_localization EKF and by the SafetyCoordinator (IMU_TILTED/IMU_FALLEN faults).

Sigyn has two BNO055 IMUs: front_right (Board 2, primary for EKF) and rear_left (secondary, used for cross-validation). The BNO055 integrates a 3-axis accelerometer, gyroscope, and magnetometer with an onboard fusion processor.

Specification: 100 Hz publish rate; heading accuracy ±1° (calibrated), ±2–3° typical; angular velocity noise ~0.005 rad/s; linear acceleration noise ~0.03 m/s².

Safety faults:
  - IMU_TILTED (WARNING): roll or pitch ≥10° and <20°. Velocity is not stopped but the operator is alerted. Auto-clears when tilt returns to normal.
  - IMU_FALLEN (EMERGENCY_STOP, latching): roll or pitch ≥20°. Immediately asserts hardware e-stop via Teensy Board 2. Requires human acknowledgment to clear.

Calibration: The BNO055 must be periodically recalibrated using the figure-8 motion procedure. Uncalibrated sensors show ±2–3° heading error which causes AMCL convergence problems. The calibration status is reported in /diagnostics.

Book note: The BNO055 is a popular choice for hobby robots because it handles sensor fusion internally, reducing PC-side computation. The trade-off is that the fusion algorithm is a black box — you cannot tune the Kalman filter weights. For research robots, a raw MPU-9250 + robot_localization is more controllable.

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/Imu` |
| **Bandwidth** | 78.9 kB/s |
| **Rate** | 28.9 Hz (20.7–52.3) |
| **Avg message size** | 340 B |
| **Delay (end-to-end)** | 2.9 ms (1.3–15.0) |
| **Health** | <span style="color:red;font-size:9pt">&#9679;</span> low (28.9 Hz, exp 100.0) |
| **Sample count** | 420 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/sigyn/sensors/imu_rear_left` {#sigyn-sensors-imu_rear_left}

**BNO055 9-DOF IMU on the rear-left corner of Sigyn's base, published at 100 Hz.**

Secondary IMU used for cross-validation with imu_front_right and for detecting asymmetric tilt (one corner of the base lifting). Also contributes to the EKF fusion when configured.

TODO: Same architecture as front_right; add cross-validation logic notes once implemented.

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/Imu` |
| **Bandwidth** | 77.1 kB/s |
| **Rate** | 28.9 Hz (20.7–62.3) |
| **Avg message size** | 332 B |
| **Delay (end-to-end)** | 3.4 ms (1.3–18.0) |
| **Health** | <span style="color:red;font-size:9pt">&#9679;</span> low (28.9 Hz, exp 100.0) |
| **Sample count** | 420 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/sigyn/sensors/odom` {#sigyn-sensors-odom}

**Raw wheel-encoder odometry from the RoboClaw motor controller, published at 50 Hz.**

Published by wr_ros_teensy's TopicPublisher when it receives ODOM firmware messages from Board 1. Contains pose and twist derived from RoboClaw encoder counts. Covariance values are conservative estimates; the EKF on /odom is the preferred source for navigation.

The RoboClaw motor controller on Sigyn's drive base sends encoder counts to Teensy Board 1, which computes differential-drive odometry at ~50 Hz and sends an ODOM wire message to the PC. TopicPublisher converts this to a nav_msgs/Odometry message.

Kinematic model: Sigyn uses a differential-drive model (two independently driven wheels). Odometry is computed from left and right encoder deltas: Δx = (ΔL+ΔR)/2 × cos(θ), Δy = (ΔL+ΔR)/2 × sin(θ), Δθ = (ΔR-ΔL)/wheelbase.

Known limitations: (1) Wheel slip on carpet or when turning sharply degrades accuracy. (2) The covariance matrix in the raw message uses conservative fixed values; the EKF updates these dynamically. (3) At 50 Hz the position update lag is ~20 ms — acceptable for Nav2 but visible in slow-motion analysis.

See also: /sigyn/roboclaw/status for motor current, temperature, and error flags that give context for odometry quality.

| Property | Value |
|----------|-------|
| **Type** | `nav_msgs/msg/Odometry` |
| **Bandwidth** | 187.3 kB/s |
| **Rate** | 32.3 Hz (21.1–65.9) |
| **Avg message size** | 724 B |
| **Delay (end-to-end)** | 2.8 ms (1.3–17.1) |
| **Health** | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (32.3 Hz) |
| **Sample count** | 468 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/ekf_filter_node` — BEST_EFFORT / VOLATILE (depth 10)

---

#### `/sigyn/sensors/range/front_left_fwd` {#sigyn-sensors-range-front_left_fwd}

**VL53L0X time-of-flight range reading from the front-left forward-facing sensor.**

Published by wr_ros_teensy from Teensy Board 2 at ~50 Hz. Range is in metres; max reliable range is 0.5 m for the local costmap (hardware can read up to ~2 m but accuracy degrades). Used by both the local costmap RangeSensorLayer and the safety system.

Sigyn has 8 VL53L0X sensors arranged at the four corners of the base: two at each corner, one pointing forward/backward along the robot axis and one pointing sideways. The naming convention is <front|rear>_<left|right>_<fwd|bkwd|side>.

VL53L0X technology: Time-of-flight laser ranging at 940 nm. Unlike IR distance sensors, it is not fooled by surface colour. FOV is ±12.5° (25° cone). Range: 3 cm to 2 m (best accuracy 10 cm–1.2 m). At distances below 10 cm, the hardware may return 0.000 m (hardware noise), which is why min_range=0.10 m in the RangeSensorLayer.

Local costmap role: The RangeSensorLayer uses all 8 sensors to mark obstacles within ~45 cm and clear cells when readings are out-of-range. The 50 Hz rate is much faster than the costmap update cycle (15 Hz), so the costmap always has fresh readings.

Safety role: SafetyCoordinator monitors these topics. Ring 3 (closest ring): any sensor < threshold_ring3 (typically 15 cm) raises a Ring 3 fault → velocity degraded to 50%. Ring 2: any sensor < threshold_ring2 (typically 8 cm) raises ESTOP. This means the robot can stop itself even if the costmap is temporarily stale — the range sensors provide a direct, low-latency safety check.

Diagnostic pattern: If the local costmap shows a persistent obstacle on one side that does not clear when the robot moves away, check whether the corresponding range sensor is stuck reporting a low value. Compare /sigyn/sensors/range/<name> against the costmap's range_sensor_layer debug topic.

Book note: Time-of-flight sensors are increasingly common in consumer devices (phone face unlock, robot vacuums). For a mobile robot assistant operating near people, their fast response time (< 5 ms per reading) makes them superior to ultrasonic sensors for close-range safety.

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/Range` |
| **Bandwidth** | 12.4 kB/s |
| **Rate** | 25.8 Hz (15.4–181.3) |
| **Avg message size** | 60 B |
| **Delay (end-to-end)** | 2.7 ms (1.3–18.2) |
| **Health** | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (25.8 Hz) |
| **Sample count** | 373 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/local_costmap/local_costmap` — BEST_EFFORT / VOLATILE (depth 5)
- `/rqt_gui_py_node_1352476` — RELIABLE / VOLATILE (depth 10)

---

#### `/sigyn/sensors/range/front_left_side` {#sigyn-sensors-range-front_left_side}

**VL53L0X range from the front-left side-facing sensor.**

Same architecture as front_left_fwd but pointing laterally left from the front-left corner. Detects obstacles alongside the robot body, preventing side collisions during navigation in narrow corridors.

TODO: Same architecture as front_left_fwd.

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/Range` |
| **Bandwidth** | 12.9 kB/s |
| **Rate** | 25.2 Hz (15.4–100.1) |
| **Avg message size** | 64 B |
| **Delay (end-to-end)** | 2.9 ms (1.3–15.4) |
| **Health** | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (25.2 Hz) |
| **Sample count** | 365 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/local_costmap/local_costmap` — BEST_EFFORT / VOLATILE (depth 5)

---

#### `/sigyn/sensors/range/front_right_fwd` {#sigyn-sensors-range-front_right_fwd}

**VL53L0X range from the front-right forward-facing sensor.**

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/Range` |
| **Bandwidth** | 13.2 kB/s |
| **Rate** | 25.7 Hz (9.9–85.0) |
| **Avg message size** | 64 B |
| **Delay (end-to-end)** | 2.8 ms (1.2–13.4) |
| **Health** | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (25.7 Hz) |
| **Sample count** | 372 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/local_costmap/local_costmap` — BEST_EFFORT / VOLATILE (depth 5)

---

#### `/sigyn/sensors/range/front_right_side` {#sigyn-sensors-range-front_right_side}

**VL53L0X range from the front-right side-facing sensor.**

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/Range` |
| **Bandwidth** | 13.0 kB/s |
| **Rate** | 25.4 Hz (14.5–80.2) |
| **Avg message size** | 64 B |
| **Delay (end-to-end)** | 2.8 ms (1.3–15.4) |
| **Health** | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (25.4 Hz) |
| **Sample count** | 368 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/local_costmap/local_costmap` — BEST_EFFORT / VOLATILE (depth 5)

---

#### `/sigyn/sensors/range/rear_left_bkwd` {#sigyn-sensors-range-rear_left_bkwd}

**VL53L0X range from the rear-left backward-facing sensor.**

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/Range` |
| **Bandwidth** | 12.4 kB/s |
| **Rate** | 25.7 Hz (18.7–39.7) |
| **Avg message size** | 60 B |
| **Delay (end-to-end)** | 2.8 ms (1.3–16.8) |
| **Health** | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (25.7 Hz) |
| **Sample count** | 372 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/local_costmap/local_costmap` — BEST_EFFORT / VOLATILE (depth 5)

---

#### `/sigyn/sensors/range/rear_left_side` {#sigyn-sensors-range-rear_left_side}

**VL53L0X range from the rear-left side-facing sensor.**

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/Range` |
| **Bandwidth** | 12.2 kB/s |
| **Rate** | 25.4 Hz (15.2–70.4) |
| **Avg message size** | 60 B |
| **Delay (end-to-end)** | 2.9 ms (1.2–15.4) |
| **Health** | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (25.4 Hz) |
| **Sample count** | 368 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/local_costmap/local_costmap` — BEST_EFFORT / VOLATILE (depth 5)

---

#### `/sigyn/sensors/range/rear_right_bkwd` {#sigyn-sensors-range-rear_right_bkwd}

**VL53L0X range from the rear-right backward-facing sensor.**

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/Range` |
| **Bandwidth** | 13.1 kB/s |
| **Rate** | 25.5 Hz (17.9–44.8) |
| **Avg message size** | 64 B |
| **Delay (end-to-end)** | 2.8 ms (1.2–16.6) |
| **Health** | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (25.5 Hz) |
| **Sample count** | 369 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/local_costmap/local_costmap` — BEST_EFFORT / VOLATILE (depth 5)

---

#### `/sigyn/sensors/range/rear_right_side` {#sigyn-sensors-range-rear_right_side}

**VL53L0X range from the rear-right side-facing sensor.**

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/Range` |
| **Bandwidth** | 13.0 kB/s |
| **Rate** | 25.2 Hz (18.8–37.6) |
| **Avg message size** | 64 B |
| **Delay (end-to-end)** | 2.8 ms (1.2–14.9) |
| **Health** | <span style="color:#bb8800;font-size:9pt">&#9679;</span> low (25.2 Hz) |
| **Sample count** | 366 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/local_costmap/local_costmap` — BEST_EFFORT / VOLATILE (depth 5)

---

### Sigyn / Stepper

#### `/sigyn/stepper/status` {#sigyn-stepper-status}

**Elevator and extender stepper motor status (position, speed, homed) at 10 Hz.**

Published by TopicPublisher from Board 3 GRIP3 wire messages. Contains elevator position (m), extender position (m), speed, and homed flags for both axes. Consumed by ElevatorPositionController to track goal completion.

TODO: Add mechanical range, homing sequence description, relationship to MoveElevator action.

| Property | Value |
|----------|-------|
| **Type** | `wr_interfaces/msg/StepperStatus` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Sigyn / Teensy_Bridge

#### `/sigyn/teensy_bridge/battery/status` {#sigyn-teensy_bridge-battery-status}

**Low-rate battery status string published by the teensy_bridge for dashboard display.**

A simplified battery status complementing /sigyn/power/battery. Published at ~0.5 Hz as a human-readable string for display in terminals and simple dashboards.

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/BatteryState` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | (no publisher) |
| **Sample count** | 0 |

**Publishers:**

<span style="color:orange">**NONE**</span>

**Subscribers:**

- `/battery_overlay_publisher` — RELIABLE / VOLATILE (depth 10)

---

### Smooth_Path

#### `/smooth_path/_action/feedback` {#smooth_path-_action-feedback}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/action/SmoothPath_FeedbackMessage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/smoother_server` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/smooth_path/_action/status` {#smooth_path-_action-status}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `action_msgs/msg/GoalStatusArray` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/smoother_server` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Smoother Server

#### `/smoother_server/transition_event` {#smoother_server-transition_event}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `lifecycle_msgs/msg/TransitionEvent` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/smoother_server` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Spin

#### `/spin/_action/feedback` {#spin-_action-feedback}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/action/Spin_FeedbackMessage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/behavior_server` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/bt_navigator_navigate_to_pose_rclcpp_node` — RELIABLE / VOLATILE (depth 10)
- `/bt_navigator_navigate_through_poses_rclcpp_node` — RELIABLE / VOLATILE (depth 10)

---

#### `/spin/_action/status` {#spin-_action-status}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `action_msgs/msg/GoalStatusArray` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/behavior_server` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

- `/bt_navigator_navigate_to_pose_rclcpp_node` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/bt_navigator_navigate_through_poses_rclcpp_node` — RELIABLE / TRANSIENT_LOCAL (depth 1)

---

### Stereo

#### `/stereo/points2` {#stereo-points2}

**Stereo point cloud from the OAK-D camera used as a 3D obstacle source.**

This topic feeds the global costmap oakd_left_obstacle and oakd_right_obstacle layers (currently commented out in navigation.yaml). When enabled it provides 3D obstacle detection beyond the lidar range.

| Property | Value |
|----------|-------|
| **Type** | `sensor_msgs/msg/LaserScan` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/pointcloud_to_laserscan_node` — BEST_EFFORT / VOLATILE (depth 5)

**Subscribers:**

- `/local_costmap/local_costmap` — BEST_EFFORT / VOLATILE (depth 50)

---

### Teensy Bridge

#### `/teensy_bridge/status` {#teensy_bridge-status}

**Human-readable status string from the teensy_bridge node, published every 500 ms.**

std_msgs/String published by the LifecycleNode timer at 2 Hz. Contains connection status, board IDs, protocol version, and fault summary. Useful for operator dashboards and system health checks.

| Property | Value |
|----------|-------|
| **Type** | `std_msgs/msg/String` |
| **Bandwidth** | 305 B/s |
| **Rate** | 2.0 Hz (2.0–5.8) |
| **Avg message size** | 18 B |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:green;font-size:9pt">&#9679;</span> OK |
| **Sample count** | 30 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

#### `/teensy_bridge/transition_event` {#teensy_bridge-transition_event}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `lifecycle_msgs/msg/TransitionEvent` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/teensy_bridge` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/launch_ros_4535` — RELIABLE / VOLATILE (depth 10)

---

### Velocity Smoother

#### `/velocity_smoother/transition_event` {#velocity_smoother-transition_event}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `lifecycle_msgs/msg/TransitionEvent` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/velocity_smoother` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

### Wait

#### `/wait/_action/feedback` {#wait-_action-feedback}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `nav2_msgs/action/Wait_FeedbackMessage` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/behavior_server` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

- `/bt_navigator_navigate_to_pose_rclcpp_node` — RELIABLE / VOLATILE (depth 10)
- `/bt_navigator_navigate_through_poses_rclcpp_node` — RELIABLE / VOLATILE (depth 10)

---

#### `/wait/_action/status` {#wait-_action-status}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `action_msgs/msg/GoalStatusArray` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/behavior_server` — RELIABLE / TRANSIENT_LOCAL (depth 1)

**Subscribers:**

- `/bt_navigator_navigate_to_pose_rclcpp_node` — RELIABLE / TRANSIENT_LOCAL (depth 1)
- `/bt_navigator_navigate_through_poses_rclcpp_node` — RELIABLE / TRANSIENT_LOCAL (depth 1)

---

### Waypoint Follower

#### `/waypoint_follower/transition_event` {#waypoint_follower-transition_event}

**TODO – no description written yet**

| Property | Value |
|----------|-------|
| **Type** | `lifecycle_msgs/msg/TransitionEvent` |
| **Bandwidth** | — |
| **Rate** | — |
| **Avg message size** | — |
| **Delay (end-to-end)** | N/A |
| **Health** | <span style="color:orange;font-size:9pt">&#9679;</span> silent |
| **Sample count** | 0 |

**Publishers:**

- `/waypoint_follower` — RELIABLE / VOLATILE (depth 10)

**Subscribers:**

<span style="color:orange">**NONE**</span>

---

## Silent Topics

These topics have **at least one registered publisher** but no message was observed during the 15.0-second sampling window. They may be event-driven (low-rate), awaiting a trigger (e.g., a navigation goal), or indicate a misconfigured node.

| Topic | Type | Publishers | Subscribers |
|-------|------|-----------|------------|
| `/amcl/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | `/amcl` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/assisted_teleop/_action/feedback` | `nav2_msgs/action/AssistedTeleop_FeedbackMessage` | `/behavior_server` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/assisted_teleop/_action/status` | `action_msgs/msg/GoalStatusArray` | `/behavior_server` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/backup/_action/feedback` | `nav2_msgs/action/BackUp_FeedbackMessage` | `/behavior_server` RELIABLE/VOLATILE | `/bt_navigator_navigate_to_pose_rclcpp_node` RELIABLE/VOLATILE<br>`/bt_navigator_navigate_through_poses_rclcpp_node` RELIABLE/VOLATILE |
| `/backup/_action/status` | `action_msgs/msg/GoalStatusArray` | `/behavior_server` RELIABLE/TRANSIENT_LOCAL | `/bt_navigator_navigate_to_pose_rclcpp_node` RELIABLE/TRANSIENT_LOCAL<br>`/bt_navigator_navigate_through_poses_rclcpp_node` RELIABLE/TRANSIENT_LOCAL |
| `/battery_overlay_text` | `rviz_2d_overlay_msgs/msg/OverlayText` | `/battery_overlay_publisher` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/behavior_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | `/behavior_server` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/behavior_tree_log` | `nav2_msgs/msg/BehaviorTreeLog` | `/bt_navigator_navigate_to_pose_rclcpp_node` RELIABLE/VOLATILE<br>`/bt_navigator_navigate_through_poses_rclcpp_node` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/bt_navigator/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | `/bt_navigator` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `/wr_teleop_twist_keyboard` RELIABLE/VOLATILE<br>`/wr_twist_multiplexer_node` RELIABLE/VOLATILE | `/teensy_bridge` RELIABLE/VOLATILE |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | `/controller_server` RELIABLE/VOLATILE<br>`/behavior_server` RELIABLE/VOLATILE<br>`/behavior_server` RELIABLE/VOLATILE<br>`/behavior_server` RELIABLE/VOLATILE<br>`/behavior_server` RELIABLE/VOLATILE<br>`/behavior_server` RELIABLE/VOLATILE | `/wr_twist_multiplexer_node` RELIABLE/VOLATILE<br>`/velocity_smoother` RELIABLE/VOLATILE |
| `/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | `/velocity_smoother` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/compute_path_through_poses/_action/feedback` | `nav2_msgs/action/ComputePathThroughPoses_FeedbackMessage` | `/planner_server` RELIABLE/VOLATILE | `/bt_navigator_navigate_through_poses_rclcpp_node` RELIABLE/VOLATILE |
| `/compute_path_through_poses/_action/status` | `action_msgs/msg/GoalStatusArray` | `/planner_server` RELIABLE/TRANSIENT_LOCAL | `/bt_navigator_navigate_through_poses_rclcpp_node` RELIABLE/TRANSIENT_LOCAL |
| `/compute_path_to_pose/_action/feedback` | `nav2_msgs/action/ComputePathToPose_FeedbackMessage` | `/planner_server` RELIABLE/VOLATILE | `/bt_navigator_navigate_to_pose_rclcpp_node` RELIABLE/VOLATILE |
| `/compute_path_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` | `/planner_server` RELIABLE/TRANSIENT_LOCAL | `/bt_navigator_navigate_to_pose_rclcpp_node` RELIABLE/TRANSIENT_LOCAL |
| `/controller_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | `/controller_server` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/drive_on_heading/_action/feedback` | `nav2_msgs/action/DriveOnHeading_FeedbackMessage` | `/behavior_server` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/drive_on_heading/_action/status` | `action_msgs/msg/GoalStatusArray` | `/behavior_server` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/follow_gps_waypoints/_action/feedback` | `nav2_msgs/action/FollowGPSWaypoints_FeedbackMessage` | `/waypoint_follower` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/follow_gps_waypoints/_action/status` | `action_msgs/msg/GoalStatusArray` | `/waypoint_follower` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/follow_path/_action/feedback` | `nav2_msgs/action/FollowPath_FeedbackMessage` | `/controller_server` RELIABLE/VOLATILE | `/bt_navigator_navigate_to_pose_rclcpp_node` RELIABLE/VOLATILE<br>`/bt_navigator_navigate_through_poses_rclcpp_node` RELIABLE/VOLATILE |
| `/follow_path/_action/status` | `action_msgs/msg/GoalStatusArray` | `/controller_server` RELIABLE/TRANSIENT_LOCAL | `/bt_navigator_navigate_to_pose_rclcpp_node` RELIABLE/TRANSIENT_LOCAL<br>`/bt_navigator_navigate_through_poses_rclcpp_node` RELIABLE/TRANSIENT_LOCAL |
| `/follow_waypoints/_action/feedback` | `nav2_msgs/action/FollowWaypoints_FeedbackMessage` | `/waypoint_follower` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/follow_waypoints/_action/status` | `action_msgs/msg/GoalStatusArray` | `/waypoint_follower` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/global_costmap/costmap_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | `/global_costmap/global_costmap` RELIABLE/TRANSIENT_LOCAL | `/smoother_server` RELIABLE/TRANSIENT_LOCAL |
| `/global_costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | `/global_costmap/global_costmap` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/global_costmap/global_costmap/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | `/global_costmap/global_costmap` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/global_costmap/obstacle_layer_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | `/global_costmap/global_costmap` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/global_costmap/obstacle_layer_updates` | `map_msgs/msg/OccupancyGridUpdate` | `/global_costmap/global_costmap` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/global_costmap/static_layer_raw_updates` | `nav2_msgs/msg/CostmapUpdate` | `/global_costmap/global_costmap` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/global_costmap/static_layer_updates` | `map_msgs/msg/OccupancyGridUpdate` | `/global_costmap/global_costmap` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/gripper/move_elevator/_action/feedback` | `sigyn_interfaces/action/MoveElevator_FeedbackMessage` | `/teensy_bridge` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/gripper/move_elevator/_action/status` | `action_msgs/msg/GoalStatusArray` | `/teensy_bridge` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/gripper/move_extender/_action/feedback` | `sigyn_interfaces/action/MoveExtender_FeedbackMessage` | `/teensy_bridge` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/gripper/move_extender/_action/status` | `action_msgs/msg/GoalStatusArray` | `/teensy_bridge` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/local_costmap/local_costmap/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | `/local_costmap/local_costmap` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/local_costmap/oakd_top_layer` | `nav_msgs/msg/OccupancyGrid` | `/local_costmap/local_costmap` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/local_costmap/oakd_top_layer_raw` | `nav2_msgs/msg/Costmap` | `/local_costmap/local_costmap` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/local_costmap/range_sensor_layer` | `nav_msgs/msg/OccupancyGrid` | `/local_costmap/local_costmap` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/local_costmap/range_sensor_layer_raw` | `nav2_msgs/msg/Costmap` | `/local_costmap/local_costmap` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/local_costmap/voxel_layer` | `nav_msgs/msg/OccupancyGrid` | `/local_costmap/local_costmap` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/local_costmap/voxel_layer_raw` | `nav2_msgs/msg/Costmap` | `/local_costmap/local_costmap` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/map_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | `/map_server` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/navigate_through_poses/_action/feedback` | `nav2_msgs/action/NavigateThroughPoses_FeedbackMessage` | `/bt_navigator` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/navigate_through_poses/_action/status` | `action_msgs/msg/GoalStatusArray` | `/bt_navigator` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/navigate_to_pose/_action/feedback` | `nav2_msgs/action/NavigateToPose_FeedbackMessage` | `/bt_navigator` RELIABLE/VOLATILE | `/bt_navigator` RELIABLE/VOLATILE<br>`/waypoint_follower` RELIABLE/VOLATILE |
| `/navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` | `/bt_navigator` RELIABLE/TRANSIENT_LOCAL | `/bt_navigator` RELIABLE/TRANSIENT_LOCAL<br>`/waypoint_follower` RELIABLE/TRANSIENT_LOCAL |
| `/oakd_top/can_point_base` | `geometry_msgs/msg/PointStamped` | `/oakd_detector` BEST_EFFORT/VOLATILE | <span style="color:orange">NONE</span> |
| `/oakd_top/can_point_camera` | `geometry_msgs/msg/PointStamped` | `/oakd_detector` BEST_EFFORT/VOLATILE | <span style="color:orange">NONE</span> |
| `/oakd_top/can_point_raw` | `geometry_msgs/msg/PointStamped` | `/oakd_detector` BEST_EFFORT/VOLATILE | <span style="color:orange">NONE</span> |
| `/oakd_top/color/image/compressed` | `sensor_msgs/msg/CompressedImage` | `/oakd_top/oakd_color_compressed_republisher` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/oakd_top/depth_sample_base` | `geometry_msgs/msg/PointStamped` | `/oakd_detector` BEST_EFFORT/VOLATILE | <span style="color:orange">NONE</span> |
| `/oakd_top/depth_sample_camera` | `geometry_msgs/msg/PointStamped` | `/oakd_detector` BEST_EFFORT/VOLATILE | <span style="color:orange">NONE</span> |
| `/oakd_top/out` | `sensor_msgs/msg/Image` | `/oakd_top/oakd_color_compressed_republisher` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/oakd_top/out/compressedDepth` | `sensor_msgs/msg/CompressedImage` | `/oakd_top/oakd_color_compressed_republisher` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/oakd_top/out/theora` | `theora_image_transport/msg/Packet` | `/oakd_top/oakd_color_compressed_republisher` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/oakd_top/out/zstd` | `sensor_msgs/msg/CompressedImage` | `/oakd_top/oakd_color_compressed_republisher` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/optimal_trajectory` | `nav_msgs/msg/Path` | `/controller_server` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | `/joint_state_publisher` RELIABLE/VOLATILE<br>`/battery_overlay_publisher` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE<br>`/wr_teleop_twist_keyboard` RELIABLE/VOLATILE<br>`/scan_to_scan_filter_chain` RELIABLE/VOLATILE<br>`/pointcloud_to_laserscan_node` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE<br>`/top_ldlidar` RELIABLE/VOLATILE<br>`/joint_state_publisher` RELIABLE/VOLATILE<br>`/wr_twist_multiplexer_node` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE<br>`/teensy_bridge` RELIABLE/VOLATILE<br>`/_ros2cli_daemon_0_d9b837226db446f08c14e93c332b5d5a` RELIABLE/VOLATILE<br>`/map_server` RELIABLE/VOLATILE<br>`/amcl` RELIABLE/VOLATILE<br>`/lifecycle_manager_localization` RELIABLE/VOLATILE<br>`/controller_server` RELIABLE/VOLATILE<br>`/local_costmap/local_costmap` RELIABLE/VOLATILE<br>`/smoother_server` RELIABLE/VOLATILE<br>`/planner_server` RELIABLE/VOLATILE<br>`/global_costmap/global_costmap` RELIABLE/VOLATILE<br>`/behavior_server` RELIABLE/VOLATILE<br>`/bt_navigator` RELIABLE/VOLATILE<br>`/waypoint_follower` RELIABLE/VOLATILE<br>`/velocity_smoother` RELIABLE/VOLATILE<br>`/lifecycle_manager_navigation` RELIABLE/VOLATILE<br>`/bt_navigator_navigate_to_pose_rclcpp_node` RELIABLE/VOLATILE<br>`/bt_navigator_navigate_through_poses_rclcpp_node` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE<br>`/launch_ros_4535` RELIABLE/VOLATILE<br>`/ekf_filter_node` RELIABLE/VOLATILE<br>`/joint_state_publisher` RELIABLE/VOLATILE<br>`/joint_state_publisher` RELIABLE/VOLATILE<br>`/topic_analysis_discovery` RELIABLE/VOLATILE<br>`/oakd_top/oakd_color_compressed_republisher` RELIABLE/VOLATILE<br>`/rqt_gui_py_node_1352476` RELIABLE/VOLATILE<br>`/cup_ldlidar` RELIABLE/VOLATILE<br>`/oakd_detector` RELIABLE/VOLATILE<br>`/joint_state_publisher` RELIABLE/VOLATILE<br>`/joint_state_publisher` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE<br>`/joint_state_publisher` RELIABLE/VOLATILE<br>`/sigyn_notifier` RELIABLE/VOLATILE | `/robot_state_publisher` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE<br>`/scan_to_scan_filter_chain` RELIABLE/VOLATILE<br>`/pointcloud_to_laserscan_node` RELIABLE/VOLATILE<br>`/transform_listener_impl_5e865b2ef690` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE<br>`/top_ldlidar` RELIABLE/VOLATILE<br>`/wr_twist_multiplexer_node` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE<br>`/teensy_bridge` RELIABLE/VOLATILE<br>`/nav2_container` RELIABLE/VOLATILE<br>`/map_server` RELIABLE/VOLATILE<br>`/amcl` RELIABLE/VOLATILE<br>`/lifecycle_manager_localization` RELIABLE/VOLATILE<br>`/controller_server` RELIABLE/VOLATILE<br>`/local_costmap/local_costmap` RELIABLE/VOLATILE<br>`/smoother_server` RELIABLE/VOLATILE<br>`/planner_server` RELIABLE/VOLATILE<br>`/global_costmap/global_costmap` RELIABLE/VOLATILE<br>`/behavior_server` RELIABLE/VOLATILE<br>`/bt_navigator` RELIABLE/VOLATILE<br>`/waypoint_follower` RELIABLE/VOLATILE<br>`/velocity_smoother` RELIABLE/VOLATILE<br>`/lifecycle_manager_navigation` RELIABLE/VOLATILE<br>`/transform_listener_impl_71275c008a00` RELIABLE/VOLATILE<br>`/transform_listener_impl_712754000fa0` RELIABLE/VOLATILE<br>`/bt_navigator_navigate_to_pose_rclcpp_node` RELIABLE/VOLATILE<br>`/bt_navigator_navigate_through_poses_rclcpp_node` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE<br>`/ekf_filter_node` RELIABLE/VOLATILE<br>`/transform_listener_impl_5f8a1e348bd0` RELIABLE/VOLATILE<br>`/oakd_top/oakd_color_compressed_republisher` RELIABLE/VOLATILE<br>`/oakd_top/oakd_color_compressed_republisher` RELIABLE/VOLATILE<br>`/oakd_top/oakd_color_compressed_republisher` RELIABLE/VOLATILE<br>`/oakd_top/oakd_color_compressed_republisher` RELIABLE/VOLATILE<br>`/cup_ldlidar` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE<br>`/robot_state_publisher` RELIABLE/VOLATILE |
| `/particle_cloud` | `nav2_msgs/msg/ParticleCloud` | `/amcl` BEST_EFFORT/VOLATILE | <span style="color:orange">NONE</span> |
| `/plan` | `nav_msgs/msg/Path` | `/planner_server` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/plan_smoothed` | `nav_msgs/msg/Path` | `/smoother_server` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/planner_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | `/planner_server` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/sigyn/safety/human_alert` | `wr_interfaces/msg/HumanAlert` | `/teensy_bridge` RELIABLE/VOLATILE | `/sigyn_notifier` RELIABLE/VOLATILE |
| `/sigyn/stepper/status` | `wr_interfaces/msg/StepperStatus` | `/teensy_bridge` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/smooth_path/_action/feedback` | `nav2_msgs/action/SmoothPath_FeedbackMessage` | `/smoother_server` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/smooth_path/_action/status` | `action_msgs/msg/GoalStatusArray` | `/smoother_server` RELIABLE/TRANSIENT_LOCAL | <span style="color:orange">NONE</span> |
| `/smoother_server/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | `/smoother_server` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/spin/_action/feedback` | `nav2_msgs/action/Spin_FeedbackMessage` | `/behavior_server` RELIABLE/VOLATILE | `/bt_navigator_navigate_to_pose_rclcpp_node` RELIABLE/VOLATILE<br>`/bt_navigator_navigate_through_poses_rclcpp_node` RELIABLE/VOLATILE |
| `/spin/_action/status` | `action_msgs/msg/GoalStatusArray` | `/behavior_server` RELIABLE/TRANSIENT_LOCAL | `/bt_navigator_navigate_to_pose_rclcpp_node` RELIABLE/TRANSIENT_LOCAL<br>`/bt_navigator_navigate_through_poses_rclcpp_node` RELIABLE/TRANSIENT_LOCAL |
| `/stereo/points2` | `sensor_msgs/msg/LaserScan` | `/pointcloud_to_laserscan_node` BEST_EFFORT/VOLATILE | `/local_costmap/local_costmap` BEST_EFFORT/VOLATILE |
| `/teensy_bridge/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | `/teensy_bridge` RELIABLE/VOLATILE | `/launch_ros_4535` RELIABLE/VOLATILE |
| `/trajectories` | `visualization_msgs/msg/MarkerArray` | `/controller_server` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/transformed_global_plan` | `nav_msgs/msg/Path` | `/controller_server` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/velocity_smoother/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | `/velocity_smoother` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |
| `/wait/_action/feedback` | `nav2_msgs/action/Wait_FeedbackMessage` | `/behavior_server` RELIABLE/VOLATILE | `/bt_navigator_navigate_to_pose_rclcpp_node` RELIABLE/VOLATILE<br>`/bt_navigator_navigate_through_poses_rclcpp_node` RELIABLE/VOLATILE |
| `/wait/_action/status` | `action_msgs/msg/GoalStatusArray` | `/behavior_server` RELIABLE/TRANSIENT_LOCAL | `/bt_navigator_navigate_to_pose_rclcpp_node` RELIABLE/TRANSIENT_LOCAL<br>`/bt_navigator_navigate_through_poses_rclcpp_node` RELIABLE/TRANSIENT_LOCAL |
| `/waypoint_follower/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | `/waypoint_follower` RELIABLE/VOLATILE | <span style="color:orange">NONE</span> |

### Topics with No Publishers

These topics have **subscribers but no publisher** — possible misconfiguration or a node that is not running.

| Topic | Type | Subscribers |
|-------|------|------------|
| `/clock` | `rosgraph_msgs/msg/Clock` | `/map_server` BEST_EFFORT/VOLATILE<br>`/amcl` BEST_EFFORT/VOLATILE |
| `/cmd_vel_gripper` | `geometry_msgs/msg/Twist` | `/teensy_bridge` RELIABLE/VOLATILE |
| `/cmd_vel_joystick` | `geometry_msgs/msg/Twist` | `/wr_twist_multiplexer_node` RELIABLE/VOLATILE |
| `/cmd_vel_keyboard` | `geometry_msgs/msg/Twist` | `/wr_twist_multiplexer_node` RELIABLE/VOLATILE |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | `/behavior_server` RELIABLE/VOLATILE |
| `/controller_selector` | `std_msgs/msg/String` | `/bt_navigator_navigate_to_pose_rclcpp_node` RELIABLE/TRANSIENT_LOCAL<br>`/bt_navigator_navigate_through_poses_rclcpp_node` RELIABLE/TRANSIENT_LOCAL |
| `/global_costmap/footprint` | `geometry_msgs/msg/Polygon` | `/global_costmap/global_costmap` RELIABLE/VOLATILE |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | `/bt_navigator` RELIABLE/VOLATILE |
| `/gripper/home` | `std_msgs/msg/Empty` | `/teensy_bridge` RELIABLE/VOLATILE |
| `/gripper/position/command` | `sigyn_interfaces/msg/GripperPositionCommand` | `/teensy_bridge` RELIABLE/VOLATILE |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `/amcl` RELIABLE/VOLATILE |
| `/local_costmap/footprint` | `geometry_msgs/msg/Polygon` | `/local_costmap/local_costmap` RELIABLE/VOLATILE |
| `/planner_selector` | `std_msgs/msg/String` | `/bt_navigator_navigate_to_pose_rclcpp_node` RELIABLE/TRANSIENT_LOCAL<br>`/bt_navigator_navigate_through_poses_rclcpp_node` RELIABLE/TRANSIENT_LOCAL |
| `/preempt_teleop` | `std_msgs/msg/Empty` | `/behavior_server` RELIABLE/VOLATILE |
| `/set_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `/ekf_filter_node` RELIABLE/VOLATILE |
| `/sigyn/take_oakd_picture` | `std_msgs/msg/Bool` | `/oakd_detector` RELIABLE/VOLATILE |
| `/sigyn/teensy_bridge/battery/status` | `sensor_msgs/msg/BatteryState` | `/battery_overlay_publisher` RELIABLE/VOLATILE |
| `/speed_limit` | `nav2_msgs/msg/SpeedLimit` | `/controller_server` RELIABLE/VOLATILE |

---

*Report generated by `report.py` — part of the Sigyn topic-analysis pipeline.*
*Edit `topic_descriptions.json` to improve descriptions, then re-run `describe.py && report.py`.*
