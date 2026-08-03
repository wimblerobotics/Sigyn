#!/bin/bash
# bag_record.sh - Record ROS2 bag with selected topics
# Output is written to the home directory with a timestamped folder name.
# Edit the TOPICS list below to include only the topics you want to record.

OUTPUT_DIR="$HOME/bags/sigyn_$(date +%Y%m%d_%H%M%S)"

TOPICS=(
  /amcl/transition_event
  /amcl_pose
  /battery_overlay_text
  /battery_state
  /behavior_server/transition_event
  /behavior_tree_log
  /bond
  /bt_navigator/transition_event
  /clicked_point
  /clock
  /cmd_vel
  /cmd_vel_gripper
  /cmd_vel_joystick
  /cmd_vel_keyboard
  /cmd_vel_nav
  /cmd_vel_smoothed
  /cmd_vel_teleop
  /controller_selector
  /controller_server/transition_event
  /detected_dock_pose
  /diagnostics
  /dock_pose
  /docking_server/transition_event
  /docking_trajectory
  /filtered_dock_pose
  /global_costmap/costmap
  /global_costmap/costmap_raw
  /global_costmap/costmap_raw_updates
  /global_costmap/costmap_updates
  /global_costmap/footprint
  /global_costmap/global_costmap/transition_event
  /global_costmap/obstacle_layer
  /global_costmap/obstacle_layer_raw
  /global_costmap/obstacle_layer_raw_updates
  /global_costmap/obstacle_layer_updates
  /global_costmap/published_footprint
  /global_costmap/static_layer
  /global_costmap/static_layer_raw
  /global_costmap/static_layer_raw_updates
  /global_costmap/static_layer_updates
  /goal_pose
  /gripper/home
  /gripper/position/command
  /gripper/status
  /initialpose
  /joint_states
  /local_costmap/clearing_endpoints
  /local_costmap/costmap
  /local_costmap/costmap_raw
  /local_costmap/costmap_raw_updates
  /local_costmap/costmap_updates
  /local_costmap/footprint
  /local_costmap/local_costmap/transition_event
  /local_costmap/oakd_top_layer
  /local_costmap/oakd_top_layer_raw
  /local_costmap/oakd_top_layer_raw_updates
  /local_costmap/oakd_top_layer_updates
  /local_costmap/published_footprint
  /local_costmap/range_sensor_layer
  /local_costmap/range_sensor_layer_raw
  /local_costmap/range_sensor_layer_raw_updates
  /local_costmap/range_sensor_layer_updates
  /local_costmap/voxel_layer
  /local_costmap/voxel_layer_raw
  /local_costmap/voxel_layer_raw_updates
  /local_costmap/voxel_layer_updates
  /map
  /map_server/transition_event
  /map_updates
  # /oakd/annotated_image
  # /oakd/annotated_image/compressed
  # /oakd/can_detections
  # /oakd/object_detector_heartbeat
  # /oakd_apriltag_node/annotated_image
  # /oakd_apriltag_node/camera_info
  # /oakd_apriltag_node/depth_image
  /oakd_apriltag_node/detections
  /oakd_apriltag_node/points
  # /oakd_apriltag_node/rgb_image
  # /oakd_top/camera_info
  # /oakd_top/can_point_base
  # /oakd_top/can_point_camera
  # /oakd_top/can_point_raw
  # /oakd_top/color/image/compressed
  # /oakd_top/depth_image
  # /oakd_top/depth_raw
  # /oakd_top/depth_sample_base
  # /oakd_top/depth_sample_camera
  # /oakd_top/out
  # /oakd_top/out/compressedDepth
  # /oakd_top/out/theora
  # /oakd_top/out/zstd
  # /oakd_top/points
  # /oakd_top/rgb_preview
  /odom
  /optimal_trajectory
  /parameter_events
  /particle_cloud
  /plan
  /plan_smoothed
  /planner_selector
  /planner_server/transition_event
  /preempt_teleop
  /raw_scan
  /robot_description
  /rosout
  /scan
  /scan_cup
  /scan_filtered
  /set_pose
  /sigyn/power/battery
  /sigyn/power/charger
  /sigyn/power/rail
  /sigyn/roboclaw/status
  /sigyn/safety/fault_events
  /sigyn/safety/fault_list
  /sigyn/safety/human_alert
  /sigyn/safety/state
  /sigyn/sensors/imu_front_right
  /sigyn/sensors/imu_rear_left
  /sigyn/sensors/odom
  /sigyn/sensors/range/front_left_fwd
  /sigyn/sensors/range/front_left_side
  /sigyn/sensors/range/front_right_fwd
  /sigyn/sensors/range/front_right_side
  /sigyn/sensors/range/rear_left_bkwd
  /sigyn/sensors/range/rear_left_side
  /sigyn/sensors/range/rear_right_bkwd
  /sigyn/sensors/range/rear_right_side
  /sigyn/stepper/status
  /sigyn/take_oakd_picture
  /sigyn/teensy_bridge/battery/status
  /smoother_server/transition_event
  /speed_limit
  /staging_pose
  /stereo/points2
  /teensy_bridge/status
  /teensy_bridge/transition_event
  /tf
  /tf_static
  /trajectories
  /transformed_global_plan
  /velocity_smoother/transition_event
  /waypoint_follower/transition_event
  /waypoints
)

echo "Recording bag to: $OUTPUT_DIR"
echo "Press Ctrl+C to stop recording."

ros2 bag record -o "$OUTPUT_DIR" "${TOPICS[@]}"
