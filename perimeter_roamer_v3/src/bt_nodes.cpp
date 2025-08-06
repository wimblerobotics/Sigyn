// Copyright 2025 Wimble Robotics
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "perimeter_roamer_v3/bt_nodes.hpp"
#include <chrono>
#include <random>
#include <cmath>

using namespace std::chrono_literals;

namespace perimeter_roamer_v3
{

  // CheckBatteryState Implementation
  BT::NodeStatus CheckBatteryState::tick()
  {
    if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("CheckBatteryState"), "ROS node not set!");
      return BT::NodeStatus::FAILURE;
    }

    // Get data from blackboard
    float battery_level = 100.0f;
    rclcpp::Time battery_time(0);

    auto blackboard = config().blackboard;
    if (blackboard) {
      try {
        battery_level = blackboard->get<float>("battery_level");
      }
      catch (const std::exception&) {
        // Key not found or wrong type, use default
      }

      try {
        battery_time = blackboard->get<rclcpp::Time>("battery_time");
      }
      catch (const std::exception&) {
        // Key not found or wrong type, use default
      }
    }

    float threshold = 20.0f;
    getInput("low_battery_threshold", threshold);

    // Check if we have recent battery data
    auto now = node_->get_clock()->now();

    if (battery_time.nanoseconds() == 0) {
      RCLCPP_INFO_THROTTLE(rclcpp::get_logger("CheckBatteryState"), *node_->get_clock(), 2000,
        "No battery data available yet, assuming OK");
      return BT::NodeStatus::FAILURE; // Battery OK, don't need to charge
    }

    if ((now - battery_time).seconds() > 5.0) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("CheckBatteryState"), *node_->get_clock(), 5000,
        "Battery data is stale, assuming battery OK");
      return BT::NodeStatus::FAILURE; // Battery OK, don't need to charge
    }

    if (battery_level <= threshold) {
      RCLCPP_INFO(rclcpp::get_logger("CheckBatteryState"),
        "Battery low (%.1f%%), need to charge!", battery_level);
      return BT::NodeStatus::SUCCESS; // Battery low, need to charge
    }
    else {
      RCLCPP_DEBUG(rclcpp::get_logger("CheckBatteryState"),
        "Battery OK (%.1f%%)", battery_level);
      return BT::NodeStatus::FAILURE; // Battery OK, don't need to charge
    }
  }

  // ClassifySpace Implementation

  // CheckLidarHealth Implementation
  BT::NodeStatus CheckLidarHealth::tick()
  {
    if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("CheckLidarHealth"), "ROS node not set!");
      return BT::NodeStatus::FAILURE;
    }

    // Get data from blackboard
    sensor_msgs::msg::LaserScan::SharedPtr scan_data;
    rclcpp::Time scan_time(0);

    auto blackboard = config().blackboard;
    if (blackboard) {
      try {
        scan_data = blackboard->get<sensor_msgs::msg::LaserScan::SharedPtr>("scan_data");
      }
      catch (const std::exception&) {
        // Key not found or wrong type
      }

      try {
        scan_time = blackboard->get<rclcpp::Time>("scan_time");
      }
      catch (const std::exception&) {
        // Key not found or wrong type, use default
      }
    }

    float max_invalid_ratio = 0.5f;
    float scan_timeout = 2.0f;
    getInput("max_invalid_ratio", max_invalid_ratio);
    getInput("scan_timeout", scan_timeout);

    // Check if we have recent scan data
    auto now = node_->get_clock()->now();

    if (scan_time.nanoseconds() == 0) {
      RCLCPP_INFO_THROTTLE(rclcpp::get_logger("CheckLidarHealth"), *node_->get_clock(), 2000,
        "No LIDAR data available yet");
      return BT::NodeStatus::FAILURE;
    }

    if ((now - scan_time).seconds() > scan_timeout) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("CheckLidarHealth"), *node_->get_clock(), 2000,
        "LIDAR data is stale!");
      return BT::NodeStatus::FAILURE;
    }

    if (!scan_data) {
      return BT::NodeStatus::FAILURE;
    }

    // Count invalid readings (inf, nan, or out of range)
    int invalid_count = 0;
    int total_count = scan_data->ranges.size();

    for (const auto& range : scan_data->ranges) {
      if (std::isinf(range) || std::isnan(range) ||
        range < scan_data->range_min || range > scan_data->range_max) {
        invalid_count++;
      }
    }

    float invalid_ratio = static_cast<float>(invalid_count) / total_count;

    if (invalid_ratio > max_invalid_ratio) {
      RCLCPP_WARN(rclcpp::get_logger("CheckLidarHealth"),
        "LIDAR health poor: %.1f%% invalid readings", invalid_ratio * 100.0f);
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("CheckLidarHealth"),
      "LIDAR health good: %.1f%% invalid readings", invalid_ratio * 100.0f);
    return BT::NodeStatus::SUCCESS;
  }

  void CheckLidarHealth::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_ = msg;
    last_scan_time_ = node_->get_clock()->now();
    RCLCPP_INFO(rclcpp::get_logger("CheckLidarHealth"),
      "LIDAR callback received: %zu ranges, frame: %s",
      msg->ranges.size(), msg->header.frame_id.c_str());
  }

  // ClassifySpace Implementation
  BT::NodeStatus ClassifySpace::tick()
  {
    if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("ClassifySpace"), "ROS node not set!");
      return BT::NodeStatus::FAILURE;
    }

    // Get scan data from blackboard instead of subscription
    sensor_msgs::msg::LaserScan::SharedPtr current_scan;
    try {
      current_scan = config().blackboard->get<sensor_msgs::msg::LaserScan::SharedPtr>("scan_data");
    }
    catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("ClassifySpace"), "No scan data available in blackboard");
      return BT::NodeStatus::FAILURE;
    }

    if (!current_scan) {
      RCLCPP_WARN(rclcpp::get_logger("ClassifySpace"), "Scan data is null");
      return BT::NodeStatus::FAILURE;
    }

    // Analyze the current space (skip database for now)
    SpaceType space_type = analyzeSpace(current_scan);

    // Convert to string and set output
    std::string space_str;
    switch (space_type) {
    case SpaceType::ROOM: space_str = "ROOM"; break;
    case SpaceType::HALLWAY: space_str = "HALLWAY"; break;
    case SpaceType::DOORWAY: space_str = "DOORWAY"; break;
    case SpaceType::VERY_NARROW: space_str = "VERY_NARROW"; break;
    default: space_str = "UNKNOWN"; break;
    }

    setOutput("space_type", space_str);

    // Generate suggested pose
    geometry_msgs::msg::Pose next_pose = generateNextPose(space_type);
    setOutput("suggested_pose", next_pose);

    RCLCPP_INFO(rclcpp::get_logger("ClassifySpace"),
      "Classified space as: %s, generated pose: x=%.2f, y=%.2f, z=%.2f",
      space_str.c_str(), next_pose.position.x, next_pose.position.y, next_pose.position.z);

    return BT::NodeStatus::SUCCESS;
  }

  bool ClassifySpace::loadWallDatabase()
  {
    std::string db_path = "/home/ros/sigyn_ws/walls.db";

    int rc = sqlite3_open(db_path.c_str(), &db_);
    if (rc != SQLITE_OK) {
      RCLCPP_ERROR(rclcpp::get_logger("ClassifySpace"), "Can't open database: %s", sqlite3_errmsg(db_));
      return false;
    }

    // Query walls from database (adjust query based on actual schema)
    const char* sql = "SELECT start_x, start_y, length, width FROM walls";
    sqlite3_stmt* stmt;

    rc = sqlite3_prepare_v2(db_, sql, -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
      RCLCPP_ERROR(rclcpp::get_logger("ClassifySpace"), "SQL prepare error: %s", sqlite3_errmsg(db_));
      return false;
    }

    walls_.clear();
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      Wall wall;
      wall.start_x = sqlite3_column_double(stmt, 0);
      wall.start_y = sqlite3_column_double(stmt, 1);
      wall.length = sqlite3_column_double(stmt, 2);
      wall.width = sqlite3_column_double(stmt, 3);

      // Calculate angle based on dimensions (simple heuristic)
      wall.angle = (wall.length > wall.width) ? 0.0 : M_PI / 2.0;

      walls_.push_back(wall);
    }

    sqlite3_finalize(stmt);

    RCLCPP_INFO(rclcpp::get_logger("ClassifySpace"), "Loaded %zu walls from database", walls_.size());
    return true;
  }

  SpaceType ClassifySpace::analyzeSpace(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    if (!scan || scan->ranges.empty()) {
      return SpaceType::UNKNOWN;
    }

    // Analyze LIDAR data to classify space
    std::vector<float> valid_ranges;
    for (const auto& range : scan->ranges) {
      if (!std::isinf(range) && !std::isnan(range) &&
        range >= scan->range_min && range <= scan->range_max) {
        valid_ranges.push_back(range);
      }
    }

    if (valid_ranges.empty()) {
      return SpaceType::UNKNOWN;
    }

    // Calculate statistics
    float min_range = *std::min_element(valid_ranges.begin(), valid_ranges.end());
    float max_range = *std::max_element(valid_ranges.begin(), valid_ranges.end());
    float avg_range = std::accumulate(valid_ranges.begin(), valid_ranges.end(), 0.0f) / valid_ranges.size();

    // Count how many readings are within different distance bands
    int close_readings = 0;   // < 0.8m (very close to walls)
    int medium_readings = 0;  // 0.8m - 2.0m 
    int far_readings = 0;     // > 2.0m

    for (float range : valid_ranges) {
      if (range < 0.8f) close_readings++;
      else if (range < 2.0f) medium_readings++;
      else far_readings++;
    }

    int total_readings = valid_ranges.size();
    float close_ratio = static_cast<float>(close_readings) / total_readings;
    float far_ratio = static_cast<float>(far_readings) / total_readings;

    // Classification logic based on robot's 0.44m radius
    if (min_range < 0.6f && close_ratio > 0.7f) {
      // Very constrained space
      if (avg_range < 1.0f) {
        return SpaceType::VERY_NARROW;
      }
      else {
        return SpaceType::DOORWAY;
      }
    }
    else if (far_ratio > 0.3f && max_range > 3.0f) {
      // Open space - likely a room
      return SpaceType::ROOM;
    }
    else if (close_ratio > 0.4f && avg_range < 1.5f) {
      // Constrained but not very narrow - likely hallway
      return SpaceType::HALLWAY;
    }
    else if (far_ratio > 0.2f) {
      // Some open space - default to room
      return SpaceType::ROOM;
    }

    return SpaceType::HALLWAY; // Default fallback
  }

  geometry_msgs::msg::Pose ClassifySpace::generateNextPose(SpaceType space_type)
  {
    geometry_msgs::msg::Pose pose;

    // Initialize with current pose (in practice, you'd get this from TF)
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;

    // Generate random movement based on space type
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> angle_dist(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<> distance_dist(0.5, 2.0);

    double angle = angle_dist(gen);
    double distance;

    switch (space_type) {
    case SpaceType::ROOM:
      // In rooms, move further and more freely
      distance = std::uniform_real_distribution<>(1.0, 3.0)(gen);
      break;
    case SpaceType::HALLWAY:
      // In hallways, shorter movements to stay centered
      distance = std::uniform_real_distribution<>(0.5, 1.5)(gen);
      break;
    case SpaceType::DOORWAY:
    case SpaceType::VERY_NARROW:
      // In narrow spaces, very careful small movements
      distance = std::uniform_real_distribution<>(0.3, 0.8)(gen);
      break;
    default:
      distance = 1.0;
    }

    pose.position.x = distance * cos(angle);
    pose.position.y = distance * sin(angle);

    // Set orientation to face movement direction
    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    pose.orientation = tf2::toMsg(q);

    return pose;
  }

  void ClassifySpace::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    current_scan_ = msg;
    RCLCPP_INFO(rclcpp::get_logger("ClassifySpace"),
      "ClassifySpace LIDAR callback received: %zu ranges",
      msg->ranges.size());
  }

  // NavigateToPose Implementation
  BT::NodeStatus NavigateToPose::onStart()
  {
    if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "ROS node not set!");
      return BT::NodeStatus::FAILURE;
    }

    // Initialize action client if needed
    if (!action_client_) {
      action_client_ = rclcpp_action::create_client<NavigateAction>(
        node_->get_node_base_interface(), node_->get_node_graph_interface(),
        node_->get_node_logging_interface(), node_->get_node_waitables_interface(),
        "/navigate_to_pose");

      // Non-blocking check - if server is not available, we'll retry
      if (!action_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
        RCLCPP_WARN(rclcpp::get_logger("NavigateToPose"), "Action server not immediately available, will retry...");
        action_state_ = ActionState::IDLE;
        return BT::NodeStatus::RUNNING; // Keep trying
      }
    }

    // Check if we're resuming an interrupted goal
    if (was_interrupted_ && action_state_ == ActionState::GOAL_INTERRUPTED) {
      RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "Resuming interrupted navigation goal");
      was_interrupted_ = false;
      action_state_ = ActionState::IDLE; // Reset to allow new goal
    }

    // Get inputs for new goal or check if current goal is still valid
    geometry_msgs::msg::Pose target_pose;
    if (!getInput("target_pose", target_pose)) {
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "No target_pose provided!");
      return BT::NodeStatus::FAILURE;
    }

    std::string behavior_tree;
    getInput("behavior_tree", behavior_tree);
    
    float timeout = 30.0f;
    getInput("timeout", timeout);

    // Check if this is a new goal or the same goal
    bool is_new_goal = (action_state_ == ActionState::IDLE) ||
                       (std::abs(target_pose.position.x - current_target_pose_.position.x) > 0.01) ||
                       (std::abs(target_pose.position.y - current_target_pose_.position.y) > 0.01) ||
                       (behavior_tree != current_behavior_tree_);

    if (is_new_goal) {
      // Cancel any existing goal
      if (goal_handle_ && action_state_ != ActionState::IDLE) {
        RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "Canceling previous goal for new target");
        auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
      }

      // Store new goal parameters
      current_target_pose_ = target_pose;
      current_behavior_tree_ = behavior_tree;
      current_timeout_ = timeout;
      goal_start_time_ = std::chrono::steady_clock::now();
      
      // Reset state
      action_state_ = ActionState::IDLE;
      result_received_ = false;
      navigation_result_ = BT::NodeStatus::FAILURE;
    }

    // Send goal if we're in idle state
    if (action_state_ == ActionState::IDLE) {
      if (sendGoal()) {
        action_state_ = ActionState::SENDING_GOAL;
        RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "Sent navigation goal");
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "Failed to send navigation goal");
        return BT::NodeStatus::FAILURE;
      }
    }

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus NavigateToPose::onRunning()
  {
    // Check timeout
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - goal_start_time_).count();
    if (elapsed >= current_timeout_) {
      RCLCPP_WARN(rclcpp::get_logger("NavigateToPose"), 
                  "Navigation timeout after %.1f seconds", current_timeout_);
      if (goal_handle_) {
        auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
      }
      action_state_ = ActionState::GOAL_FAILED;
      setOutput("goal_interrupted", false);
      return BT::NodeStatus::FAILURE;
    }

    // Handle different action states
    switch (action_state_) {
      case ActionState::SENDING_GOAL:
        // Check if goal was accepted/rejected (non-blocking)
        if (goal_handle_future_.valid() &&
            goal_handle_future_.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready) {
          goal_handle_ = goal_handle_future_.get();
          if (!goal_handle_) {
            RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "Goal was rejected!");
            action_state_ = ActionState::GOAL_FAILED;
            setOutput("goal_interrupted", false);
            return BT::NodeStatus::FAILURE;
          }
          action_state_ = ActionState::GOAL_ACTIVE;
          RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "Goal accepted, navigation active");
        }
        return BT::NodeStatus::RUNNING;

      case ActionState::GOAL_ACTIVE:
        // Check result (non-blocking)
        if (result_received_.load()) {
          BT::NodeStatus result = navigation_result_.load();
          if (result == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "Navigation succeeded!");
            action_state_ = ActionState::GOAL_COMPLETED;
            setOutput("goal_interrupted", false);
            return BT::NodeStatus::SUCCESS;
          } else {
            RCLCPP_WARN(rclcpp::get_logger("NavigateToPose"), "Navigation failed or was canceled!");
            action_state_ = ActionState::GOAL_FAILED;
            setOutput("goal_interrupted", false);
            return BT::NodeStatus::FAILURE;
          }
        }
        return BT::NodeStatus::RUNNING;

      case ActionState::GOAL_COMPLETED:
        setOutput("goal_interrupted", false);
        return BT::NodeStatus::SUCCESS;

      case ActionState::GOAL_FAILED:
        setOutput("goal_interrupted", false);
        return BT::NodeStatus::FAILURE;

      case ActionState::GOAL_INTERRUPTED:
        // This state is set when onHalted() is called
        setOutput("goal_interrupted", true);
        return BT::NodeStatus::FAILURE; // Allow behavior tree to handle interruption

      default:
        return BT::NodeStatus::RUNNING;
    }
  }

  void NavigateToPose::onHalted()
  {
    RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "NavigateToPose halted - marking as interrupted");
    
    if (goal_handle_ && (action_state_ == ActionState::GOAL_ACTIVE || action_state_ == ActionState::SENDING_GOAL)) {
      RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "Canceling active navigation goal");
      auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
      // Don't reset goal_handle_ immediately - let it cancel gracefully
    }
    
    // Mark as interrupted so we can resume later if needed
    action_state_ = ActionState::GOAL_INTERRUPTED;
    was_interrupted_ = true;
    setOutput("goal_interrupted", true);
  }

  bool NavigateToPose::sendGoal()
  {
    if (!action_client_) {
      return false;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node_->get_clock()->now();
    goal_msg.pose.pose = current_target_pose_;
    goal_msg.behavior_tree = current_behavior_tree_;

    auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavigateToPose::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.result_callback =
      std::bind(&NavigateToPose::resultCallback, this, std::placeholders::_1);

    // Reset atomic flags
    result_received_ = false;
    navigation_result_ = BT::NodeStatus::FAILURE;

    goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
    return true;
  }

  void NavigateToPose::goalResponseCallback(const rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr& goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "Goal was rejected by server");
      action_state_ = ActionState::GOAL_FAILED;
    }
    else {
      RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "Goal accepted by server");
      action_state_ = ActionState::GOAL_ACTIVE;
    }
  }

  void NavigateToPose::resultCallback(const rclcpp_action::ClientGoalHandle<NavigateAction>::WrappedResult& result)
  {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "Navigation completed successfully");
      navigation_result_.store(BT::NodeStatus::SUCCESS);
      action_state_ = ActionState::GOAL_COMPLETED;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "Navigation was aborted");
      navigation_result_.store(BT::NodeStatus::FAILURE);
      action_state_ = ActionState::GOAL_FAILED;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      // Check if this was our own cancellation due to interruption
      if (action_state_ == ActionState::GOAL_INTERRUPTED) {
        RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "Navigation was canceled due to interruption");
      } else {
        RCLCPP_WARN(rclcpp::get_logger("NavigateToPose"), "Navigation was canceled");
        action_state_ = ActionState::GOAL_FAILED;
      }
      navigation_result_.store(BT::NodeStatus::FAILURE);
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "Unknown result code");
      navigation_result_.store(BT::NodeStatus::FAILURE);
      action_state_ = ActionState::GOAL_FAILED;
      break;
    }
    
    result_received_.store(true);
  }

  // LoadWaypoints Implementation
  BT::NodeStatus LoadWaypoints::tick()
  {
    if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("LoadWaypoints"), "ROS node not set!");
      return BT::NodeStatus::FAILURE;
    }

    std::string database_path;
    if (!getInput("database_path", database_path)) {
      RCLCPP_ERROR(rclcpp::get_logger("LoadWaypoints"), "database_path not provided");
      return BT::NodeStatus::FAILURE;
    }

    // Expand home directory if needed
    if (database_path.find("~/") == 0) {
      const char* home = getenv("HOME");
      if (home) {
        database_path = std::string(home) + database_path.substr(1);
      }
    }

    bool success = loadWaypointsFromDatabase(database_path);

    if (success && !waypoints_.empty()) {
      // Store waypoints in blackboard for other nodes to access
      config().blackboard->set("waypoints", waypoints_);
      config().blackboard->set("total_waypoints", static_cast<int>(waypoints_.size()));
      // Initialize current_waypoint_index only on first load
      bool first_load = false;
      try {
        // If key exists, get will succeed
        (void)config().blackboard->get<int>("current_waypoint_index");
      }
      catch (const std::exception&) {
        first_load = true;
      }
      if (first_load) {
        config().blackboard->set("current_waypoint_index", 0);
        RCLCPP_INFO(rclcpp::get_logger("LoadWaypoints"),
          "LoadWaypoints: initialized current_waypoint_index to 0");
      }
      else {
        RCLCPP_INFO(rclcpp::get_logger("LoadWaypoints"),
          "LoadWaypoints: retained existing current_waypoint_index = %d",
          config().blackboard->get<int>("current_waypoint_index"));
      }
      setOutput("waypoints_loaded", true);

      RCLCPP_INFO(rclcpp::get_logger("LoadWaypoints"),
        "Successfully loaded %zu waypoints from database: %s",
        waypoints_.size(), database_path.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    else {
      setOutput("waypoints_loaded", false);
      RCLCPP_ERROR(rclcpp::get_logger("LoadWaypoints"),
        "Failed to load waypoints from database: %s", database_path.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  bool LoadWaypoints::loadWaypointsFromDatabase(const std::string& db_path)
  {
    sqlite3* db;
    int rc = sqlite3_open(db_path.c_str(), &db);

    if (rc != SQLITE_OK) {
      RCLCPP_ERROR(rclcpp::get_logger("LoadWaypoints"),
        "Can't open database: %s", sqlite3_errmsg(db));
      return false;
    }

    const char* sql = "SELECT id, x_pose, y_pose, z_pose, x_orientation, y_orientation, z_orientation, w_orientation, text FROM waypoints ORDER BY id";
    sqlite3_stmt* stmt;

    rc = sqlite3_prepare_v2(db, sql, -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
      RCLCPP_ERROR(rclcpp::get_logger("LoadWaypoints"),
        "SQL prepare error: %s", sqlite3_errmsg(db));
      sqlite3_close(db);
      return false;
    }

    waypoints_.clear();
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      Waypoint wp;
      wp.id = sqlite3_column_int(stmt, 0);
      wp.x = sqlite3_column_double(stmt, 1);
      wp.y = sqlite3_column_double(stmt, 2);
      wp.z = sqlite3_column_double(stmt, 3);
      wp.qx = sqlite3_column_double(stmt, 4);
      wp.qy = sqlite3_column_double(stmt, 5);
      wp.qz = sqlite3_column_double(stmt, 6);
      wp.qw = sqlite3_column_double(stmt, 7);

      const char* text = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 8));
      wp.text = text ? text : "";
      wp.visited = false;

      waypoints_.push_back(wp);

      RCLCPP_DEBUG(rclcpp::get_logger("LoadWaypoints"),
        "Loaded waypoint %d: (%.2f, %.2f, %.2f) '%s'",
        wp.id, wp.x, wp.y, wp.z, wp.text.c_str());
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);

    return !waypoints_.empty();
  }

  // CheckWaypointsComplete Implementation
  BT::NodeStatus CheckWaypointsComplete::tick()
  {
    int current_index = 0;
    int total_waypoints = 0;
    bool loop_waypoints = false;

    try {
      current_index = config().blackboard->get<int>("current_waypoint_index");
      total_waypoints = config().blackboard->get<int>("total_waypoints");
      loop_waypoints = config().blackboard->get<bool>("loop_waypoints");
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("CheckWaypointsComplete"),
        "Failed to get waypoint data from blackboard: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }

    // Check if we've completed all waypoints
    if (current_index >= total_waypoints) {
      if (loop_waypoints) {
        // Reset to start and continue
        config().blackboard->set("current_waypoint_index", 0);
        setOutput("all_complete", false);
        RCLCPP_INFO(rclcpp::get_logger("CheckWaypointsComplete"),
          "Completed all waypoints, looping back to start");
        return BT::NodeStatus::SUCCESS; // Continue the sequence
      }
      else {
        // Stop - all waypoints completed
        setOutput("all_complete", true);
        RCLCPP_INFO(rclcpp::get_logger("CheckWaypointsComplete"),
          "All waypoints completed! Stopping waypoint following.");
        return BT::NodeStatus::FAILURE; // This will cause the sequence to fail and exit
      }
    }

    // Still have waypoints to visit
    setOutput("all_complete", false);
    RCLCPP_INFO(rclcpp::get_logger("CheckWaypointsComplete"),
      "Waypoint %d of %d - continuing", current_index + 1, total_waypoints);
    return BT::NodeStatus::SUCCESS; // Continue the sequence
  }

  // GetNextWaypoint Implementation
  BT::NodeStatus GetNextWaypoint::tick()
  {
    // Get waypoints from blackboard
    std::vector<Waypoint> waypoints = getWaypointsFromBlackboard();
    if (waypoints.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("GetNextWaypoint"), "No waypoints available");
      return BT::NodeStatus::FAILURE;
    }

    // Get current index
    int current_index = 0;
    try {
      current_index = config().blackboard->get<int>("current_waypoint_index");
    }
    catch (const std::exception&) {
      RCLCPP_WARN(rclcpp::get_logger("GetNextWaypoint"), "current_waypoint_index not found, using 0");
      config().blackboard->set("current_waypoint_index", 0);
    }
    // Debug: log the retrieved current index and total waypoints
    RCLCPP_INFO(rclcpp::get_logger("GetNextWaypoint"),
      "GetNextWaypoint: current_waypoint_index from blackboard = %d", current_index);
    RCLCPP_INFO(rclcpp::get_logger("GetNextWaypoint"),
      "GetNextWaypoint: total waypoints available = %zu", waypoints.size());

    // Check bounds
    if (current_index >= static_cast<int>(waypoints.size())) {
      RCLCPP_ERROR(rclcpp::get_logger("GetNextWaypoint"),
        "Current index %d exceeds waypoint count %zu",
        current_index, waypoints.size());
      return BT::NodeStatus::FAILURE;
    }

    // Get the waypoint (waypoints are already sorted by ID from LoadWaypoints)
    const Waypoint& wp = waypoints[current_index];
    // Debug: log the selected waypoint details
    RCLCPP_INFO(rclcpp::get_logger("GetNextWaypoint"),
      "GetNextWaypoint: selected waypoint id=%d, name='%s', pos=(%.2f, %.2f, %.2f)",
      wp.id, wp.text.c_str(), wp.x, wp.y, wp.z);

    // Create pose message
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = wp.x;
    target_pose.position.y = wp.y;
    target_pose.position.z = wp.z;
    target_pose.orientation.x = wp.qx;
    target_pose.orientation.y = wp.qy;
    target_pose.orientation.z = wp.qz;
    target_pose.orientation.w = wp.qw;

    // Set outputs
    setOutput("target_pose", target_pose);
    setOutput("waypoint_id", wp.id);
    setOutput("waypoint_name", wp.text);

    RCLCPP_INFO(rclcpp::get_logger("GetNextWaypoint"),
      "Next waypoint [%d/%zu]: ID=%d, Name='%s', Pos=(%.2f, %.2f, %.2f)",
      current_index + 1, waypoints.size(), wp.id, wp.text.c_str(), wp.x, wp.y, wp.z);

    return BT::NodeStatus::SUCCESS;
  }

  std::vector<Waypoint> GetNextWaypoint::getWaypointsFromBlackboard()
  {
    std::vector<Waypoint> waypoints;
    try {
      waypoints = config().blackboard->get<std::vector<Waypoint>>("waypoints");
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("GetNextWaypoint"),
        "Failed to get waypoints from blackboard: %s", e.what());
    }
    return waypoints;
  }

  // NavigateToWaypoint Implementation
  // NavigateToWaypoint Implementation
  BT::NodeStatus NavigateToWaypoint::onStart()
  {
    if (!node_) {
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToWaypoint"), "ROS node not set!");
      return BT::NodeStatus::FAILURE;
    }

    // Initialize action client if not already done
    if (!action_client_) {
      action_client_ = rclcpp_action::create_client<NavigateAction>(node_, "navigate_to_pose");

      if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(rclcpp::get_logger("NavigateToWaypoint"),
          "NavigateToPose action server not available after 5 seconds! Is Nav2 running?");
        return BT::NodeStatus::FAILURE;
      }
    }

    // Get inputs
    geometry_msgs::msg::Pose target_pose;
    if (!getInput("target_pose", target_pose)) {
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToWaypoint"), "No target_pose provided!");
      return BT::NodeStatus::FAILURE;
    }

    getInput("waypoint_id", current_waypoint_id_);
    getInput("waypoint_name", current_waypoint_name_);
    getInput("timeout", timeout_seconds_);

    // Record start time for timeout and feedback tracking
    start_time_ = std::chrono::steady_clock::now();
    last_feedback_time_ = start_time_;
    last_distance_remaining_ = -1.0;

    RCLCPP_INFO(rclcpp::get_logger("NavigateToWaypoint"),
      "Starting navigation to waypoint %d '%s' at (%.2f, %.2f) with timeout %.1fs",
      current_waypoint_id_, current_waypoint_name_.c_str(),
      target_pose.position.x, target_pose.position.y, timeout_seconds_);

    // Validate the pose before sending
    if (std::isnan(target_pose.position.x) || std::isnan(target_pose.position.y) ||
      std::isnan(target_pose.orientation.w)) {
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToWaypoint"),
        "Invalid target pose contains NaN values! Pose: x=%.2f, y=%.2f, w=%.2f",
        target_pose.position.x, target_pose.position.y, target_pose.orientation.w);
      return BT::NodeStatus::FAILURE;
    }

    auto goal_msg = NavigateAction::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node_->get_clock()->now();
    goal_msg.pose.pose = target_pose;

    // Reset atomic flags
    result_received_ = false;
    navigation_result_ = BT::NodeStatus::FAILURE;

    auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&NavigateToWaypoint::resultCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&NavigateToWaypoint::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

    // Send goal asynchronously and store the goal handle  
    auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);

    // Wait longer for goal acceptance to detect immediate rejections
    if (goal_handle_future.wait_for(std::chrono::milliseconds(1000)) == std::future_status::ready) {
      goal_handle_ = goal_handle_future.get();
      if (!goal_handle_) {
        RCLCPP_ERROR(rclcpp::get_logger("NavigateToWaypoint"),
          "Goal to waypoint %d was REJECTED! Check if robot is localized and goal is reachable.",
          current_waypoint_id_);
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(rclcpp::get_logger("NavigateToWaypoint"),
        "Goal to waypoint %d accepted", current_waypoint_id_);
    }
    else {
      // Goal handle will be set asynchronously, continue to RUNNING state
      RCLCPP_INFO(rclcpp::get_logger("NavigateToWaypoint"),
        "Goal to waypoint %d sent, waiting for acceptance", current_waypoint_id_);
    }

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus NavigateToWaypoint::onRunning()
  {
    // Check for timeout first
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_).count();
    if (elapsed >= timeout_seconds_) {
      RCLCPP_WARN(rclcpp::get_logger("NavigateToWaypoint"),
        "Navigation to waypoint %d timed out after %.1fs",
        current_waypoint_id_, timeout_seconds_);
      if (goal_handle_) {
        auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
      }
      return BT::NodeStatus::FAILURE;
    }

    // Check result - use atomic load for thread safety
    if (result_received_.load()) {
      BT::NodeStatus result = navigation_result_.load();
      if (result == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("NavigateToWaypoint"),
          "Successfully reached waypoint %d '%s'!",
          current_waypoint_id_, current_waypoint_name_.c_str());
        return BT::NodeStatus::SUCCESS;
      }
      else {
        RCLCPP_WARN(rclcpp::get_logger("NavigateToWaypoint"),
          "Failed to reach waypoint %d '%s'",
          current_waypoint_id_, current_waypoint_name_.c_str());
        return BT::NodeStatus::FAILURE;
      }
    }

    // Add periodic status logging
    if (elapsed > 0 && elapsed % 5 == 0) {
      static int last_logged_second = -1;
      if (elapsed != last_logged_second) {
        RCLCPP_INFO(rclcpp::get_logger("NavigateToWaypoint"),
          "Still navigating to waypoint %d '%s'... (%.0fs/%.0fs)",
          current_waypoint_id_, current_waypoint_name_.c_str(),
          static_cast<double>(elapsed), static_cast<double>(timeout_seconds_));
        last_logged_second = elapsed;
      }
    }

    return BT::NodeStatus::RUNNING;
  }

  void NavigateToWaypoint::onHalted()
  {
    if (goal_handle_) {
      RCLCPP_INFO(rclcpp::get_logger("NavigateToWaypoint"),
        "Canceling navigation to waypoint %d", current_waypoint_id_);
      auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
      goal_handle_.reset();
    }
  }

  void NavigateToWaypoint::resultCallback(const rclcpp_action::ClientGoalHandle<NavigateAction>::WrappedResult& result)
  {
    // Set atomic flags for thread safety
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      navigation_result_.store(BT::NodeStatus::SUCCESS);
      RCLCPP_INFO(rclcpp::get_logger("NavigateToWaypoint"),
        "Navigation result: SUCCESS for waypoint %d", current_waypoint_id_);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      navigation_result_.store(BT::NodeStatus::FAILURE);
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToWaypoint"),
        "Navigation result: ABORTED for waypoint %d", current_waypoint_id_);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      navigation_result_.store(BT::NodeStatus::FAILURE);
      RCLCPP_WARN(rclcpp::get_logger("NavigateToWaypoint"),
        "Navigation result: CANCELED for waypoint %d", current_waypoint_id_);
      break;
    default:
      navigation_result_.store(BT::NodeStatus::FAILURE);
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToWaypoint"),
        "Navigation result: UNKNOWN (%d) for waypoint %d",
        static_cast<int>(result.code), current_waypoint_id_);
      break;
    }

    // Signal that result has been received
    result_received_.store(true);
  }

  void NavigateToWaypoint::feedbackCallback(rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr,
    const std::shared_ptr<const NavigateAction::Feedback> feedback)
  {
    auto now = std::chrono::steady_clock::now();
    auto time_since_last_feedback = std::chrono::duration_cast<std::chrono::seconds>(now - last_feedback_time_).count();

    // Log feedback every 3 seconds to avoid spam
    if (time_since_last_feedback >= 3) {
      double current_distance = feedback->distance_remaining;

      if (last_distance_remaining_ > 0) {
        double distance_progress = last_distance_remaining_ - current_distance;
        RCLCPP_INFO(rclcpp::get_logger("NavigateToWaypoint"),
          "Navigation progress to waypoint %d: %.2fm remaining (%.2fm progress in %lds)",
          current_waypoint_id_, current_distance, distance_progress, time_since_last_feedback);
      }
      else {
        RCLCPP_INFO(rclcpp::get_logger("NavigateToWaypoint"),
          "Navigation progress to waypoint %d: %.2fm remaining",
          current_waypoint_id_, current_distance);
      }

      last_distance_remaining_ = current_distance;
      last_feedback_time_ = now;
    }
  }

  // MarkWaypointVisited Implementation
  BT::NodeStatus MarkWaypointVisited::tick()
  {
    int waypoint_id = -1;
    if (!getInput("waypoint_id", waypoint_id)) {
      RCLCPP_ERROR(rclcpp::get_logger("MarkWaypointVisited"), "waypoint_id not provided");
      return BT::NodeStatus::FAILURE;
    }

    // Get waypoints from blackboard and mark as visited
    try {
      auto waypoints = config().blackboard->get<std::vector<Waypoint>>("waypoints");
      for (auto& wp : waypoints) {
        if (wp.id == waypoint_id) {
          wp.visited = true;
          RCLCPP_INFO(rclcpp::get_logger("MarkWaypointVisited"),
            "Marked waypoint %d '%s' as visited", wp.id, wp.text.c_str());
          break;
        }
      }
      // Update blackboard
      config().blackboard->set("waypoints", waypoints);
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("MarkWaypointVisited"),
        "Failed to access waypoints from blackboard: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

  // IncrementWaypointIndex Implementation
  BT::NodeStatus IncrementWaypointIndex::tick()
  {
    try {
      int current_index;



      if (!getInput("current_waypoint_index", current_index)) {
        RCLCPP_ERROR(rclcpp::get_logger("IncrementWaypointIndex"),
          "Failed to get current_waypoint_index input");
        return BT::NodeStatus::FAILURE;
      }
      // Debug: log before increment and blackboard value
      int bb_value = 0;
      try { bb_value = config().blackboard->get<int>("current_waypoint_index"); }
      catch (...) {}
      RCLCPP_INFO(rclcpp::get_logger("IncrementWaypointIndex"),
        "IncrementWaypointIndex: before increment, input current_index = %d, blackboard current_waypoint_index = %d",
        current_index, bb_value);
      // increment
      current_index++;
      // write back to blackboard and output port
      config().blackboard->set("current_waypoint_index", current_index);
      setOutput("current_waypoint_index", current_index);
      // Debug: log after increment and blackboard value
      try { bb_value = config().blackboard->get<int>("current_waypoint_index"); }
      catch (...) {}
      RCLCPP_INFO(rclcpp::get_logger("IncrementWaypointIndex"),
        "IncrementWaypointIndex: after increment, new current_index = %d, blackboard current_waypoint_index = %d",
        current_index, bb_value);

    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("IncrementWaypointIndex"),
        "Failed to increment waypoint index: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

} // namespace perimeter_roamer_v3
