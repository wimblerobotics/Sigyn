// Copyright 2024 Wimble Robotics
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
    } catch (const std::exception&) {
      // Key not found or wrong type, use default
    }
    
    try {
      battery_time = blackboard->get<rclcpp::Time>("battery_time");
    } catch (const std::exception&) {
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
  } else {
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
    } catch (const std::exception&) {
      // Key not found or wrong type
    }
    
    try {
      scan_time = blackboard->get<rclcpp::Time>("scan_time");
    } catch (const std::exception&) {
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
  } catch (const std::exception& e) {
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

  RCLCPP_INFO(rclcpp::get_logger("ClassifySpace"), "Classified space as: %s", space_str.c_str());
  
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
    wall.angle = (wall.length > wall.width) ? 0.0 : M_PI/2.0;
    
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
    } else {
      return SpaceType::DOORWAY;
    }
  } else if (far_ratio > 0.3f && max_range > 3.0f) {
    // Open space - likely a room
    return SpaceType::ROOM;
  } else if (close_ratio > 0.4f && avg_range < 1.5f) {
    // Constrained but not very narrow - likely hallway
    return SpaceType::HALLWAY;
  } else if (far_ratio > 0.2f) {
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
BT::NodeStatus NavigateToPose::tick()
{
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "ROS node not set!");
    return BT::NodeStatus::FAILURE;
  }

  // Initialize action client if not already done
  if (!action_client_) {
    action_client_ = rclcpp_action::create_client<NavigateAction>(node_, "navigate_to_pose");
    
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "Action server not available!");
      return BT::NodeStatus::FAILURE;
    }
  }

  // If we haven't sent a goal yet, send it
  if (!goal_sent_) {
    geometry_msgs::msg::Pose target_pose;
    if (!getInput("target_pose", target_pose)) {
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "No target_pose provided!");
      return BT::NodeStatus::FAILURE;
    }

    auto goal_msg = NavigateAction::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node_->get_clock()->now();
    goal_msg.pose.pose = target_pose;

    // Optional: Set behavior tree
    std::string bt_xml;
    if (getInput("behavior_tree", bt_xml)) {
      goal_msg.behavior_tree = bt_xml;
    }

    auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = 
      std::bind(&NavigateToPose::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.result_callback = 
      std::bind(&NavigateToPose::resultCallback, this, std::placeholders::_1);

    goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
    goal_sent_ = true;
    
    RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "Sent navigation goal");
    return BT::NodeStatus::RUNNING;
  }

  // Check if goal is still running
  if (goal_handle_future_.valid()) {
    auto status = goal_handle_future_.wait_for(std::chrono::seconds(0));
    if (status != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    
    goal_handle_ = goal_handle_future_.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "Goal was rejected!");
      goal_sent_ = false;
      return BT::NodeStatus::FAILURE;
    }
  }

  // Goal is accepted, wait for completion
  if (goal_handle_) {
    auto status = goal_handle_->get_status();
    if (status == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
      RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "Navigation succeeded!");
      goal_sent_ = false;
      return BT::NodeStatus::SUCCESS;
    } else if (status == rclcpp_action::GoalStatus::STATUS_ABORTED ||
               status == rclcpp_action::GoalStatus::STATUS_CANCELED) {
      RCLCPP_WARN(rclcpp::get_logger("NavigateToPose"), "Navigation failed or was canceled!");
      goal_sent_ = false;
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void NavigateToPose::halt()
{
  if (goal_handle_) {
    RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "Canceling navigation goal");
    action_client_->async_cancel_goal(goal_handle_);
  }
  goal_sent_ = false;
}

void NavigateToPose::goalResponseCallback(const ActionClient::GoalHandle::SharedPtr& goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "Goal accepted by server");
  }
}

void NavigateToPose::resultCallback(const ActionClient::WrappedResult& result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(rclcpp::get_logger("NavigateToPose"), "Navigation completed successfully");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "Navigation was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(rclcpp::get_logger("NavigateToPose"), "Navigation was canceled");
      break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToPose"), "Unknown result code");
      break;
  }
}

} // namespace perimeter_roamer_v3
