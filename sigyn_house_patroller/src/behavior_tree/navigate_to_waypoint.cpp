#include "sigyn_house_patroller/behavior_tree/navigate_to_waypoint.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace sigyn_house_patroller {

NavigateToWaypoint::NavigateToWaypoint(const std::string& xml_tag_name, 
                                       const BT::NodeConfiguration& conf)
    : BT::StatefulActionNode(xml_tag_name, conf) {
  
  // Get node from blackboard
  getInput("node", node_);
  if (!node_) {
    throw BT::RuntimeError("NavigateToWaypoint requires a 'node' input");
  }
  
  // Create action client
  nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    node_, "/navigate_to_pose");
  
  // Get parameters
  getInput("waypoint_name", waypoint_name_);
  getInput("timeout_seconds", timeout_seconds_);
  
  RCLCPP_INFO(node_->get_logger(), "NavigateToWaypoint initialized for waypoint: %s", 
              waypoint_name_.c_str());
}

BT::NodeStatus NavigateToWaypoint::onStart() {
  // Get target pose
  geometry_msgs::msg::PoseStamped target_pose;
  if (!getInput("target_pose", target_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToWaypoint: target_pose not provided");
    return BT::NodeStatus::FAILURE;
  }
  
  // Wait for action server
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToWaypoint: Action server not available");
    return BT::NodeStatus::FAILURE;
  }
  
  // Prepare goal
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose = target_pose;
  
  // Send goal
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  
  send_goal_options.goal_response_callback = [this](auto goal_handle) {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "NavigateToWaypoint: Goal was rejected");
      navigation_result_ = BT::NodeStatus::FAILURE;
    } else {
      RCLCPP_INFO(node_->get_logger(), "NavigateToWaypoint: Goal accepted");
      goal_handle_ = goal_handle;
    }
  };
  
  send_goal_options.result_callback = [this](const auto& result) {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node_->get_logger(), "NavigateToWaypoint: Navigation succeeded");
        navigation_result_ = BT::NodeStatus::SUCCESS;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "NavigateToWaypoint: Navigation aborted");
        navigation_result_ = BT::NodeStatus::FAILURE;
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(node_->get_logger(), "NavigateToWaypoint: Navigation canceled");
        navigation_result_ = BT::NodeStatus::FAILURE;
        break;
      default:
        RCLCPP_ERROR(node_->get_logger(), "NavigateToWaypoint: Unknown result code");
        navigation_result_ = BT::NodeStatus::FAILURE;
        break;
    }
  };
  
  send_goal_options.feedback_callback = [this](auto goal_handle, const auto& feedback) {
    (void)goal_handle;
    std::lock_guard<std::mutex> lock(nav_mutex_);
    current_pose_ = feedback->current_pose;
    distance_remaining_ = feedback->distance_remaining;
    estimated_time_remaining_ = feedback->estimated_time_remaining;
  };
  
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    navigation_result_ = BT::NodeStatus::RUNNING;
    start_time_ = std::chrono::steady_clock::now();
  }
  
  nav_client_->async_send_goal(goal, send_goal_options);
  
  RCLCPP_INFO(node_->get_logger(), "NavigateToWaypoint: Started navigation to %s", 
              waypoint_name_.c_str());
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToWaypoint::onRunning() {
  std::lock_guard<std::mutex> lock(nav_mutex_);
  
  // Check timeout
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_).count();
  
  if (elapsed > timeout_seconds_) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToWaypoint: Navigation timeout");
    if (goal_handle_) {
      nav_client_->async_cancel_goal(goal_handle_);
    }
    return BT::NodeStatus::FAILURE;
  }
  
  // Set output values
  setOutput("current_pose", current_pose_);
  setOutput("distance_remaining", distance_remaining_);
  setOutput("elapsed_time", static_cast<double>(elapsed));
  
  return navigation_result_;
}

void NavigateToWaypoint::onHalted() {
  std::lock_guard<std::mutex> lock(nav_mutex_);
  
  if (goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "NavigateToWaypoint: Canceling navigation");
    nav_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
  }
  
  navigation_result_ = BT::NodeStatus::IDLE;
}

}  // namespace sigyn_house_patroller

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<sigyn_house_patroller::NavigateToWaypoint>("NavigateToWaypoint");
}
