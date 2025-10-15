// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

#include <memory>
#include <string>
#include <chrono>
#include <signal.h>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "perimeter_roamer_v3/bt_nodes.hpp"

using namespace std::chrono_literals;

// Global flag for signal handling
std::atomic<bool> g_shutdown_requested{false};

void signal_handler(int signal) {
  (void)signal;
  g_shutdown_requested = true;
  RCLCPP_INFO(rclcpp::get_logger("perimeter_roamer"), "Shutdown requested");
}

namespace perimeter_roamer_v3
{

class PerimeterRoamerNode : public rclcpp::Node
{
public:
  PerimeterRoamerNode()
  : Node("perimeter_roamer_node")
  {
    this->declare_parameter<std::string>("bt_xml_filename", "");
    this->declare_parameter<std::string>("waypoint_database_path", "data/patrol_waypoints.db");
    this->declare_parameter<bool>("loop_waypoints", false);
    
    std::string bt_xml_filename = this->get_parameter("bt_xml_filename").as_string();
    std::string waypoint_db_path = this->get_parameter("waypoint_database_path").as_string();
    bool loop_waypoints = this->get_parameter("loop_waypoints").as_bool();

    // Convert relative path to absolute path relative to package directory
    if (waypoint_db_path.find("/") != 0) {  // Not an absolute path
      // Get package directory from ROS parameter server or construct it
      std::string pkg_path = "/home/ros/sigyn_ws/src/Sigyn/perimeter_roamer_v3/";
      waypoint_db_path = pkg_path + waypoint_db_path;
    }

    if (bt_xml_filename.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'bt_xml_filename' is not set.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loading Behavior Tree from: %s", bt_xml_filename.c_str());
    RCLCPP_INFO(this->get_logger(), "Waypoint database path: %s", waypoint_db_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Loop waypoints: %s", loop_waypoints ? "true" : "false");

    // Create main node subscriptions to ensure callbacks are processed
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&PerimeterRoamerNode::scanCallback, this, std::placeholders::_1));
    
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/sigyn/teensy_bridge/battery/status", 10,
      std::bind(&PerimeterRoamerNode::batteryCallback, this, std::placeholders::_1));

    BT::BehaviorTreeFactory factory;

    // Create shared pointer to this node for BT nodes that need ROS access
    auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});

    // --- Register our custom nodes ---
    factory.registerNodeType<CheckBatteryState>("CheckBatteryState");
    factory.registerNodeType<CheckLidarHealth>("CheckLidarHealth");
    factory.registerNodeType<ClassifySpace>("ClassifySpace");
    factory.registerNodeType<NavigateToPose>("NavigateToPose");
    factory.registerNodeType<CheckGoalInterrupted>("CheckGoalInterrupted");
    factory.registerNodeType<IsRoom>("IsRoom");
    factory.registerNodeType<IsHallway>("IsHallway");
    factory.registerNodeType<IsDoorway>("IsDoorway");
    factory.registerNodeType<IsVeryNarrow>("IsVeryNarrow");
    
    // Waypoint following nodes
    factory.registerNodeType<LoadWaypoints>("LoadWaypoints");
    factory.registerNodeType<CheckWaypointsComplete>("CheckWaypointsComplete");
    factory.registerNodeType<GetNextWaypoint>("GetNextWaypoint");
    factory.registerNodeType<NavigateToWaypoint>("NavigateToWaypoint");
    factory.registerNodeType<MarkWaypointVisited>("MarkWaypointVisited");
    factory.registerNodeType<IncrementWaypointIndex>("IncrementWaypointIndex");

    // Create the tree
    tree_ = factory.createTreeFromFile(bt_xml_filename);

    // Set the ROS node for all BT nodes that need it
    for (auto& node : tree_.nodes) {
      auto ros_bt_node = dynamic_cast<RosNodeBT*>(node.get());
      if (ros_bt_node) {
        ros_bt_node->setRosNode(node_ptr);
      }
    }

    // Store sensor data in blackboard for BT nodes
    auto blackboard = tree_.rootBlackboard();
    blackboard->set("scan_data", last_scan_);
    blackboard->set("scan_time", last_scan_time_);
    blackboard->set("battery_level", battery_level_);
    blackboard->set("battery_time", last_battery_time_);
    blackboard->set("waypoint_database_path", waypoint_db_path);
    blackboard->set("loop_waypoints", loop_waypoints);
    
    // Initialize waypoint following state
    blackboard->set("current_waypoint_index", 0);  // Start from first waypoint
    blackboard->set("total_waypoints", 0);  // Will be set by LoadWaypoints
    blackboard->set("waypoints_complete", false);
    
    // Set a default charging pose (for demo - in real robot this would be loaded from config)
    geometry_msgs::msg::Pose charging_pose;
    charging_pose.position.x = 11.0;  // Return to origin for charging
    charging_pose.position.y = 9.0;
    charging_pose.position.z = 0.0;
    charging_pose.orientation.w = 1.0;
    blackboard->set("charging_pose", charging_pose);

    // Add a logger to print the tree status to the console
    logger_ = std::make_unique<BT::StdCoutLogger>(tree_);

    // Create a timer to tick the tree
    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&PerimeterRoamerNode::tick_tree, this));
  }

private:
  void tick_tree()
  {
    // Check if shutdown was requested
    if (g_shutdown_requested) {
      RCLCPP_INFO(this->get_logger(), "Shutdown requested, stopping behavior tree");
      timer_->cancel();
      return;
    }
    
    BT::NodeStatus status = tree_.tickRoot();
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Behavior Tree execution completed successfully. Continuing...");
      // Don't shutdown, keep running
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_DEBUG(this->get_logger(), "Behavior Tree tick returned FAILURE. Retrying...");
      // Don't shutdown, keep trying
    }
    // Only RUNNING status continues without logging
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_ = msg;
    last_scan_time_ = this->get_clock()->now();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Main node LIDAR callback: %zu ranges", msg->ranges.size());
    
    // Update blackboard
    if (tree_.rootBlackboard()) {
      tree_.rootBlackboard()->set("scan_data", last_scan_);
      tree_.rootBlackboard()->set("scan_time", last_scan_time_);
    }
  }

  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    // Only process battery messages from the main 36V LiFePO4 battery
    if (msg->header.frame_id != "36VLIPO") {
      RCLCPP_DEBUG(this->get_logger(), "Ignoring battery message from frame_id: %s", 
                   msg->header.frame_id.c_str());
      return;
    }
    
    battery_level_ = msg->percentage * 100.0f;
    last_battery_time_ = this->get_clock()->now();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Main node 36VLIPO battery callback: %.1f%% (%.1fV)", 
                         battery_level_, msg->voltage);
    
    // Update blackboard
    if (tree_.rootBlackboard()) {
      tree_.rootBlackboard()->set("battery_level", battery_level_);
      tree_.rootBlackboard()->set("battery_time", last_battery_time_);
    }
  }

  BT::Tree tree_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<BT::StdCoutLogger> logger_;
  
  // Sensor data
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  rclcpp::Time last_scan_time_;
  float battery_level_ = 100.0f;
  rclcpp::Time last_battery_time_;
};

}  // namespace perimeter_roamer_v3

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // Set up signal handlers
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  
  auto node = std::make_shared<perimeter_roamer_v3::PerimeterRoamerNode>();
  
  // Use regular single-threaded executor - this is often more reliable for BT integration
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  // Handle Ctrl+C gracefully with polling executor
  try {
    while (rclcpp::ok() && !g_shutdown_requested) {
      executor.spin_some(std::chrono::milliseconds(50));  // Reduced timeout for better responsiveness
      if (g_shutdown_requested) {
        break;
      }
    }
  } catch (const rclcpp::exceptions::RCLError & e) {
    RCLCPP_ERROR(node->get_logger(), "RCL error: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
  }
  
  RCLCPP_INFO(node->get_logger(), "Shutting down perimeter roamer node");
  
  // Force shutdown sequence
  executor.cancel();
  rclcpp::shutdown();
  return 0;
}