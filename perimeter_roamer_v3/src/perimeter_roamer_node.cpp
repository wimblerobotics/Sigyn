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

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "perimeter_roamer_v3/bt_nodes.hpp"

using namespace std::chrono_literals;

namespace perimeter_roamer_v3
{

class PerimeterRoamerNode : public rclcpp::Node
{
public:
  PerimeterRoamerNode()
  : Node("perimeter_roamer_node")
  {
    this->declare_parameter<std::string>("bt_xml_filename", "");
    std::string bt_xml_filename = this->get_parameter("bt_xml_filename").as_string();

    if (bt_xml_filename.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'bt_xml_filename' is not set.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loading Behavior Tree from: %s", bt_xml_filename.c_str());

    // Create main node subscriptions to ensure callbacks are processed
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&PerimeterRoamerNode::scanCallback, this, std::placeholders::_1));
    
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/battery_state", 10,
      std::bind(&PerimeterRoamerNode::batteryCallback, this, std::placeholders::_1));

    BT::BehaviorTreeFactory factory;

    // Create shared pointer to this node for BT nodes that need ROS access
    auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});

    // --- Register our custom nodes ---
    factory.registerNodeType<CheckBatteryState>("CheckBatteryState");
    factory.registerNodeType<CheckLidarHealth>("CheckLidarHealth");
    factory.registerNodeType<ClassifySpace>("ClassifySpace");
    factory.registerNodeType<NavigateToPose>("NavigateToPose");
    factory.registerNodeType<IsRoom>("IsRoom");
    factory.registerNodeType<IsHallway>("IsHallway");
    factory.registerNodeType<IsDoorway>("IsDoorway");
    factory.registerNodeType<IsVeryNarrow>("IsVeryNarrow");

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
    
    // Set a default charging pose (for demo - in real robot this would be loaded from config)
    geometry_msgs::msg::Pose charging_pose;
    charging_pose.position.x = 0.0;  // Return to origin for charging
    charging_pose.position.y = 0.0;
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
    battery_level_ = msg->percentage * 100.0f;
    last_battery_time_ = this->get_clock()->now();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Main node battery callback: %.1f%%", battery_level_);
    
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
  
  auto node = std::make_shared<perimeter_roamer_v3::PerimeterRoamerNode>();
  
  // Use regular single-threaded executor - this is often more reliable for BT integration
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}