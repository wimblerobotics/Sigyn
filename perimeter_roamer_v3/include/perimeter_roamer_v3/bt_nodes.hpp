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

#ifndef PERIMETER_ROAMER_V3__BT_NODES_HPP_
#define PERIMETER_ROAMER_V3__BT_NODES_HPP_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <sqlite3.h>
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <algorithm>

namespace perimeter_roamer_v3
{

enum class SpaceType {
  UNKNOWN,
  ROOM,
  HALLWAY,
  DOORWAY,
  VERY_NARROW
};

struct Wall {
  double start_x, start_y;
  double length, width;
  double angle; // orientation in radians
};

/**
 * @brief Base class for BT nodes that need ROS 2 node access
 */
class RosNodeBT {
protected:
  std::shared_ptr<rclcpp::Node> node_;
  
public:
  void setRosNode(std::shared_ptr<rclcpp::Node> node) {
    node_ = node;
  }
};

/**
 * @brief Custom behavior tree node to check battery state
 */
class CheckBatteryState : public BT::ConditionNode, public RosNodeBT
{
public:
  CheckBatteryState(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config), battery_level_(100.0f), last_battery_time_(0), startup_time_(0)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<float>("low_battery_threshold", 20.0f, "Battery percentage threshold for low battery")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  float battery_level_;
  rclcpp::Time last_battery_time_;
  rclcpp::Time startup_time_;
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
};

/**
 * @brief Check LIDAR health by analyzing scan data
 */
class CheckLidarHealth : public BT::ConditionNode, public RosNodeBT
{
public:
  CheckLidarHealth(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config), last_scan_time_(0), startup_time_(0)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<float>("max_invalid_ratio", 0.5f, "Maximum ratio of invalid readings allowed"),
      BT::InputPort<float>("scan_timeout", 2.0f, "Timeout for scan data in seconds")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  rclcpp::Time last_scan_time_;
  rclcpp::Time startup_time_;
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

/**
 * @brief Classify the current space based on LIDAR data and wall database
 */
class ClassifySpace : public BT::SyncActionNode, public RosNodeBT
{
public:
  ClassifySpace(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config), db_(nullptr)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::string>("space_type", "Classified space type"),
      BT::OutputPort<geometry_msgs::msg::Pose>("suggested_pose", "Suggested next pose")
    };
  }

  BT::NodeStatus tick() override;

private:
  sqlite3* db_;
  std::vector<Wall> walls_;
  sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  
  bool loadWallDatabase();
  SpaceType analyzeSpace(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  geometry_msgs::msg::Pose generateNextPose(SpaceType space_type);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

/**
 * @brief Navigate to a specific pose using Nav2
 */
class NavigateToPose : public BT::StatefulActionNode, public RosNodeBT
{
public:
  NavigateToPose(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : BT::StatefulActionNode(xml_tag_name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Pose>("target_pose", "Target pose to navigate to"),
      BT::InputPort<std::string>("behavior_tree", "Behavior tree for navigation"),
      BT::InputPort<float>("timeout", 30.0f, "Timeout for navigation in seconds")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using NavigateAction = nav2_msgs::action::NavigateToPose;
  
  rclcpp_action::Client<NavigateAction>::SharedPtr action_client_;
  rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr goal_handle_;
  std::shared_future<rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr> goal_handle_future_;
  
  bool result_received_ = false;
  BT::NodeStatus navigation_result_ = BT::NodeStatus::FAILURE;

  void goalResponseCallback(const rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr& goal_handle);
  void resultCallback(const rclcpp_action::ClientGoalHandle<NavigateAction>::WrappedResult& result);
};

/**
 * @brief Condition nodes for different space types
 */
class IsRoom : public BT::ConditionNode
{
public:
  IsRoom(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("space_type", "Current space type from ClassifySpace")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string space_type;
    if (getInput("space_type", space_type) && space_type == "ROOM") {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

class IsHallway : public BT::ConditionNode
{
public:
  IsHallway(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("space_type", "Current space type from ClassifySpace")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string space_type;
    if (getInput("space_type", space_type) && space_type == "HALLWAY") {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

class IsDoorway : public BT::ConditionNode
{
public:
  IsDoorway(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("space_type", "Current space type from ClassifySpace")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string space_type;
    if (getInput("space_type", space_type) && space_type == "DOORWAY") {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

class IsVeryNarrow : public BT::ConditionNode
{
public:
  IsVeryNarrow(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("space_type", "Current space type from ClassifySpace")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string space_type;
    if (getInput("space_type", space_type) && space_type == "VERY_NARROW") {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

} // namespace perimeter_roamer_v3

#endif // PERIMETER_ROAMER_V3__BT_NODES_HPP_
