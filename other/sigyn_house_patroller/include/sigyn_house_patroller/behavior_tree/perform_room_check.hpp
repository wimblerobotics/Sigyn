#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <chrono>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sigyn_house_patroller {

/**
 * @brief Behavior tree node to perform comprehensive room check
 */
class PerformRoomCheck : public BT::StatefulActionNode {
public:
  PerformRoomCheck(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);
  
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS2 node for subscriptions"),
      BT::InputPort<std::string>("room_name", "Name of the room to check"),
      BT::InputPort<double>("check_duration", 30.0, "Duration to perform check (seconds)"),
      BT::InputPort<int>("min_observations", 10, "Minimum number of observations"),
      BT::OutputPort<bool>("room_checked", "True if room check completed"),
      BT::OutputPort<double>("anomaly_rate", "Rate of anomalies detected"),
      BT::OutputPort<int>("observation_count", "Number of observations made"),
      BT::OutputPort<int>("anomalies_detected", "Number of anomalies detected"),
      BT::OutputPort<double>("check_duration", "Actual duration of check")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  
  sensor_msgs::msg::Image::SharedPtr last_image_;
  sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;
  
  std::string room_name_;
  double check_duration_;
  int min_observations_;
  
  bool image_received_;
  bool cloud_received_;
  int observation_count_;
  int anomalies_detected_;
  bool check_complete_;
  
  std::chrono::steady_clock::time_point start_time_;
  
  std::mutex data_mutex_;
  
  bool AnalyzeImageForAnomalies();
  bool AnalyzeCloudForAnomalies();
};

}  // namespace sigyn_house_patroller
