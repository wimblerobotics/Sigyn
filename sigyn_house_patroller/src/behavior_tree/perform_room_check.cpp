#include "sigyn_house_patroller/behavior_tree/perform_room_check.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sigyn_house_patroller {

PerformRoomCheck::PerformRoomCheck(const std::string& xml_tag_name, 
                                   const BT::NodeConfiguration& conf)
    : BT::StatefulActionNode(xml_tag_name, conf) {
  
  // Get node from blackboard
  getInput("node", node_);
  if (!node_) {
    throw BT::RuntimeError("PerformRoomCheck requires a 'node' input");
  }
  
  // Create subscriptions for sensor data
  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", rclcpp::QoS(10).reliable(),
    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_image_ = msg;
      image_received_ = true;
    });
  
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/points", rclcpp::QoS(10).reliable(),
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_cloud_ = msg;
      cloud_received_ = true;
    });
  
  // Get parameters
  getInput("room_name", room_name_);
  getInput("check_duration", check_duration_);
  getInput("min_observations", min_observations_);
  
  RCLCPP_INFO(node_->get_logger(), "PerformRoomCheck initialized for room: %s", 
              room_name_.c_str());
}

BT::NodeStatus PerformRoomCheck::onStart() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // Reset state
  image_received_ = false;
  cloud_received_ = false;
  observation_count_ = 0;
  anomalies_detected_ = 0;
  check_complete_ = false;
  
  start_time_ = std::chrono::steady_clock::now();
  
  RCLCPP_INFO(node_->get_logger(), "PerformRoomCheck: Starting room check for %s", 
              room_name_.c_str());
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PerformRoomCheck::onRunning() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_).count();
  
  // Check if we have sensor data
  if (image_received_ && cloud_received_) {
    observation_count_++;
    
    // Simple anomaly detection (placeholder)
    if (AnalyzeImageForAnomalies() || AnalyzeCloudForAnomalies()) {
      anomalies_detected_++;
    }
    
    // Reset flags for next observation
    image_received_ = false;
    cloud_received_ = false;
  }
  
  // Check completion conditions
  bool time_complete = elapsed >= check_duration_;
  bool observations_complete = observation_count_ >= min_observations_;
  
  if (time_complete || observations_complete) {
    check_complete_ = true;
    
    // Calculate results
    double anomaly_rate = (observation_count_ > 0) ? 
                         (double)anomalies_detected_ / observation_count_ : 0.0;
    
    // Set output values
    setOutput("room_checked", true);
    setOutput("anomaly_rate", anomaly_rate);
    setOutput("observation_count", observation_count_);
    setOutput("anomalies_detected", anomalies_detected_);
    setOutput("check_duration", static_cast<double>(elapsed));
    
    RCLCPP_INFO(node_->get_logger(), 
                "PerformRoomCheck: Completed room check for %s - "
                "observations: %d, anomalies: %d, rate: %.2f", 
                room_name_.c_str(), observation_count_, anomalies_detected_, anomaly_rate);
    
    // Return success if no significant anomalies
    return (anomaly_rate < 0.3) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  
  // Continue checking
  return BT::NodeStatus::RUNNING;
}

void PerformRoomCheck::onHalted() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  RCLCPP_INFO(node_->get_logger(), "PerformRoomCheck: Room check halted for %s", 
              room_name_.c_str());
  
  check_complete_ = false;
}

bool PerformRoomCheck::AnalyzeImageForAnomalies() {
  if (!last_image_) {
    return false;
  }
  
  // Placeholder for actual image analysis
  // In a real implementation, this would use computer vision
  // to detect changes, objects, or other anomalies
  
  // For now, randomly detect anomalies 10% of the time
  static int call_count = 0;
  call_count++;
  
  return (call_count % 10 == 0);
}

bool PerformRoomCheck::AnalyzeCloudForAnomalies() {
  if (!last_cloud_) {
    return false;
  }
  
  // Placeholder for actual point cloud analysis
  // In a real implementation, this would use PCL
  // to detect structural changes or new objects
  
  // For now, randomly detect anomalies 15% of the time
  static int call_count = 0;
  call_count++;
  
  return (call_count % 7 == 0);
}

}  // namespace sigyn_house_patroller

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<sigyn_house_patroller::PerformRoomCheck>("PerformRoomCheck");
}
