/**
 * @file imu_monitor_node.cpp
 * @brief IMU monitor node for TeensyV2 dual BNO055 sensor system
 * 
 * This node subscribes to IMU messages from the TeensyV2 bridge and publishes
 * ROS2 sensor_msgs::msg::Imu messages for the dual BNO055 sensor system.
 * 
 * @author Sigyn Robotics
 * @date 2025
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "sigyn_to_sensor_v2/message_parser.h"

using namespace sigyn_to_sensor_v2;

class IMUMonitorNode : public rclcpp::Node {
public:
  IMUMonitorNode() : Node("imu_monitor"), parser_(this->get_logger()) {
    // Create publishers for each IMU sensor
    imu_publishers_[0] = this->create_publisher<sensor_msgs::msg::Imu>("imu/sensor_0", 10);
    imu_publishers_[1] = this->create_publisher<sensor_msgs::msg::Imu>("imu/sensor_1", 10);
    
    // Create diagnostic publisher
    diagnostic_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "diagnostics", 10);
    
    // Register callback for IMU messages
    parser_.RegisterCallback(MessageType::IMU, 
        std::bind(&IMUMonitorNode::HandleIMUMessage, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    // Create diagnostic timer
    diagnostic_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&IMUMonitorNode::PublishDiagnostics, this));
    
    RCLCPP_INFO(this->get_logger(), "IMU Monitor Node initialized");
  }

  /**
   * @brief Process incoming message from TeensyV2 bridge
   * 
   * @param message Raw message string from Teensy
   */
  void ProcessMessage(const std::string& message) {
    parser_.ParseMessage(message, this->get_clock()->now());
  }

private:
  /**
   * @brief Handle parsed IMU message
   * 
   * @param data Parsed IMU data
   * @param timestamp Message timestamp
   */
  void HandleIMUMessage(const MessageData& data, rclcpp::Time timestamp) {
    IMUData imu_data = parser_.ParseIMUData(data);
    
    if (!imu_data.valid) {
      RCLCPP_WARN(this->get_logger(), "Invalid IMU data received");
      return;
    }
    
    // Validate sensor ID
    if (imu_data.sensor_id < 0 || imu_data.sensor_id >= 2) {
      RCLCPP_WARN(this->get_logger(), "Invalid IMU sensor ID: %d", imu_data.sensor_id);
      return;
    }
    
    // Convert to ROS message and publish
    sensor_msgs::msg::Imu imu_msg = parser_.ToImuMsg(imu_data, timestamp);
    imu_publishers_[imu_data.sensor_id]->publish(imu_msg);
    
    // Update statistics
    message_counts_[imu_data.sensor_id]++;
    last_message_time_[imu_data.sensor_id] = timestamp;
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "Published IMU data for sensor %d: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                 imu_data.sensor_id, imu_data.qx, imu_data.qy, imu_data.qz, imu_data.qw);
  }

  /**
   * @brief Publish diagnostic information
   */
  void PublishDiagnostics() {
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = this->get_clock()->now();
    
    // Create diagnostic status for each sensor
    for (int i = 0; i < 2; i++) {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "imu_sensor_" + std::to_string(i);
      status.hardware_id = "teensy_v2_bno055_" + std::to_string(i);
      
      // Check if we've received recent messages
      auto now = this->get_clock()->now();
      auto time_since_last = now - last_message_time_[i];
      
      if (message_counts_[i] == 0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "No messages received";
      } else if (time_since_last.seconds() > 5.0) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Messages stale (last: " + std::to_string(time_since_last.seconds()) + "s ago)";
      } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "Operating normally";
      }
      
      // Add values
      diagnostic_msgs::msg::KeyValue kv;
      
      kv.key = "message_count";
      kv.value = std::to_string(message_counts_[i]);
      status.values.push_back(kv);
      
      kv.key = "last_message_age_s";
      kv.value = std::to_string(time_since_last.seconds());
      status.values.push_back(kv);
      
      diag_array.status.push_back(status);
    }
    
    diagnostic_pub_->publish(diag_array);
  }

  // Member variables
  MessageParser parser_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publishers_[2];
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  
  // Statistics
  uint64_t message_counts_[2] = {0, 0};
  rclcpp::Time last_message_time_[2] = {rclcpp::Time(0), rclcpp::Time(0)};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<IMUMonitorNode>();
  
  RCLCPP_INFO(node->get_logger(), "Starting IMU Monitor Node");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
