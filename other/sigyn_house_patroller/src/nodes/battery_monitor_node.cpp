#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "sigyn_house_patroller/msg/threat_alert.hpp"
#include "sigyn_house_patroller/msg/system_health.hpp"

namespace sigyn_house_patroller {

class BatteryMonitorNode : public rclcpp::Node {
public:
  explicit BatteryMonitorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("battery_monitor", options),
        last_battery_update_(std::chrono::steady_clock::now()) {
    
    // Declare parameters
    declare_parameter("critical_battery_level", 0.15);
    declare_parameter("low_battery_level", 0.25);
    declare_parameter("monitoring_frequency", 2.0);
    declare_parameter("battery_timeout", 30.0);
    declare_parameter("battery_topic", "/battery_state");
    declare_parameter("enable_predictions", true);
    declare_parameter("prediction_window", 300.0);  // 5 minutes
    
    // Get parameters
    critical_level_ = get_parameter("critical_battery_level").as_double();
    low_level_ = get_parameter("low_battery_level").as_double();
    monitoring_frequency_ = get_parameter("monitoring_frequency").as_double();
    battery_timeout_ = get_parameter("battery_timeout").as_double();
    battery_topic_ = get_parameter("battery_topic").as_string();
    enable_predictions_ = get_parameter("enable_predictions").as_bool();
    prediction_window_ = get_parameter("prediction_window").as_double();
    
    // Initialize state
    current_battery_level_ = 1.0;
    is_charging_ = false;
    battery_healthy_ = true;
    
    // Publishers
    threat_alert_pub_ = create_publisher<msg::ThreatAlert>(
      "~/threat_alerts", rclcpp::QoS(10).reliable());
    
    battery_status_pub_ = create_publisher<std_msgs::msg::Float64>(
      "~/battery_level", rclcpp::QoS(10).reliable());
    
    health_pub_ = create_publisher<msg::SystemHealth>(
      "~/health", rclcpp::QoS(10).reliable());
    
    // Subscribers
    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      battery_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&BatteryMonitorNode::BatteryCallback, this, std::placeholders::_1));
    
    // Timers
    monitoring_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / monitoring_frequency_)),
      std::bind(&BatteryMonitorNode::MonitoringTimerCallback, this));
    
    health_timer_ = create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&BatteryMonitorNode::HealthTimerCallback, this));
    
    RCLCPP_INFO(get_logger(), "Battery monitor started - Critical: %.1f%%, Low: %.1f%%", 
                critical_level_ * 100, low_level_ * 100);
  }

private:
  void BatteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(battery_mutex_);
    
    current_battery_state_ = *msg;
    current_battery_level_ = msg->percentage;
    is_charging_ = (msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
    last_battery_update_ = std::chrono::steady_clock::now();
    
    // Add to history for prediction
    if (enable_predictions_) {
      BatteryReading reading;
      reading.timestamp = std::chrono::steady_clock::now();
      reading.level = current_battery_level_;
      reading.is_charging = is_charging_;
      reading.voltage = msg->voltage;
      reading.current = msg->current;
      
      battery_history_.push_back(reading);
      
      // Keep only recent history
      auto cutoff_time = std::chrono::steady_clock::now() - 
                        std::chrono::seconds(static_cast<int>(prediction_window_));
      battery_history_.erase(
        std::remove_if(battery_history_.begin(), battery_history_.end(),
                      [cutoff_time](const BatteryReading& reading) {
                        return reading.timestamp < cutoff_time;
                      }),
        battery_history_.end());
    }
    
    // Publish current battery level
    std_msgs::msg::Float64 level_msg;
    level_msg.data = current_battery_level_;
    battery_status_pub_->publish(level_msg);
    
    // Check for immediate threats
    CheckBatteryThreats();
  }
  
  void MonitoringTimerCallback() {
    if (!battery_healthy_) {
      if (last_threat_time_ == rclcpp::Time(0, 0, RCL_ROS_TIME) ||
          (this->now() - last_threat_time_).seconds() > 60) {
        msg::ThreatAlert alert;
        alert.header.stamp = this->now();
        alert.threat_id = "battery_monitor_unhealthy_" + std::to_string(this->now().seconds());
        alert.threat_type = "battery_monitor_unhealthy";
        alert.severity_level = 2; // WARNING
        alert.description = "Battery monitor is not receiving updates.";
        alert.room_name = "System";
        alert.sensor_data_json = "{}";
        threat_alert_pub_->publish(alert);
        last_threat_time_ = this->now();
      }
    }
    
    // Check if we've lost battery data
    auto now = std::chrono::steady_clock::now();
    auto time_since_update = std::chrono::duration_cast<std::chrono::seconds>(
      now - last_battery_update_).count();
    
    if (time_since_update > battery_timeout_) {
      battery_healthy_ = false;
      
      // Send communication loss alert
      msg::ThreatAlert alert;
      alert.header.stamp = this->now();
      alert.threat_id = "battery_comm_loss";
      alert.threat_type = "communication_failure";
      alert.severity_level = 2; // WARNING
      alert.description = "Battery communication lost for " + std::to_string(time_since_update) + " seconds";
      alert.room_name = "System";
      alert.sensor_data_json = "{}";
      
      threat_alert_pub_->publish(alert);
      
      RCLCPP_ERROR(get_logger(), "Battery communication lost for %ld seconds", time_since_update);
    } else {
      battery_healthy_ = true;
    }
    
    // Perform battery prediction if enabled
    if (enable_predictions_ && !battery_history_.empty()) {
      PerformBatteryPrediction();
    }
  }
  
  void CheckBatteryThreats() {
    if (is_charging_) {
      return;  // No threats while charging
    }
    
    // Critical battery level
    if (current_battery_level_ < critical_level_) {
      if (last_critical_threat_time_ == rclcpp::Time(0, 0, RCL_ROS_TIME) ||
          (this->now() - last_critical_threat_time_).seconds() > 300) {
        msg::ThreatAlert alert;
        alert.header.stamp = this->now();
        alert.threat_id = "battery_critical_" + std::to_string(this->now().seconds());
        alert.threat_type = "battery_critical";
        alert.severity_level = 3; // CRITICAL
        alert.description = "Battery level is critically low.";
        alert.room_name = "System";
        alert.sensor_data_json = "{\"level\": " + std::to_string(current_battery_level_) +
                                  "}";
        threat_alert_pub_->publish(alert);
        last_critical_threat_time_ = this->now();
      }
    } else if (current_battery_level_ < low_level_) {
      if (last_low_threat_time_ == rclcpp::Time(0, 0, RCL_ROS_TIME) ||
          (this->now() - last_low_threat_time_).seconds() > 600) {
        msg::ThreatAlert alert;
        alert.header.stamp = this->now();
        alert.threat_id = "battery_low_" + std::to_string(this->now().seconds());
        alert.threat_type = "battery_low";
        alert.severity_level = 2; // WARNING
        alert.description = "Battery level is low.";
        alert.room_name = "System";
        alert.sensor_data_json = "{\"level\": " + std::to_string(current_battery_level_) + "}";
        threat_alert_pub_->publish(alert);
        last_low_threat_time_ = this->now();
      }
    }
  }
  
  void PerformBatteryPrediction() {
    if (battery_history_.size() < 2) {
      return;
    }
    
    // Calculate discharge rate (simple linear regression)
    double sum_time = 0.0, sum_level = 0.0, sum_time_level = 0.0, sum_time_sq = 0.0;
    int count = 0;
    
    auto base_time = battery_history_[0].timestamp;
    
    for (const auto& reading : battery_history_) {
      if (!reading.is_charging) {  // Only consider discharging periods
        double time_seconds = std::chrono::duration_cast<std::chrono::seconds>(
          reading.timestamp - base_time).count();
        
        sum_time += time_seconds;
        sum_level += reading.level;
        sum_time_level += time_seconds * reading.level;
        sum_time_sq += time_seconds * time_seconds;
        count++;
      }
    }
    
    if (count < 2) {
      return;
    }
    
    // Calculate discharge rate (level/second)
    double discharge_rate = (count * sum_time_level - sum_time * sum_level) / 
                           (count * sum_time_sq - sum_time * sum_time);
    
    if (discharge_rate >= 0) {
      return;  // Battery not discharging or rate calculation error
    }
    
    // Predict time to critical level
    double time_to_critical = (current_battery_level_ - critical_level_) / (-discharge_rate);
    
    // Alert if we'll hit critical level soon
    if (time_to_critical > 0 && time_to_critical < prediction_window_) {
      if (last_prediction_threat_time_ == rclcpp::Time(0, 0, RCL_ROS_TIME) ||
          (this->now() - last_prediction_threat_time_).seconds() > 600) {
        msg::ThreatAlert alert;
        alert.header.stamp = this->now();
        alert.threat_id = "battery_prediction_" + std::to_string(this->now().seconds());
        alert.threat_type = "battery_prediction";
        alert.severity_level = 2; // WARNING
        alert.description = "Predicted to reach critical battery level soon.";
        alert.room_name = "System";
        alert.sensor_data_json = "{\"predicted_time\": " + std::to_string(time_to_critical) +
                                  "}";
        threat_alert_pub_->publish(alert);
        last_prediction_threat_time_ = this->now();
      }
    }
  }
  
  void HealthTimerCallback() {
    msg::SystemHealth health;
    health.header.stamp = this->now();
    health.overall_health = battery_healthy_ ? 1 : 2; // HEALTHY : DEGRADED
    health.system_status_description = battery_healthy_ ? "OK" : "Battery monitor not receiving updates";

    health.component_names.push_back("battery_monitor");
    health.component_health_status.push_back(battery_healthy_ ? 1 : 2); // HEALTHY : DEGRADED
    health.component_descriptions.push_back(std::string("battery_monitor: ") + (battery_healthy_ ? "OK" : "ERROR"));
    health.last_update_times.push_back(this->now());

    health.component_names.push_back("battery_level");
    health.component_health_status.push_back(1); // HEALTHY
    health.component_descriptions.push_back("battery_level: " + std::to_string(current_battery_level_));
    health.last_update_times.push_back(this->now());

    health.component_names.push_back("charging_state");
    health.component_health_status.push_back(1); // HEALTHY
    health.component_descriptions.push_back("charging: " + std::string(is_charging_ ? "YES" : "NO"));
    health.last_update_times.push_back(this->now());

    health_pub_->publish(health);
  }
  
  struct BatteryReading {
    std::chrono::steady_clock::time_point timestamp;
    double level;
    bool is_charging;
    double voltage;
    double current;
  };
  
  // Parameters
  double critical_level_;
  double low_level_;
  double monitoring_frequency_;
  double battery_timeout_;
  std::string battery_topic_;
  bool enable_predictions_;
  double prediction_window_;
  
  // State
  double current_battery_level_;
  bool is_charging_;
  bool battery_healthy_;
  sensor_msgs::msg::BatteryState current_battery_state_;
  std::chrono::steady_clock::time_point last_battery_update_;
  std::vector<BatteryReading> battery_history_;
  rclcpp::Time last_threat_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_critical_threat_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_low_threat_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_prediction_threat_time_{0, 0, RCL_ROS_TIME};
  
  // ROS2 interfaces
  rclcpp::Publisher<msg::ThreatAlert>::SharedPtr threat_alert_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_status_pub_;
  rclcpp::Publisher<msg::SystemHealth>::SharedPtr health_pub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::TimerBase::SharedPtr monitoring_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  
  std::mutex battery_mutex_;
};

}  // namespace sigyn_house_patroller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<sigyn_house_patroller::BatteryMonitorNode>();
  
  RCLCPP_INFO(rclcpp::get_logger("battery_monitor"), "Starting battery monitor node");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
