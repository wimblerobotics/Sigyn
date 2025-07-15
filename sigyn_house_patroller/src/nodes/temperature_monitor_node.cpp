#include <memory>
#include <chrono>
#include <vector>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/float64.hpp>

#include "sigyn_house_patroller/msg/threat_alert.hpp"
#include "sigyn_house_patroller/msg/system_health.hpp"
#include "sigyn_house_patroller/msg/room_identification.hpp"

namespace sigyn_house_patroller {

class TemperatureMonitorNode : public rclcpp::Node {
public:
  explicit TemperatureMonitorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("temperature_monitor", options),
        last_temp_update_(std::chrono::steady_clock::now()) {
    
    // Declare parameters
    declare_parameter("anomaly_threshold", 5.0);  // degrees Celsius
    declare_parameter("monitoring_frequency", 1.0);
    declare_parameter("temperature_timeout", 60.0);
    declare_parameter("temperature_topic", "/temperature");
    declare_parameter("history_size", 100);
    declare_parameter("baseline_learning_samples", 50);
    declare_parameter("alert_cooldown", 300.0);  // 5 minutes
    declare_parameter("enable_trend_analysis", true);
    declare_parameter("trend_window", 600.0);  // 10 minutes
    
    // Room baseline temperatures (can be loaded from config)
    declare_parameter("room_baselines.living_room", 22.0);
    declare_parameter("room_baselines.kitchen", 20.0);
    declare_parameter("room_baselines.bedroom_1", 21.0);
    declare_parameter("room_baselines.bedroom_2", 20.0);
    declare_parameter("room_baselines.bathroom", 23.0);
    declare_parameter("room_baselines.hallway", 21.0);
    declare_parameter("room_baselines.entry", 20.0);
    
    // Get parameters
    anomaly_threshold_ = get_parameter("anomaly_threshold").as_double();
    monitoring_frequency_ = get_parameter("monitoring_frequency").as_double();
    temperature_timeout_ = get_parameter("temperature_timeout").as_double();
    temperature_topic_ = get_parameter("temperature_topic").as_string();
    history_size_ = get_parameter("history_size").as_int();
    baseline_learning_samples_ = get_parameter("baseline_learning_samples").as_int();
    alert_cooldown_ = get_parameter("alert_cooldown").as_double();
    enable_trend_analysis_ = get_parameter("enable_trend_analysis").as_bool();
    trend_window_ = get_parameter("trend_window").as_double();
    
    // Load room baselines
    LoadRoomBaselines();
    
    // Initialize state
    current_temperature_ = 20.0;
    current_room_ = "unknown";
    temperature_healthy_ = true;
    
    // Publishers
    threat_alert_pub_ = create_publisher<sigyn_house_patroller::msg::ThreatAlert>(
      "~/threat_alerts", rclcpp::QoS(10).reliable());
    
    temp_status_pub_ = create_publisher<std_msgs::msg::Float64>(
      "~/temperature", rclcpp::QoS(10).reliable());
    
    health_pub_ = create_publisher<sigyn_house_patroller::msg::SystemHealth>(
      "~/health", rclcpp::QoS(10).reliable());
    
    // Subscribers
    temperature_sub_ = create_subscription<sensor_msgs::msg::Temperature>(
      temperature_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&TemperatureMonitorNode::TemperatureCallback, this, std::placeholders::_1));
    
    room_id_sub_ = create_subscription<sigyn_house_patroller::msg::RoomIdentification>(
      "/sigyn_house_patroller/room_identification", rclcpp::QoS(10).reliable(),
      std::bind(&TemperatureMonitorNode::RoomIdentificationCallback, this, std::placeholders::_1));
    
    // Timers
    monitoring_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / monitoring_frequency_)),
      std::bind(&TemperatureMonitorNode::MonitoringTimerCallback, this));
    
    health_timer_ = create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&TemperatureMonitorNode::HealthTimerCallback, this));
    
    RCLCPP_INFO(get_logger(), "Temperature monitor started - Anomaly threshold: %.1f°C", 
                anomaly_threshold_);
  }

private:
  // Add missing member variables
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_anomaly_alerts_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_trend_alerts_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_comm_alerts_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_stale_alerts_;

  void TemperatureCallback(const sensor_msgs::msg::Temperature::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(temp_mutex_);
    
    current_temperature_ = msg->temperature;
    last_temp_update_ = std::chrono::steady_clock::now();
    
    // Add to history
    TemperatureReading reading;
    reading.timestamp = std::chrono::steady_clock::now();
    reading.temperature = current_temperature_;
    reading.room = current_room_;
    
    temperature_history_.push_back(reading);
    
    // Maintain history size
    if (temperature_history_.size() > history_size_) {
      temperature_history_.erase(temperature_history_.begin());
    }
    
    // Update room-specific history
    if (current_room_ != "unknown") {
      auto& room_history = room_histories_[current_room_];
      room_history.push_back(reading);
      
      if (room_history.size() > history_size_) {
        room_history.erase(room_history.begin());
      }
      
      // Update baseline if we're still learning
      if (room_history.size() <= baseline_learning_samples_) {
        UpdateRoomBaseline(current_room_);
      }
    }
    
    // Publish current temperature
    std_msgs::msg::Float64 temp_msg;
    temp_msg.data = current_temperature_;
    temp_status_pub_->publish(temp_msg);
    
    // Check for temperature anomalies
    CheckTemperatureThreats();
    
    // Perform trend analysis if enabled
    if (enable_trend_analysis_) {
      PerformTrendAnalysis();
    }
  }
  
  void RoomIdentificationCallback(const sigyn_house_patroller::msg::RoomIdentification::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(temp_mutex_);
    
    if (msg->confidence > 0.7) {  // Only update if confident
      current_room_ = msg->room_name;
    }
  }
  
  void MonitoringTimerCallback() {
    std::lock_guard<std::mutex> lock(temp_mutex_);
    auto now = std::chrono::steady_clock::now();

    for (const auto& [room_id, baseline] : room_baselines_) {
        (void)baseline; // baseline is not used, suppress warning
        auto it = room_histories_.find(room_id);
        if (it == room_histories_.end() || it->second.empty()) {
            continue;
        }

        const auto& last_reading = it->second.back();
        auto time_since_update = std::chrono::duration_cast<std::chrono::seconds>(now - last_reading.timestamp).count();

        if (time_since_update > temperature_timeout_) {
            auto& last_stale_alert_time = last_stale_alerts_[room_id];
            auto time_since_last_stale_alert = std::chrono::duration_cast<std::chrono::seconds>(now - last_stale_alert_time).count();

            if (time_since_last_stale_alert > alert_cooldown_) {
                sigyn_house_patroller::msg::ThreatAlert alert;
                alert.header.stamp = this->now();
                alert.threat_id = "temp_monitor_stale_" + room_id + "_" + std::to_string(this->now().seconds());
                alert.threat_type = "temp_monitor_stale";
                alert.severity_level = 2; // WARNING
                alert.description = "Temperature monitor for room " + room_id + " is not receiving updates. Last update was " + std::to_string(time_since_update) + "s ago.";
                alert.sensor_data_json = "{\"room\": \"" + room_id + "\"}";
                threat_alert_pub_->publish(alert);
                last_stale_alert_time = now;
            }
        }
    }
  }
  
  void CheckTemperatureThreats() {
    if (current_room_ == "unknown") {
      return;  // Can't check anomalies without room context
    }
    
    // Check if we have a baseline for this room
    auto baseline_it = room_baselines_.find(current_room_);
    if (baseline_it == room_baselines_.end()) {
      return;  // No baseline established yet
    }
    
    double baseline_temp = baseline_it->second;
    double temperature_diff = std::abs(current_temperature_ - baseline_temp);
    
    // Check for anomaly
    if (temperature_diff > anomaly_threshold_) {
      // Check alert cooldown
      auto now = std::chrono::steady_clock::now();
      auto& last_alert_time = last_anomaly_alerts_[current_room_];
      auto time_since_last_alert = std::chrono::duration_cast<std::chrono::seconds>(now - last_alert_time).count();

      if (time_since_last_alert > alert_cooldown_) {
        // Send threat alert
        sigyn_house_patroller::msg::ThreatAlert alert;
        alert.header.stamp = this->now();
        alert.threat_id = "temp_anomaly_" + current_room_ + "_" + std::to_string(this->now().seconds());
        alert.threat_type = "temperature_anomaly";
        alert.severity_level = 2; // WARNING
        alert.description = "Temperature anomaly detected in " + current_room_ + 
                           ". Current: " + std::to_string(current_temperature_) + 
                           "C, Baseline: " + std::to_string(baseline_temp) + "C.";
        alert.confidence = std::min(1.0, temperature_diff / (anomaly_threshold_ * 2));
        alert.sensor_data_json = "{\"room\": \"" + current_room_ + 
                              "\", \"current_temp\": " + std::to_string(current_temperature_) + 
                              ", \"baseline_temp\": " + std::to_string(baseline_temp) + "}";
        
        threat_alert_pub_->publish(alert);
        
        last_alert_time = now;
        
        RCLCPP_WARN(get_logger(), "Temperature anomaly in %s: %.1fC (Baseline: %.1fC)", 
                    current_room_.c_str(), current_temperature_, baseline_temp);
      }
    }
  }
  
  void PerformTrendAnalysis() {
    if (current_room_ == "unknown" || temperature_history_.size() < 10) {
      return;  // Need more data for trend analysis
    }
    
    // Calculate temperature trend over the trend window
    auto now = std::chrono::steady_clock::now();
    auto cutoff_time = now - std::chrono::seconds(static_cast<int>(trend_window_));
    
    std::vector<TemperatureReading> recent_readings;
    for (const auto& reading : temperature_history_) {
      if (reading.timestamp >= cutoff_time && reading.room == current_room_) {
        recent_readings.push_back(reading);
      }
    }
    
    if (recent_readings.size() < 5) {
      return;  // Not enough recent data
    }
    
    // Simple linear regression to find trend
    double sum_time = 0.0, sum_temp = 0.0, sum_time_temp = 0.0, sum_time_sq = 0.0;
    int count = recent_readings.size();
    
    auto base_time = recent_readings[0].timestamp;
    
    for (const auto& reading : recent_readings) {
      double time_seconds = std::chrono::duration_cast<std::chrono::seconds>(
        reading.timestamp - base_time).count();
      
      sum_time += time_seconds;
      sum_temp += reading.temperature;
      sum_time_temp += time_seconds * reading.temperature;
      sum_time_sq += time_seconds * time_seconds;
    }
    
    double slope = (count * sum_time_temp - sum_time * sum_temp) / 
                   (count * sum_time_sq - sum_time * sum_time);
    
    // Check for rapid temperature changes
    double temp_change_per_minute = slope * 60.0;
    
    if (std::abs(temp_change_per_minute) > 0.5) {  // More than 0.5°C per minute
      auto& last_trend_alert_time = last_trend_alerts_[current_room_];
      auto time_since_last_trend_alert = std::chrono::duration_cast<std::chrono::seconds>(now - last_trend_alert_time).count();

      if (time_since_last_trend_alert > alert_cooldown_) {
        sigyn_house_patroller::msg::ThreatAlert alert;
        alert.header.stamp = this->now();
        alert.threat_id = "temp_trend_" + current_room_ + "_" + std::to_string(this->now().seconds());
        alert.threat_type = "temperature_trend";
        alert.severity_level = 1; // INFO
        alert.description = "Rapid temperature increase detected in " + current_room_ + 
                           ". Trend: " + std::to_string(temp_change_per_minute) + " C/min.";
        alert.confidence = std::min(1.0, temp_change_per_minute / 1.0); // Normalize by 1.0 C/min trend
        alert.sensor_data_json = "{\"room\": \"" + current_room_ + 
                              "\", \"trend_slope\": " + std::to_string(temp_change_per_minute) + "}";
        
        threat_alert_pub_->publish(alert);
        
        last_trend_alerts_[current_room_] = now;
        
        RCLCPP_INFO(get_logger(), "Temperature trend detected in %s: %.2f C/min", 
                    current_room_.c_str(), temp_change_per_minute);
      }
    }
  }
  
  void LoadRoomBaselines() {
    std::vector<std::string> rooms = {"living_room", "kitchen", "bedroom_1", "bedroom_2", 
                                     "bathroom", "hallway", "entry"};
    
    for (const auto& room : rooms) {
      std::string param_name = "room_baselines." + room;
      if (has_parameter(param_name)) {
        room_baselines_[room] = get_parameter(param_name).as_double();
        RCLCPP_INFO(get_logger(), "Loaded baseline for %s: %.1f°C", 
                    room.c_str(), room_baselines_[room]);
      }
    }
  }
  
  void UpdateRoomBaseline(const std::string& room) {
    auto& room_history = room_histories_[room];
    
    if (room_history.empty()) {
      return;
    }
    
    // Calculate average temperature for this room
    double sum = 0.0;
    for (const auto& reading : room_history) {
      sum += reading.temperature;
    }
    
    room_baselines_[room] = sum / room_history.size();
    
    RCLCPP_INFO(get_logger(), "Updated baseline for %s: %.1f°C (samples: %zu)", 
                room.c_str(), room_baselines_[room], room_history.size());
  }
  
  void HealthTimerCallback() {
    std::lock_guard<std::mutex> lock(temp_mutex_);
    
    sigyn_house_patroller::msg::SystemHealth health_msg;
    health_msg.header.stamp = this->now();
    
    // Check overall temperature sensor health
    auto now = std::chrono::steady_clock::now();
    auto time_since_update = std::chrono::duration_cast<std::chrono::seconds>(now - last_temp_update_).count();
    
    if (time_since_update > temperature_timeout_) {
      temperature_healthy_ = false;
    } else {
      temperature_healthy_ = true;
    }
    
    health_msg.overall_health = temperature_healthy_ ? 1 : 2; // 1=HEALTHY, 2=DEGRADED
    health_msg.system_status_description = temperature_healthy_ ? 
                         "System nominal." : 
                         "System degraded.";

    health_msg.component_names.push_back(this->get_name());
    health_msg.component_health_status.push_back(temperature_healthy_ ? 1 : 3); // 1=HEALTHY, 3=UNHEALTHY
    health_msg.component_descriptions.push_back(temperature_healthy_ ? 
                         "Temperature sensor is online." : 
                         "Temperature sensor is offline. Last update " + std::to_string(time_since_update) + "s ago.");
    health_msg.last_update_times.push_back(rclcpp::Time(last_temp_update_.time_since_epoch().count()));

    health_pub_->publish(health_msg);
  }

  void SendCommunicationAlert(const std::string& sensor_name, double time_since_update) {
    auto now = std::chrono::steady_clock::now();
    auto& last_alert_time_ref = last_comm_alerts_[sensor_name];
    auto time_since_last_alert = std::chrono::duration_cast<std::chrono::seconds>(now - last_alert_time_ref).count();

    if (time_since_last_alert > alert_cooldown_) {
        sigyn_house_patroller::msg::ThreatAlert alert;
        alert.header.stamp = this->now();
        alert.threat_id = "comm_loss_" + sensor_name + "_" + std::to_string(this->now().seconds());
        alert.threat_type = "communication_loss";
        alert.severity_level = 3; // CRITICAL
        alert.description = "Communication lost with " + sensor_name + " sensor. No data for " + 
                           std::to_string(time_since_update) + " seconds.";
        alert.confidence = 1.0;
        alert.sensor_data_json = "{\"sensor\": \"" + sensor_name + "\"}";
        
        threat_alert_pub_->publish(alert);
        last_alert_time_ref = now;
    }
  }
  
  struct TemperatureReading {
    std::chrono::steady_clock::time_point timestamp;
    double temperature;
    std::string room;
  };
  
  // Parameters
  double anomaly_threshold_;
  double monitoring_frequency_;
  double temperature_timeout_;
  std::string temperature_topic_;
  int history_size_;
  int baseline_learning_samples_;
  double alert_cooldown_;
  bool enable_trend_analysis_;
  double trend_window_;
  
  // State
  double current_temperature_;
  std::string current_room_;
  bool temperature_healthy_;
  std::chrono::steady_clock::time_point last_temp_update_;
  
  // Data storage
  std::vector<TemperatureReading> temperature_history_;
  std::unordered_map<std::string, std::vector<TemperatureReading>> room_histories_;
  std::unordered_map<std::string, double> room_baselines_;
  
  // ROS2 interfaces
  rclcpp::Publisher<sigyn_house_patroller::msg::ThreatAlert>::SharedPtr threat_alert_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_status_pub_;
  rclcpp::Publisher<sigyn_house_patroller::msg::SystemHealth>::SharedPtr health_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temperature_sub_;
  rclcpp::Subscription<sigyn_house_patroller::msg::RoomIdentification>::SharedPtr room_id_sub_;
  rclcpp::TimerBase::SharedPtr monitoring_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  
  std::mutex temp_mutex_;
};

}  // namespace sigyn_house_patroller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<sigyn_house_patroller::TemperatureMonitorNode>();
  
  RCLCPP_INFO(rclcpp::get_logger("temperature_monitor"), "Starting temperature monitor node");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
