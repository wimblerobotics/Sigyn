#include <memory>
#include <chrono>
#include <vector>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include "sigyn_house_patroller/msg/threat_alert.hpp"
#include "sigyn_house_patroller/msg/system_health.hpp"
#include "sigyn_house_patroller/srv/set_patrol_mode.hpp"
#include "sigyn_house_patroller/action/patrol_to_waypoint.hpp"

namespace sigyn_house_patroller {

class ThreatResponseNode : public rclcpp::Node {
public:
  explicit ThreatResponseNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("threat_response", options) {
    
    // Declare parameters
    declare_parameter("response_timeout", 300.0);  // 5 minutes
    declare_parameter("investigation_distance", 1.0);  // meters
    declare_parameter("threat_priority_threshold", 2);  // WARNING level
    declare_parameter("enable_auto_response", true);
    declare_parameter("response_cooldown", 120.0);  // 2 minutes
    declare_parameter("max_investigation_attempts", 3);
    declare_parameter("emergency_stop_topic", "/emergency_stop");
    declare_parameter("email_alerts", true);
    declare_parameter("patrol_mode_service", "/sigyn_house_patroller/set_patrol_mode");
    declare_parameter("patrol_action", "/sigyn_house_patroller/patrol_to_waypoint");
    
    // Response waypoints for different threat types
    declare_parameter("response_waypoints.battery_critical", "entry_center");
    declare_parameter("response_waypoints.temperature_anomaly", "kitchen_center");
    declare_parameter("response_waypoints.door_state_change", "front_door_check");
    declare_parameter("response_waypoints.motion_detection", "living_room_center");
    declare_parameter("response_waypoints.default", "hallway_center");
    
    // Get parameters
    response_timeout_ = get_parameter("response_timeout").as_double();
    investigation_distance_ = get_parameter("investigation_distance").as_double();
    threat_priority_threshold_ = get_parameter("threat_priority_threshold").as_int();
    enable_auto_response_ = get_parameter("enable_auto_response").as_bool();
    response_cooldown_ = get_parameter("response_cooldown").as_double();
    max_investigation_attempts_ = get_parameter("max_investigation_attempts").as_int();
    emergency_stop_topic_ = get_parameter("emergency_stop_topic").as_string();
    email_alerts_ = get_parameter("email_alerts").as_bool();
    patrol_mode_service_ = get_parameter("patrol_mode_service").as_string();
    patrol_action_ = get_parameter("patrol_action").as_string();
    
    // Load response waypoints
    LoadResponseWaypoints();
    
    // Initialize state
    is_responding_ = false;
    current_threat_level_ = 0;
    
    // Publishers
    health_pub_ = create_publisher<msg::SystemHealth>(
      "~/health", rclcpp::QoS(10).reliable());
    
    emergency_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
      emergency_stop_topic_, rclcpp::QoS(10).reliable());
    
    notification_pub_ = create_publisher<std_msgs::msg::String>(
      "~/notifications", rclcpp::QoS(10).reliable());
    
    // Subscribers
    threat_alert_sub_ = create_subscription<msg::ThreatAlert>(
      "/sigyn_house_patroller/threat_alerts", rclcpp::QoS(10).reliable(),
      std::bind(&ThreatResponseNode::ThreatAlertCallback, this, std::placeholders::_1));
    
    // Service clients
    patrol_mode_client_ = create_client<srv::SetPatrolMode>(patrol_mode_service_);
    
    // Action clients
    patrol_action_client_ = rclcpp_action::create_client<action::PatrolToWaypoint>(
      this, patrol_action_);
    
    // Timers
    health_timer_ = create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&ThreatResponseNode::HealthTimerCallback, this));
    
    response_timeout_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ThreatResponseNode::ResponseTimeoutCallback, this));
    
    RCLCPP_INFO(get_logger(), "Threat response node started - Auto response: %s", 
                enable_auto_response_ ? "enabled" : "disabled");
  }

private:
  enum class ResponseState {
    kIdle,
    kAssessing,
    kInvestigating,
    kResolved,
    kFailed
  };
  
  struct ThreatResponse {
    std::string threat_id;
    std::string threat_type;
    int severity;
    geometry_msgs::msg::Point location;
    std::chrono::system_clock::time_point detected_time;
    std::chrono::system_clock::time_point response_start_time;
    ResponseState state;
    int investigation_attempts;
    std::string response_waypoint;
    std::string response_actions;
    bool email_sent;
    
    ThreatResponse() : severity(0), state(ResponseState::kIdle), 
                      investigation_attempts(0), email_sent(false) {}
  };
  
  void ThreatAlertCallback(const msg::ThreatAlert::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(response_mutex_);
    
    RCLCPP_INFO(get_logger(), "Received threat alert: %s (severity: %d)", 
                msg->threat_type.c_str(), msg->severity_level);
    
    // Update current threat level
    current_threat_level_ = std::max(current_threat_level_, (int)msg->severity_level);
    
    // Check if we should respond to this threat
    if (msg->severity_level >= threat_priority_threshold_) {
      ProcessThreatAlert(*msg);
    }
  }
  
  void ProcessThreatAlert(const msg::ThreatAlert& alert) {
    // Check if we're already responding to this threat
    auto it = active_responses_.find(alert.threat_id);
    if (it != active_responses_.end()) {
      // Update existing response
      it->second.severity = std::max(it->second.severity, (int)alert.severity_level);
      return;
    }
    
    // Create new threat response
    ThreatResponse response;
    response.threat_id = alert.threat_id;
    response.threat_type = alert.threat_type;
    response.severity = alert.severity_level;
    response.location = alert.location;
    response.detected_time = std::chrono::system_clock::now();
    response.state = ResponseState::kAssessing;
    
    // Determine response waypoint
    response.response_waypoint = GetResponseWaypoint(alert.threat_type);
    
    active_responses_[alert.threat_id] = response;
    
    RCLCPP_INFO(get_logger(), "Created threat response for: %s", alert.threat_id.c_str());
    
    // Send notification
    SendNotification("Threat detected: " + alert.threat_type + " - " + alert.description);
    
    // Send email alert if enabled
    if (email_alerts_ && !response.email_sent) {
      SendEmailAlert(alert);
      response.email_sent = true;
    }
    
    // Start response if auto-response is enabled
    if (enable_auto_response_) {
      StartThreatResponse(alert.threat_id);
    }
  }
  
  void StartThreatResponse(const std::string& threat_id) {
    auto it = active_responses_.find(threat_id);
    if (it == active_responses_.end()) {
      return;
    }
    
    ThreatResponse& response = it->second;
    
    // Check response cooldown
    if (IsResponseCooldownActive(response.threat_type)) {
      RCLCPP_INFO(get_logger(), "Response cooldown active for threat type: %s", 
                  response.threat_type.c_str());
      return;
    }
    
    // Handle different threat types
    if (response.threat_type == "battery_critical") {
      HandleBatteryThreat(response);
    } else if (response.threat_type == "temperature_anomaly") {
      HandleTemperatureThreat(response);
    } else if (response.threat_type == "door_state_change") {
      HandleDoorThreat(response);
    } else if (response.threat_type == "motion_detection" || 
               response.threat_type == "visual_environment_change" ||
               response.threat_type == "3d_environment_change") {
      HandleMotionThreat(response);
    } else {
      HandleGenericThreat(response);
    }
    
    response.response_start_time = std::chrono::system_clock::now();
    response.state = ResponseState::kInvestigating;
    is_responding_ = true;
    
    // Update response cooldown
    UpdateResponseCooldown(response.threat_type);
    
    RCLCPP_INFO(get_logger(), "Started response for threat: %s", threat_id.c_str());
  }
  
  void HandleBatteryThreat(ThreatResponse& response) {
    // For battery threats, navigate to charging station
    response.response_actions = "Navigate to charging station";
    
    // Set patrol mode to emergency response
    SetPatrolMode("emergency");
    
    // Navigate to entry point (presumably near charging station)
    NavigateToResponseWaypoint(response.response_waypoint);
    
    // Send emergency notification
    SendNotification("CRITICAL: Battery level critical - returning to base");
  }
  
  void HandleTemperatureThreat(ThreatResponse& response) {
    // For temperature threats, investigate the area
    response.response_actions = "Investigate temperature anomaly";
    
    // Navigate to investigate
    NavigateToResponseWaypoint(response.response_waypoint);
    
    // Send notification
    SendNotification("Investigating temperature anomaly in area");
  }
  
  void HandleDoorThreat(ThreatResponse& response) {
    // For door threats, investigate the door
    response.response_actions = "Investigate door state change";
    
    // Navigate to door area
    NavigateToResponseWaypoint(response.response_waypoint);
    
    // Send notification
    SendNotification("Investigating door state change - security check");
  }
  
  void HandleMotionThreat(ThreatResponse& response) {
    // For motion/change threats, investigate the area
    response.response_actions = "Investigate motion/change detection";
    
    // If severity is high, trigger emergency stop
    if (response.severity >= msg::ThreatAlert::SEVERITY_CRITICAL) {
      TriggerEmergencyStop();
      response.response_actions += " - EMERGENCY STOP TRIGGERED";
    } else {
      // Navigate to investigate
      NavigateToResponseWaypoint(response.response_waypoint);
    }
    
    // Send notification
    SendNotification("Motion/change detected - investigating area");
  }
  
  void HandleGenericThreat(ThreatResponse& response) {
    // Generic threat handling
    response.response_actions = "Generic threat response";
    
    // Navigate to default response waypoint
    NavigateToResponseWaypoint(response.response_waypoint);
    
    // Send notification
    SendNotification("Threat detected - initiating response");
  }
  
  void NavigateToResponseWaypoint(const std::string& waypoint_id) {
    if (!patrol_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Patrol action server not available");
      return;
    }
    
    auto goal = action::PatrolToWaypoint::Goal();
    goal.waypoint_name = waypoint_id;
    
    auto send_goal_options = rclcpp_action::Client<action::PatrolToWaypoint>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ThreatResponseNode::PatrolGoalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&ThreatResponseNode::PatrolFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&ThreatResponseNode::PatrolResultCallback, this, std::placeholders::_1);
    
    patrol_action_client_->async_send_goal(goal, send_goal_options);
    
    RCLCPP_INFO(get_logger(), "Navigating to response waypoint: %s", waypoint_id.c_str());
  }
  
  void SetPatrolMode(const std::string& mode) {
    if (!patrol_mode_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Patrol mode service not available");
      return;
    }
    
    auto request = std::make_shared<srv::SetPatrolMode::Request>();
    request->requested_mode = mode;
    
    auto result = patrol_mode_client_->async_send_request(request);
    
    RCLCPP_INFO(get_logger(), "Set patrol mode to: %s", mode.c_str());
  }
  
  void TriggerEmergencyStop() {
    auto emergency_msg = std_msgs::msg::Bool();
    emergency_msg.data = true;
    emergency_stop_pub_->publish(emergency_msg);
    
    RCLCPP_ERROR(get_logger(), "EMERGENCY STOP TRIGGERED");
    
    // Send emergency notification
    SendNotification("EMERGENCY STOP - Critical threat detected");
  }
  
  void SendNotification(const std::string& message) {
    std_msgs::msg::String notification;
    notification.data = message;
    notification_pub_->publish(notification);
    
    RCLCPP_INFO(get_logger(), "Notification: %s", message.c_str());
  }
  
  void SendEmailAlert(const msg::ThreatAlert& alert) {
    // In a real implementation, this would send an email
    // For now, just log the email content
    
    std::string email_subject = "Sigyn Security Alert: " + alert.threat_type;
    std::string email_body = std::string("Threat detected:\n") +
                           "Type: " + alert.threat_type + "\n" +
                           "Severity: " + std::to_string(alert.severity_level) + "\n" +
                           "Description: " + alert.description + "\n" +
                           "Time: " + std::to_string(alert.header.stamp.sec) + "\n" +
                           "Location: (" + std::to_string(alert.location.x) + ", " + 
                                          std::to_string(alert.location.y) + ")\n" +
                           "Confidence: " + std::to_string(alert.confidence);
    
    RCLCPP_INFO(get_logger(), "EMAIL ALERT:\nSubject: %s\nBody: %s", 
                email_subject.c_str(), email_body.c_str());
    
    // Send notification about email
    SendNotification("Email alert sent for: " + alert.threat_type);
  }
  
  std::string GetResponseWaypoint(const std::string& threat_type) {
    auto it = response_waypoints_.find(threat_type);
    if (it != response_waypoints_.end()) {
      return it->second;
    }
    
    // Return default waypoint
    auto default_it = response_waypoints_.find("default");
    if (default_it != response_waypoints_.end()) {
      return default_it->second;
    }
    
    return "hallway_center";  // Fallback
  }
  
  void LoadResponseWaypoints() {
    std::vector<std::string> threat_types = {
      "battery_critical", "temperature_anomaly", "door_state_change", 
      "motion_detection", "default"
    };
    
    for (const auto& type : threat_types) {
      std::string param_name = "response_waypoints." + type;
      if (has_parameter(param_name)) {
        response_waypoints_[type] = get_parameter(param_name).as_string();
        RCLCPP_INFO(get_logger(), "Loaded response waypoint for %s: %s", 
                    type.c_str(), response_waypoints_[type].c_str());
      }
    }
  }
  
  bool IsResponseCooldownActive(const std::string& threat_type) {
    auto it = last_response_times_.find(threat_type);
    if (it == last_response_times_.end()) {
      return false;
    }
    
    auto now = std::chrono::system_clock::now();
    auto time_since_response = std::chrono::duration_cast<std::chrono::seconds>(
      now - it->second).count();
    
    return time_since_response < response_cooldown_;
  }
  
  void UpdateResponseCooldown(const std::string& threat_type) {
    last_response_times_[threat_type] = std::chrono::system_clock::now();
  }
  
  void PatrolGoalResponseCallback(const rclcpp_action::ClientGoalHandle<action::PatrolToWaypoint>::SharedPtr& goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
    }
  }
  
  void PatrolFeedbackCallback(
    rclcpp_action::ClientGoalHandle<action::PatrolToWaypoint>::SharedPtr,
    const std::shared_ptr<const action::PatrolToWaypoint::Feedback> feedback) {
    
    RCLCPP_INFO(get_logger(), "Navigation progress: %.1f%%", feedback->progress * 100);
  }
  
  void PatrolResultCallback(const rclcpp_action::ClientGoalHandle<action::PatrolToWaypoint>::WrappedResult& result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Response navigation succeeded");
        CompleteResponse(true);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Response navigation aborted");
        CompleteResponse(false);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(get_logger(), "Response navigation canceled");
        CompleteResponse(false);
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown response navigation result code");
        CompleteResponse(false);
        break;
    }
  }
  
  void CompleteResponse(bool success) {
    std::lock_guard<std::mutex> lock(response_mutex_);
    
    // Mark all active responses as completed
    for (auto& [threat_id, response] : active_responses_) {
      response.state = success ? ResponseState::kResolved : ResponseState::kFailed;
    }
    
    is_responding_ = false;
    
    // Send completion notification
    std::string message = success ? "Threat response completed successfully" : 
                                   "Threat response failed";
    SendNotification(message);
    
    RCLCPP_INFO(get_logger(), "Response completed: %s", success ? "success" : "failure");
  }
  
  void ResponseTimeoutCallback() {
    std::lock_guard<std::mutex> lock(response_mutex_);
    
    if (!is_responding_) {
      return;
    }
    
    auto now = std::chrono::system_clock::now();
    
    // Check for response timeouts
    for (auto& [threat_id, response] : active_responses_) {
      if (response.state == ResponseState::kInvestigating) {
        auto response_duration = std::chrono::duration_cast<std::chrono::seconds>(
          now - response.response_start_time).count();
        
        if (response_duration > response_timeout_) {
          RCLCPP_WARN(get_logger(), "Response timeout for threat: %s", threat_id.c_str());
          
          response.state = ResponseState::kFailed;
          SendNotification("Response timeout for threat: " + threat_id);
          
          // Try again if we haven't exceeded max attempts
          if (response.investigation_attempts < max_investigation_attempts_) {
            response.investigation_attempts++;
            response.response_start_time = now;
            response.state = ResponseState::kInvestigating;
            
            RCLCPP_INFO(get_logger(), "Retrying response for threat: %s (attempt %d/%d)", 
                        threat_id.c_str(), response.investigation_attempts, max_investigation_attempts_);
            
            // Retry navigation
            NavigateToResponseWaypoint(response.response_waypoint);
          } else {
            RCLCPP_ERROR(get_logger(), "Max investigation attempts reached for threat: %s", 
                         threat_id.c_str());
            is_responding_ = false;
          }
        }
      }
    }
    
    // Clean up old responses
    CleanupOldResponses();
  }
  
  void CleanupOldResponses() {
    auto now = std::chrono::system_clock::now();
    const auto cleanup_age = std::chrono::hours(1);
    
    for (auto it = active_responses_.begin(); it != active_responses_.end();) {
      auto age = std::chrono::duration_cast<std::chrono::hours>(
        now - it->second.detected_time);
      
      if (age > cleanup_age && 
          (it->second.state == ResponseState::kResolved || 
           it->second.state == ResponseState::kFailed)) {
        it = active_responses_.erase(it);
      } else {
        ++it;
      }
    }
  }
  
  void HealthTimerCallback() {
    msg::SystemHealth health;
    health.header.stamp = this->now();
    health.header.frame_id = "base_link";
    health.overall_health = is_responding_ ? msg::SystemHealth::HEALTH_DEGRADED : msg::SystemHealth::HEALTH_HEALTHY;
    
    health.component_names.push_back("threat_response");
    health.component_health_status.push_back(is_responding_ ? msg::SystemHealth::HEALTH_DEGRADED : msg::SystemHealth::HEALTH_HEALTHY);
    health.component_descriptions.push_back(is_responding_ ? "RESPONDING" : "IDLE");
    health.last_update_times.push_back(this->now());

    health.component_names.push_back("threat_level");
    health.component_health_status.push_back(msg::SystemHealth::HEALTH_HEALTHY);
    health.component_descriptions.push_back(std::to_string(current_threat_level_));
    health.last_update_times.push_back(this->now());

    health.component_names.push_back("active_responses");
    health.component_health_status.push_back(msg::SystemHealth::HEALTH_HEALTHY);
    health.component_descriptions.push_back(std::to_string(active_responses_.size()));
    health.last_update_times.push_back(this->now());

    health.component_names.push_back("auto_response");
    health.component_health_status.push_back(msg::SystemHealth::HEALTH_HEALTHY);
    health.component_descriptions.push_back(enable_auto_response_ ? "enabled" : "disabled");
    health.last_update_times.push_back(this->now());
    
    health_pub_->publish(health);
  }
  
  // Parameters
  double response_timeout_;
  double investigation_distance_;
  int threat_priority_threshold_;
  bool enable_auto_response_;
  double response_cooldown_;
  int max_investigation_attempts_;
  std::string emergency_stop_topic_;
  bool email_alerts_;
  std::string patrol_mode_service_;
  std::string patrol_action_;
  
  // State
  bool is_responding_;
  int current_threat_level_;
  
  // Data storage
  std::unordered_map<std::string, ThreatResponse> active_responses_;
  std::unordered_map<std::string, std::string> response_waypoints_;
  std::unordered_map<std::string, std::chrono::system_clock::time_point> last_response_times_;
  
  // ROS2 interfaces
  rclcpp::Publisher<msg::SystemHealth>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr notification_pub_;
  rclcpp::Subscription<msg::ThreatAlert>::SharedPtr threat_alert_sub_;
  rclcpp::Client<srv::SetPatrolMode>::SharedPtr patrol_mode_client_;
  rclcpp_action::Client<action::PatrolToWaypoint>::SharedPtr patrol_action_client_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::TimerBase::SharedPtr response_timeout_timer_;
  
  std::mutex response_mutex_;
};

}  // namespace sigyn_house_patroller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<sigyn_house_patroller::ThreatResponseNode>();
  
  RCLCPP_INFO(rclcpp::get_logger("threat_response"), "Starting threat response node");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
