#include "sigyn_house_patroller/core/patrol_manager.hpp"

#include <fstream>
#include <algorithm>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>

namespace sigyn_house_patroller {

PatrolManager::PatrolManager(const std::string& node_name)
    : rclcpp::Node(node_name),
      current_mode_(PatrolMode::kIdle),
      patrol_sequence_index_(0),
      is_running_(false),
      patrol_complete_(false),
      emergency_stop_(false) {
  
  // Initialize ROS2 interfaces
  InitializeROS2Interfaces();
  
  // Create core components
  waypoint_manager_ = std::make_unique<WaypointManager>(shared_from_this());
  navigation_coordinator_ = std::make_unique<NavigationCoordinator>(shared_from_this());
  threat_detector_ = std::make_unique<ThreatDetectionManager>(shared_from_this());
  
  // Initialize with default configuration
  LoadConfiguration();
  
  RCLCPP_INFO(get_logger(), "PatrolManager initialized");
}

PatrolManager::~PatrolManager() {
  Shutdown();
}

void PatrolManager::InitializeROS2Interfaces() {
  // Publishers
  threat_alert_pub_ = create_publisher<msg::ThreatAlert>(
    "~/threat_alerts", rclcpp::QoS(10).reliable());
  
  patrol_status_pub_ = create_publisher<msg::PatrolStatus>(
    "~/patrol_status", rclcpp::QoS(10).reliable());
  
  system_health_pub_ = create_publisher<msg::SystemHealth>(
    "~/system_health", rclcpp::QoS(10).reliable());
  
  // Subscribers
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "~/emergency_stop", rclcpp::QoS(10).reliable(),
    std::bind(&PatrolManager::EmergencyStopCallback, this, std::placeholders::_1));
  
  // Service servers
  set_patrol_mode_service_ = create_service<srv::SetPatrolMode>(
    "~/set_patrol_mode",
    std::bind(&PatrolManager::SetPatrolModeCallback, this, 
              std::placeholders::_1, std::placeholders::_2));
  
  get_room_info_service_ = create_service<srv::GetRoomInfo>(
    "~/get_room_info",
    std::bind(&PatrolManager::GetRoomInfoCallback, this,
              std::placeholders::_1, std::placeholders::_2));
  
  // Action server
  patrol_action_server_ = rclcpp_action::create_server<action::PatrolToWaypoint>(
    this, "~/patrol_to_waypoint",
    std::bind(&PatrolManager::PatrolGoalCallback, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&PatrolManager::PatrolCancelCallback, this, std::placeholders::_1),
    std::bind(&PatrolManager::PatrolAcceptedCallback, this, std::placeholders::_1));
  
  // Timers
  patrol_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PatrolManager::PatrolTimerCallback, this));
  
  status_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&PatrolManager::StatusTimerCallback, this));
  
  health_timer_ = create_wall_timer(
    std::chrono::seconds(10),
    std::bind(&PatrolManager::HealthTimerCallback, this));
}

bool PatrolManager::Initialize() {
  RCLCPP_INFO(get_logger(), "Initializing PatrolManager components...");
  
  // Initialize waypoint manager
  if (!waypoint_manager_->Initialize()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize waypoint manager");
    return false;
  }
  
  // Initialize navigation coordinator
  if (!navigation_coordinator_->Initialize()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize navigation coordinator");
    return false;
  }
  
  // Initialize threat detection
  if (!threat_detector_->Initialize()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize threat detection");
    return false;
  }
  
  // Load patrol configuration
  if (!LoadPatrolConfiguration()) {
    RCLCPP_ERROR(get_logger(), "Failed to load patrol configuration");
    return false;
  }
  
  RCLCPP_INFO(get_logger(), "PatrolManager initialization complete");
  return true;
}

void PatrolManager::Start() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  if (is_running_) {
    RCLCPP_WARN(get_logger(), "PatrolManager is already running");
    return;
  }
  
  is_running_ = true;
  emergency_stop_ = false;
  
  // Start patrol execution thread
  patrol_thread_ = std::thread(&PatrolManager::PatrolExecutionLoop, this);
  
  RCLCPP_INFO(get_logger(), "PatrolManager started");
}

void PatrolManager::Stop() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  if (!is_running_) {
    RCLCPP_WARN(get_logger(), "PatrolManager is not running");
    return;
  }
  
  is_running_ = false;
  
  // Cancel any ongoing navigation
  if (navigation_coordinator_->IsNavigating()) {
    navigation_coordinator_->CancelNavigation();
  }
  
  // Wait for patrol thread to finish
  if (patrol_thread_.joinable()) {
    patrol_thread_.join();
  }
  
  RCLCPP_INFO(get_logger(), "PatrolManager stopped");
}

void PatrolManager::Shutdown() {
  Stop();
  
  // Additional cleanup if needed
  RCLCPP_INFO(get_logger(), "PatrolManager shutdown complete");
}

void PatrolManager::SetPatrolMode(PatrolMode mode) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  if (current_mode_ == mode) {
    return;
  }
  
  PatrolMode previous_mode = current_mode_;
  current_mode_ = mode;
  
  RCLCPP_INFO(get_logger(), "Patrol mode changed from %s to %s", 
              PatrolModeToString(previous_mode).c_str(),
              PatrolModeToString(mode).c_str());
  
  // Handle mode transitions
  switch (mode) {
    case PatrolMode::kIdle:
      if (navigation_coordinator_->IsNavigating()) {
        navigation_coordinator_->CancelNavigation();
      }
      break;
      
    case PatrolMode::kFullPatrol:
      patrol_sequence_index_ = 0;
      patrol_complete_ = false;
      break;
      
    case PatrolMode::kRoomInspection:
      // Room inspection mode will be handled by action server
      break;
      
    case PatrolMode::kEmergencyResponse:
      // Cancel current navigation and prepare for emergency response
      if (navigation_coordinator_->IsNavigating()) {
        navigation_coordinator_->CancelNavigation();
      }
      break;
  }
}

PatrolManager::PatrolMode PatrolManager::GetPatrolMode() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_mode_;
}

std::string PatrolManager::PatrolModeToString(PatrolMode mode) const {
  switch (mode) {
    case PatrolMode::kIdle: return "Idle";
    case PatrolMode::kFullPatrol: return "Full Patrol";
    case PatrolMode::kRoomInspection: return "Room Inspection";
    case PatrolMode::kEmergencyResponse: return "Emergency Response";
    default: return "Unknown";
  }
}

void PatrolManager::PatrolExecutionLoop() {
  RCLCPP_INFO(get_logger(), "Patrol execution loop started");
  
  while (is_running_) {
    try {
      // Check for emergency stop
      if (emergency_stop_) {
        HandleEmergencyStop();
        continue;
      }
      
      // Run threat detection
      RunThreatDetection();
      
      // Execute patrol based on current mode
      switch (current_mode_) {
        case PatrolMode::kIdle:
          // Do nothing in idle mode
          break;
          
        case PatrolMode::kFullPatrol:
          ExecuteFullPatrol();
          break;
          
        case PatrolMode::kRoomInspection:
          // Room inspection is handled by action server
          break;
          
        case PatrolMode::kEmergencyResponse:
          ExecuteEmergencyResponse();
          break;
      }
      
      // Small delay to prevent excessive CPU usage
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Exception in patrol execution loop: %s", e.what());
    }
  }
  
  RCLCPP_INFO(get_logger(), "Patrol execution loop ended");
}

void PatrolManager::ExecuteFullPatrol() {
  if (patrol_complete_) {
    return;
  }
  
  // Get current patrol sequence
  if (current_patrol_sequence_.empty()) {
    GenerateFullPatrolSequence();
  }
  
  if (current_patrol_sequence_.empty()) {
    RCLCPP_WARN(get_logger(), "No waypoints available for full patrol");
    patrol_complete_ = true;
    return;
  }
  
  // Check if we're currently navigating
  if (navigation_coordinator_->IsNavigating()) {
    return;  // Wait for current navigation to complete
  }
  
  // Move to next waypoint
  if (patrol_sequence_index_ < current_patrol_sequence_.size()) {
    const auto& waypoint = current_patrol_sequence_[patrol_sequence_index_];
    
    RCLCPP_INFO(get_logger(), "Navigating to waypoint: %s in room: %s", 
                waypoint.id.c_str(), waypoint.room_name.c_str());
    
    // Navigate to waypoint
    navigation_coordinator_->NavigateToWaypoint(waypoint,
      [this](const NavigationResult& result) {
        this->OnWaypointNavigationComplete(result);
      });
    
  } else {
    // Patrol sequence complete
    patrol_complete_ = true;
    patrol_sequence_index_ = 0;
    current_patrol_sequence_.clear();
    
    RCLCPP_INFO(get_logger(), "Full patrol sequence completed");
  }
}

void PatrolManager::ExecuteEmergencyResponse() {
  // Emergency response logic will be implemented based on threat type
  // For now, just ensure robot stops
  if (navigation_coordinator_->IsNavigating()) {
    navigation_coordinator_->CancelNavigation();
  }
}

void PatrolManager::RunThreatDetection() {
  if (!threat_detector_) {
    return;
  }
  
  // Run threat detection
  auto threats = threat_detector_->RunDetection();
  
  // Process detected threats
  for (const auto& threat : threats) {
    ProcessThreatAlert(threat);
  }
}

void PatrolManager::ProcessThreatAlert(const msg::ThreatAlert& threat) {
  // Publish threat alert
  threat_alert_pub_->publish(threat);
  
  // Log threat
  RCLCPP_WARN(get_logger(), "Threat detected: %s (severity: %d) - %s",
              threat.threat_type.c_str(), threat.severity, threat.description.c_str());
  
  // Handle threat based on severity
  switch (threat.severity) {
    case msg::ThreatAlert::SEVERITY_INFO:
      // Just log and continue
      break;
      
    case msg::ThreatAlert::SEVERITY_WARNING:
      // Continue patrol but send notification
      SendThreatNotification(threat);
      break;
      
    case msg::ThreatAlert::SEVERITY_CRITICAL:
      // Stop current patrol and investigate
      SetPatrolMode(PatrolMode::kEmergencyResponse);
      SendThreatNotification(threat);
      break;
      
    case msg::ThreatAlert::SEVERITY_EMERGENCY:
      // Emergency stop and immediate notification
      emergency_stop_ = true;
      SendThreatNotification(threat);
      break;
  }
}

void PatrolManager::SendThreatNotification(const msg::ThreatAlert& threat) {
  // TODO: Implement email notification system
  // For now, just log
  RCLCPP_ERROR(get_logger(), "THREAT NOTIFICATION: %s - %s", 
               threat.threat_type.c_str(), threat.description.c_str());
}

void PatrolManager::GenerateFullPatrolSequence() {
  auto current_pose = navigation_coordinator_->GetCurrentPose();
  current_patrol_sequence_ = waypoint_manager_->GetOptimalPatrolSequence(current_pose);
  patrol_sequence_index_ = 0;
  
  RCLCPP_INFO(get_logger(), "Generated patrol sequence with %zu waypoints", 
              current_patrol_sequence_.size());
}

void PatrolManager::OnWaypointNavigationComplete(const NavigationResult& result) {
  if (result.status == NavigationStatus::kSuccess) {
    // Update room visit time
    if (patrol_sequence_index_ < current_patrol_sequence_.size()) {
      const auto& waypoint = current_patrol_sequence_[patrol_sequence_index_];
      waypoint_manager_->UpdateRoomVisitTime(waypoint.room_name, 
                                            std::chrono::system_clock::now());
    }
    
    // Move to next waypoint
    patrol_sequence_index_++;
    
    RCLCPP_INFO(get_logger(), "Waypoint navigation completed successfully");
    
  } else {
    RCLCPP_ERROR(get_logger(), "Waypoint navigation failed: %s", result.message.c_str());
    
    // Handle navigation failure
    // For now, just move to next waypoint
    patrol_sequence_index_++;
  }
}

void PatrolManager::HandleEmergencyStop() {
  // Cancel any ongoing navigation
  if (navigation_coordinator_->IsNavigating()) {
    navigation_coordinator_->CancelNavigation();
  }
  
  // Set to emergency response mode
  SetPatrolMode(PatrolMode::kEmergencyResponse);
  
  RCLCPP_ERROR(get_logger(), "Emergency stop activated");
}

bool PatrolManager::LoadConfiguration() {
  // Load configuration from parameters
  // This is a simplified version - in practice, you'd load from config files
  
  declare_parameter("patrol_config_file", "");
  declare_parameter("waypoint_config_file", "");
  declare_parameter("room_config_file", "");
  
  return true;
}

bool PatrolManager::LoadPatrolConfiguration() {
  // Load patrol-specific configuration
  return true;
}

// ROS2 callback implementations
void PatrolManager::EmergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  emergency_stop_ = msg->data;
  
  if (emergency_stop_) {
    RCLCPP_ERROR(get_logger(), "Emergency stop received");
  } else {
    RCLCPP_INFO(get_logger(), "Emergency stop cleared");
  }
}

void PatrolManager::SetPatrolModeCallback(
    const std::shared_ptr<srv::SetPatrolMode::Request> request,
    std::shared_ptr<srv::SetPatrolMode::Response> response) {
  
  PatrolMode new_mode = static_cast<PatrolMode>(request->mode);
  
  try {
    SetPatrolMode(new_mode);
    response->success = true;
    response->message = "Patrol mode set successfully";
  } catch (const std::exception& e) {
    response->success = false;
    response->message = std::string("Failed to set patrol mode: ") + e.what();
  }
}

void PatrolManager::GetRoomInfoCallback(
    const std::shared_ptr<srv::GetRoomInfo::Request> request,
    std::shared_ptr<srv::GetRoomInfo::Response> response) {
  
  const auto* room_info = waypoint_manager_->GetRoomInfo(request->room_name);
  
  if (room_info) {
    response->room_name = room_info->name;
    response->room_type = room_info->type;
    response->center = room_info->center;
    response->area = room_info->area;
    response->is_accessible = room_info->is_accessible;
    response->success = true;
  } else {
    response->success = false;
    response->room_name = "";
  }
}

rclcpp_action::GoalResponse PatrolManager::PatrolGoalCallback(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const action::PatrolToWaypoint::Goal> goal) {
  
  (void)uuid;  // Unused parameter
  
  RCLCPP_INFO(get_logger(), "Received patrol goal for waypoint: %s", goal->waypoint_id.c_str());
  
  // Validate waypoint
  const auto* waypoint = waypoint_manager_->FindWaypoint(goal->waypoint_id);
  if (!waypoint) {
    RCLCPP_ERROR(get_logger(), "Waypoint not found: %s", goal->waypoint_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PatrolManager::PatrolCancelCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::PatrolToWaypoint>> goal_handle) {
  
  (void)goal_handle;  // Unused parameter
  
  RCLCPP_INFO(get_logger(), "Received patrol cancel request");
  
  // Cancel navigation
  navigation_coordinator_->CancelNavigation();
  
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PatrolManager::PatrolAcceptedCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::PatrolToWaypoint>> goal_handle) {
  
  // Execute patrol action in separate thread
  std::thread([this, goal_handle]() {
    this->ExecutePatrolAction(goal_handle);
  }).detach();
}

void PatrolManager::ExecutePatrolAction(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::PatrolToWaypoint>> goal_handle) {
  
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<action::PatrolToWaypoint::Feedback>();
  auto result = std::make_shared<action::PatrolToWaypoint::Result>();
  
  // Find waypoint
  const auto* waypoint = waypoint_manager_->FindWaypoint(goal->waypoint_id);
  if (!waypoint) {
    result->success = false;
    result->message = "Waypoint not found";
    goal_handle->abort(result);
    return;
  }
  
  // Navigate to waypoint
  bool navigation_complete = false;
  NavigationResult nav_result;
  
  navigation_coordinator_->NavigateToWaypoint(*waypoint,
    [&navigation_complete, &nav_result](const NavigationResult& result) {
      nav_result = result;
      navigation_complete = true;
    },
    [goal_handle, feedback](const geometry_msgs::msg::PoseStamped& pose, double progress) {
      feedback->current_pose = pose;
      feedback->progress = progress;
      goal_handle->publish_feedback(feedback);
    });
  
  // Wait for navigation to complete
  while (!navigation_complete && rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      navigation_coordinator_->CancelNavigation();
      result->success = false;
      result->message = "Navigation cancelled";
      goal_handle->canceled(result);
      return;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  // Set result
  result->success = (nav_result.status == NavigationStatus::kSuccess);
  result->message = nav_result.message;
  result->final_pose = navigation_coordinator_->GetCurrentPose();
  
  if (result->success) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
}

void PatrolManager::PatrolTimerCallback() {
  // This timer runs frequently to check patrol state
  // Most of the work is done in the patrol execution thread
}

void PatrolManager::StatusTimerCallback() {
  // Publish patrol status
  msg::PatrolStatus status;
  status.header.stamp = now();
  status.mode = static_cast<int>(current_mode_);
  status.is_navigating = navigation_coordinator_->IsNavigating();
  status.current_waypoint = "";  // Set based on current state
  
  if (patrol_sequence_index_ < current_patrol_sequence_.size()) {
    status.current_waypoint = current_patrol_sequence_[patrol_sequence_index_].id;
  }
  
  status.patrol_progress = patrol_complete_ ? 1.0 : 
    (static_cast<double>(patrol_sequence_index_) / current_patrol_sequence_.size());
  
  patrol_status_pub_->publish(status);
}

void PatrolManager::HealthTimerCallback() {
  // Publish system health
  msg::SystemHealth health;
  health.header.stamp = now();
  health.overall_health = 1.0;  // Calculate based on component health
  health.navigation_health = navigation_coordinator_->IsNavigationHealthy() ? 1.0 : 0.0;
  health.detection_health = threat_detector_->AreDetectorsHealthy() ? 1.0 : 0.0;
  
  // Add component statuses
  auto detector_health = threat_detector_->GetDetectorHealthStatus();
  for (const auto& [name, healthy] : detector_health) {
    health.component_status.push_back(name + ": " + (healthy ? "OK" : "ERROR"));
  }
  
  system_health_pub_->publish(health);
}

}  // namespace sigyn_house_patroller
