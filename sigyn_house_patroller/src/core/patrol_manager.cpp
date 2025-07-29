#include "sigyn_house_patroller/core/patrol_manager.hpp"
#include "sigyn_house_patroller/core/threat_detection_manager.hpp"

namespace sigyn_house_patroller {

PatrolManager::PatrolManager(
    const std::string& node_name,
    rclcpp::NodeOptions options,
    std::shared_ptr<WaypointManager> waypoint_manager,
    std::shared_ptr<NavigationCoordinator> navigation_coordinator,
    std::shared_ptr<ThreatDetectionManager> threat_detector)
    : rclcpp::Node(node_name, options),
      waypoint_manager_(waypoint_manager),
      navigation_coordinator_(navigation_coordinator),
      threat_detector_(threat_detector),
      current_mode_(PatrolMode::IDLE),
      patrol_index_(-1),
      is_patrolling_(false) {
  RCLCPP_INFO(this->get_logger(), "PatrolManager initialized");

  // Service for setting patrol mode
  set_patrol_mode_service_ = create_service<SetPatrolModeSrv>(
      "set_patrol_mode",
      std::bind(&PatrolManager::SetPatrolModeCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Action server for patrol to waypoint
  patrol_action_server_ = rclcpp_action::create_server<PatrolToWaypointAction>(
      this,
      "patrol_to_waypoint",
      std::bind(&PatrolManager::PatrolGoalCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PatrolManager::PatrolCancelCallback, this, std::placeholders::_1),
      std::bind(&PatrolManager::PatrolAcceptedCallback, this, std::placeholders::_1));

  // Publisher for patrol status
  patrol_status_publisher_ = create_publisher<sigyn_house_patroller::msg::PatrolStatus>("patrol_status", 10);
  system_health_publisher_ = create_publisher<sigyn_house_patroller::msg::SystemHealth>("system_health", 10);

  // Timers
  status_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PatrolManager::StatusTimerCallback, this));
  health_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&PatrolManager::HealthTimerCallback, this));

  RCLCPP_INFO(get_logger(), "PatrolManager initialized");
}

void PatrolManager::StartFullPatrol() {
  if (current_mode_ == PatrolMode::FULL_PATROL) {
    RCLCPP_WARN(get_logger(), "Full patrol already in progress.");
    return;
  }
  RCLCPP_INFO(get_logger(), "Starting full patrol.");
  current_mode_ = PatrolMode::FULL_PATROL;
  patrol_waypoints_ = waypoint_manager_->GetAllWaypoints(); // Changed from GetPatrolRoute
  if (patrol_waypoints_.empty()) {
    RCLCPP_ERROR(get_logger(), "No waypoints for full patrol route.");
    current_mode_ = PatrolMode::IDLE;
    return;
  }
  patrol_index_ = 0;
  is_patrolling_ = true;
  ExecuteFullPatrol();
}

void PatrolManager::StartRoomInspection(const std::string& room_name) {
    if (current_mode_ == PatrolMode::ROOM_INSPECTION) {
        RCLCPP_WARN(get_logger(), "Room inspection already in progress.");
        return;
    }
    RCLCPP_INFO(get_logger(), "Starting inspection for room: %s", room_name.c_str());
    current_mode_ = PatrolMode::ROOM_INSPECTION;
    patrol_waypoints_ = waypoint_manager_->GetWaypointsForRoom(room_name); // Changed from GetWaypointsByRoom
    if (patrol_waypoints_.empty()) {
        RCLCPP_ERROR(get_logger(), "No waypoints for room: %s", room_name.c_str());
        current_mode_ = PatrolMode::IDLE;
        return;
    }
    patrol_index_ = 0;
    is_patrolling_ = true;
    ExecuteRoomInspection();
}

void PatrolManager::StopPatrol() {
  if (current_mode_ == PatrolMode::IDLE) {
    RCLCPP_WARN(get_logger(), "Patrol is already idle.");
    return;
  }
  RCLCPP_INFO(get_logger(), "Stopping patrol.");
  current_mode_ = PatrolMode::IDLE;
  is_patrolling_ = false;
  patrol_index_ = -1;
  navigation_coordinator_->CancelNavigation();
}

void PatrolManager::ExecuteFullPatrol() {
  if (!is_patrolling_ || patrol_index_ >= static_cast<int>(patrol_waypoints_.size())) {
    RCLCPP_INFO(get_logger(), "Full patrol complete.");
    StopPatrol();
    return;
  }

  const auto& waypoint = patrol_waypoints_[patrol_index_];
  RCLCPP_INFO(get_logger(), "Navigating to waypoint %s", waypoint.id.c_str()); // Changed from .name to .id

  navigation_coordinator_->NavigateToWaypoint(
      waypoint,
      [this](const NavigationResult& result) {
        OnWaypointNavigationComplete(result);
      });
}

void PatrolManager::ExecuteRoomInspection() {
    if (!is_patrolling_ || patrol_index_ >= static_cast<int>(patrol_waypoints_.size())) {
        RCLCPP_INFO(get_logger(), "Room inspection complete.");
        StopPatrol();
        return;
    }

    const auto& waypoint = patrol_waypoints_[patrol_index_];
    RCLCPP_INFO(get_logger(), "Navigating to room waypoint %s", waypoint.id.c_str()); // Changed from .name to .id

    navigation_coordinator_->NavigateToWaypoint(
        waypoint,
        [this](const NavigationResult& result) {
            OnWaypointNavigationComplete(result);
        });
}

void PatrolManager::OnWaypointNavigationComplete(const NavigationResult& result) {
  if (result.status == NavigationStatus::kSuccess) {
    RCLCPP_INFO(get_logger(), "Waypoint navigation successful.");
    patrol_index_++;
    if (is_patrolling_) {
      if (current_mode_ == PatrolMode::FULL_PATROL) {
        ExecuteFullPatrol();
      } else if (current_mode_ == PatrolMode::ROOM_INSPECTION) {
        ExecuteRoomInspection();
      }
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Waypoint navigation failed: %s", result.message.c_str());
    StopPatrol();
  }
}

void PatrolManager::SetPatrolModeCallback(
    const std::shared_ptr<SetPatrolModeSrv::Request> request,
    std::shared_ptr<SetPatrolModeSrv::Response> response) {
  RCLCPP_INFO(get_logger(), "SetPatrolMode request received: %s", request->requested_mode.c_str());

  if (request->requested_mode == "idle") {
    StopPatrol();
    response->success = true;
  } else if (request->requested_mode == "full_patrol") {
    StartFullPatrol();
    response->success = true;
  } else if (request->requested_mode == "room_inspection") {
    // For room inspection, we expect a room_name to be provided.
    // This part of the logic might need to be expanded based on how room_name is passed.
    // For now, we assume it's part of another topic or a more complex service call.
    // Here, we'll just log it.
    RCLCPP_INFO(get_logger(), "Room inspection mode requested. Please specify a room.");
    // StartRoomInspection("some_room"); // Example
    response->success = true; 
  } else if (request->requested_mode == "emergency_response") {
    RCLCPP_INFO(get_logger(), "Emergency response mode activated.");
    // Logic for emergency response
    response->success = true;
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid patrol mode: %s", request->requested_mode.c_str());
    response->success = false;
    // response->message = "Invalid patrol mode"; // message field does not exist
  }
}

rclcpp_action::GoalResponse PatrolManager::PatrolGoalCallback(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const PatrolToWaypointAction::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(get_logger(), "Received patrol goal for waypoint: %s", goal->waypoint_name.c_str());
  
  const auto* waypoint = waypoint_manager_->FindWaypoint(goal->waypoint_name);
  if (!waypoint) {
    RCLCPP_ERROR(get_logger(), "Waypoint not found: %s", goal->waypoint_name.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PatrolManager::PatrolCancelCallback(
    const std::shared_ptr<GoalHandlePatrol> goal_handle) {
  RCLCPP_INFO(get_logger(), "Received request to cancel patrol goal");
  (void)goal_handle;
  navigation_coordinator_->CancelNavigation();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PatrolManager::PatrolAcceptedCallback(
    const std::shared_ptr<GoalHandlePatrol> goal_handle) {
  // this needs to return quickly to avoid blocking the action server
  // so spin up a new thread to do the work
  std::thread{std::bind(&PatrolManager::ExecutePatrolAction, this, std::placeholders::_1),
              goal_handle}
      .detach();
}

void PatrolManager::ExecutePatrolAction(const std::shared_ptr<GoalHandlePatrol> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<PatrolToWaypointAction::Feedback>();
  auto result = std::make_shared<PatrolToWaypointAction::Result>();

  const auto* waypoint = waypoint_manager_->FindWaypoint(goal->waypoint_name);
  if (!waypoint) {
    result->success = false;
    result->failure_reason = "Waypoint not found";
    goal_handle->abort(result);
    return;
  }

  std::mutex m;
  std::condition_variable cv;
  bool navigation_complete = false;
  NavigationResult nav_result;

  navigation_coordinator_->NavigateToWaypoint(
      *waypoint,
      [&](const NavigationResult& res) {
        std::lock_guard<std::mutex> lock(m);
        nav_result = res;
        navigation_complete = true;
        cv.notify_one();
      },
      [&](const geometry_msgs::msg::PoseStamped& pose, double progress) {
        feedback->current_pose = pose;
        feedback->progress = progress;
        goal_handle->publish_feedback(feedback);
      });

  std::unique_lock<std::mutex> lock(m);
  cv.wait(lock, [&] { return navigation_complete; });

  if (goal_handle->is_canceling()) {
    result->success = false;
    result->failure_reason = "Navigation cancelled";
    goal_handle->canceled(result);
    return;
  }

  if (nav_result.status == NavigationStatus::kSuccess) {
    result->success = true;
    goal_handle->succeed(result);
  } else {
    result->success = false;
    result->failure_reason = nav_result.message;
    goal_handle->abort(result);
  }
  // result->final_room = "unknown"; // Placeholder - final_room does not exist
}

void PatrolManager::StatusTimerCallback() {
  sigyn_house_patroller::msg::PatrolStatus status;
  status.header.stamp = now();
  status.patrol_mode = PatrolModeToString(current_mode_);
  
  if (is_patrolling_ && patrol_index_ >= 0 && patrol_index_ < static_cast<int>(patrol_waypoints_.size())) {
    // status.current_waypoint = patrol_waypoints_[patrol_index_].id; // current_waypoint does not exist
  } else {
    // status.current_waypoint = "none"; // current_waypoint does not exist
  }

  // status.robot_pose = navigation_coordinator_->GetCurrentPose(); // robot_pose does not exist
  
  patrol_status_publisher_->publish(status);
}

void PatrolManager::HealthTimerCallback() {
    sigyn_house_patroller::msg::SystemHealth health;
    health.header.stamp = now();
    // health.system_name = "Patrol System"; // system_name does not exist

    // // Example: Check navigation system health
    // health.component_names.push_back("Navigation");
    // bool nav_healthy = navigation_coordinator_->IsNavigating(); // Example check
    // health.component_health_status.push_back(nav_healthy ? 1 : 0); // Using 1 for OK, 0 for WARN for simplicity

    // // Example: Check threat detector health
    // health.component_names.push_back("Threat Detector");
    // auto detector_health = threat_detector_->GetDetectorHealthStatus();
    // health.component_health_status.push_back(detector_health);

    system_health_publisher_->publish(health);
}

std::string PatrolManager::PatrolModeToString(PatrolMode mode) {
  switch (mode) {
    case PatrolMode::IDLE: return "Idle";
    case PatrolMode::FULL_PATROL: return "Full Patrol";
    case PatrolMode::ROOM_INSPECTION: return "Room Inspection";
    case PatrolMode::EMERGENCY_RESPONSE: return "Emergency Response";
    default: return "Unknown";
  }
}

}  // namespace sigyn_house_patroller
