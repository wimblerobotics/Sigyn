#include "sigyn_house_patroller/core/navigation_coordinator.hpp"
#include "sigyn_house_patroller/core/waypoint_manager.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace sigyn_house_patroller {

// Constructor
NavigationCoordinator::NavigationCoordinator(rclcpp::Node::SharedPtr node)
    : node_(node),
      current_status_(NavigationStatus::kIdle),
      navigation_timeout_(std::chrono::seconds(60)), // Default timeout
      total_distance_traveled_(0.0),
      successful_navigations_(0),
      failed_navigations_(0),
      map_frame_("map"),
      robot_frame_("base_link"),
      stuck_threshold_(0.1),
      min_movement_distance_(0.05),
      max_pose_history_(20),
      stuck_timeout_(std::chrono::seconds(10))
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// Initialize
bool NavigationCoordinator::Initialize() {
    RCLCPP_INFO(node_->get_logger(), "Initializing NavigationCoordinator...");

    nav_action_client_ = rclcpp_action::create_client<NavigateToAction>(node_, "navigate_to_pose");
    if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node_->get_logger(), "NavigateToPose action server not available!");
        return false;
    }

    get_costmap_client_ = node_->create_client<nav2_msgs::srv::GetCostmap>("global_costmap/get_costmap");
    
    timeout_timer_ = node_->create_wall_timer(
        std::chrono::seconds(1), 
        std::bind(&NavigationCoordinator::CheckNavigationTimeout, this)
    );

    RCLCPP_INFO(node_->get_logger(), "NavigationCoordinator initialized successfully.");
    return true;
}

// NavigateToWaypoint
bool NavigationCoordinator::NavigateToWaypoint(const PatrolWaypoint& waypoint,
                                             NavigationCallback callback,
                                             FeedbackCallback feedback_callback) {
    return NavigateToPose(waypoint.pose, callback, feedback_callback);
}

// NavigateToPose
bool NavigationCoordinator::NavigateToPose(const geometry_msgs::msg::PoseStamped& pose,
                                         NavigationCallback callback,
                                         FeedbackCallback feedback_callback) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (current_status_ == NavigationStatus::kNavigating) {
        RCLCPP_WARN(node_->get_logger(), "Already navigating, cannot start a new goal.");
        return false;
    }

    navigation_callback_ = callback;
    feedback_callback_ = feedback_callback;
    
    return ExecuteNavigationGoal(pose);
}

// CancelNavigation
bool NavigationCoordinator::CancelNavigation() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (!current_goal_handle_) {
        RCLCPP_WARN(node_->get_logger(), "No active navigation goal to cancel.");
        return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Cancelling navigation goal.");
    auto future_cancel = nav_action_client_->async_cancel_goal(current_goal_handle_);
    
    current_status_ = NavigationStatus::kCancelled;
    last_result_.status = current_status_;
    last_result_.message = "Navigation was cancelled by user.";
    
    if (navigation_callback_) {
        navigation_callback_(last_result_);
    }

    current_goal_handle_.reset();
    return true;
}

// IsNavigating
bool NavigationCoordinator::IsNavigating() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return current_status_ == NavigationStatus::kNavigating;
}

// GetNavigationStatus
NavigationStatus NavigationCoordinator::GetNavigationStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return current_status_;
}

// GetLastResult
NavigationResult NavigationCoordinator::GetLastResult() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return last_result_;
}

// SetNavigationTimeout
void NavigationCoordinator::SetNavigationTimeout(const std::chrono::seconds& timeout) {
    navigation_timeout_ = timeout;
}

// GetEstimatedTravelTime (Stub)
std::chrono::seconds NavigationCoordinator::GetEstimatedTravelTime(const PatrolWaypoint& waypoint) const {
    // Simple estimation based on distance
    double distance = CalculateDistanceToWaypoint(waypoint);
    double average_speed = 0.5; // m/s
    return std::chrono::seconds(static_cast<long>(distance / average_speed));
}

// IsPathClear (Stub)
bool NavigationCoordinator::IsPathClear(const PatrolWaypoint& waypoint) const {
    (void)waypoint;
    RCLCPP_WARN(node_->get_logger(), "IsPathClear is a stub and always returns true.");
    return true;
}

// GetCurrentPose
geometry_msgs::msg::PoseStamped NavigationCoordinator::GetCurrentPose() const {
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header.frame_id = robot_frame_;
    robot_pose.header.stamp = node_->get_clock()->now();
    robot_pose.pose.orientation.w = 1.0; // Neutral orientation

    try {
        return tf_buffer_->transform(robot_pose, map_frame_);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "Could not transform from %s to %s: %s",
                     robot_frame_.c_str(), map_frame_.c_str(), ex.what());
        return geometry_msgs::msg::PoseStamped(); // Return empty pose on failure
    }
}

// CalculateDistanceToWaypoint
double NavigationCoordinator::CalculateDistanceToWaypoint(const PatrolWaypoint& waypoint) const {
    auto current_pose = GetCurrentPose();
    if (current_pose.header.frame_id.empty()) {
        return -1.0; // Invalid distance
    }
    double dx = current_pose.pose.position.x - waypoint.pose.pose.position.x;
    double dy = current_pose.pose.position.y - waypoint.pose.pose.position.y;
    return std::sqrt(dx*dx + dy*dy);
}

// ClearCostmaps (Stub)
bool NavigationCoordinator::ClearCostmaps() {
    RCLCPP_INFO(node_->get_logger(), "ClearCostmaps called (stub implementation).");
    // In a real implementation, you would call the clear costmaps service
    return true;
}

// IsRobotStuck (Stub)
bool NavigationCoordinator::IsRobotStuck() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    if (pose_history_.size() < max_pose_history_) {
        return false; // Not enough data
    }

    auto now = std::chrono::system_clock::now();
    if ((now - last_movement_time_) > stuck_timeout_) {
        RCLCPP_WARN(node_->get_logger(), "Robot appears to be stuck (timeout).");
        return true;
    }
    return false;
}

// ExecuteRecovery (Stub)
bool NavigationCoordinator::ExecuteRecovery() {
    RCLCPP_INFO(node_->get_logger(), "Executing recovery behavior (stub).");
    ClearCostmaps();
    // Other recovery actions could be added here
    return true;
}

// IsNavigationHealthy (Stub)
bool NavigationCoordinator::IsNavigationHealthy() const {
    // Basic check: is the action server available?
    return nav_action_client_ && nav_action_client_->action_server_is_ready();
}

// GetPerformanceMetrics (Stub)
std::unordered_map<std::string, double> NavigationCoordinator::GetPerformanceMetrics() const {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return {
        {"total_distance_traveled", total_distance_traveled_},
        {"successful_navigations", static_cast<double>(successful_navigations_)},
        {"failed_navigations", static_cast<double>(failed_navigations_)}
    };
}

// --- Private Methods ---

// ExecuteNavigationGoal
bool NavigationCoordinator::ExecuteNavigationGoal(const geometry_msgs::msg::PoseStamped& pose) {
    if (!IsValidPose(pose)) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid goal pose provided.");
        return false;
    }

    auto goal_msg = NavigateToAction::Goal();
    goal_msg.pose = pose;

    auto send_goal_options = rclcpp_action::Client<NavigateToAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavigationCoordinator::NavigationGoalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NavigationCoordinator::NavigationFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&NavigationCoordinator::NavigationResultCallback, this, std::placeholders::_1);

    auto future_goal_handle = nav_action_client_->async_send_goal(goal_msg, send_goal_options);
    
    navigation_start_time_ = std::chrono::system_clock::now();
    current_status_ = NavigationStatus::kNavigating;
    
    RCLCPP_INFO(node_->get_logger(), "Sent navigation goal.");
    return true;
}

// NavigationGoalResponseCallback
void NavigationCoordinator::NavigationGoalResponseCallback(const NavigateToGoalHandle::SharedPtr& goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Navigation goal was rejected by server");
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_ = NavigationStatus::kFailed;
        last_result_.status = current_status_;
        last_result_.message = "Goal rejected by action server.";
        if (navigation_callback_) {
            navigation_callback_(last_result_);
        }
    } else {
        RCLCPP_INFO(node_->get_logger(), "Navigation goal accepted by server, waiting for result");
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_goal_handle_ = goal_handle;
    }
}

// NavigationFeedbackCallback
void NavigationCoordinator::NavigationFeedbackCallback(
    NavigateToGoalHandle::SharedPtr,
    const std::shared_ptr<const NavigateToAction::Feedback> feedback) {
    
    UpdateNavigationMetrics(feedback->current_pose);

    if (feedback_callback_) {
        feedback_callback_(feedback->current_pose, feedback->distance_remaining);
    }
}

// NavigationResultCallback
void NavigationCoordinator::NavigationResultCallback(const NavigateToGoalHandle::WrappedResult& result) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            current_status_ = NavigationStatus::kSuccess;
            successful_navigations_++;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            current_status_ = NavigationStatus::kFailed;
            failed_navigations_++;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            current_status_ = NavigationStatus::kCancelled;
            failed_navigations_++; // Or a separate cancelled count
            break;
        default:
            current_status_ = NavigationStatus::kFailed;
            failed_navigations_++;
            break;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Navigation finished with status: %d", static_cast<int>(current_status_));

    last_result_.status = current_status_;
    last_result_.message = "Completed with code: " + std::to_string(static_cast<int>(result.code));
    last_result_.completion_time = std::chrono::system_clock::now();
    last_result_.duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        last_result_.completion_time - navigation_start_time_
    );

    if (navigation_callback_) {
        navigation_callback_(last_result_);
    }
    current_goal_handle_.reset();
}

// CheckNavigationTimeout
void NavigationCoordinator::CheckNavigationTimeout() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (current_status_ != NavigationStatus::kNavigating) {
        return;
    }

    auto now = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - navigation_start_time_);

    if (elapsed > navigation_timeout_) {
        RCLCPP_WARN(node_->get_logger(), "Navigation timed out after %ld seconds.", navigation_timeout_.count());
        current_status_ = NavigationStatus::kTimeout;
        last_result_.status = current_status_;
        last_result_.message = "Navigation timed out.";
        
        if (current_goal_handle_) {
            auto future_cancel = nav_action_client_->async_cancel_goal(current_goal_handle_);
        }

        if (navigation_callback_) {
            navigation_callback_(last_result_);
        }
        current_goal_handle_.reset();
    }
}

// UpdateNavigationMetrics
void NavigationCoordinator::UpdateNavigationMetrics(const geometry_msgs::msg::PoseStamped& pose) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    if (!pose_history_.empty()) {
        const auto& last_pose = pose_history_.back();
        double dx = pose.pose.position.x - last_pose.pose.position.x;
        double dy = pose.pose.position.y - last_pose.pose.position.y;
        double distance_moved = std::sqrt(dx*dx + dy*dy);
        
        if (distance_moved > min_movement_distance_) {
            total_distance_traveled_ += distance_moved;
            last_movement_time_ = std::chrono::system_clock::now();
        }
    } else {
        last_movement_time_ = std::chrono::system_clock::now();
    }

    pose_history_.push_back(pose);
    if (pose_history_.size() > max_pose_history_) {
        pose_history_.erase(pose_history_.begin());
    }
}

// TransformToMapFrame
geometry_msgs::msg::PoseStamped NavigationCoordinator::TransformToMapFrame(
    const geometry_msgs::msg::PoseStamped& pose) const {
    if (pose.header.frame_id == map_frame_) {
        return pose;
    }
    try {
        return tf_buffer_->transform(pose, map_frame_);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to transform pose to %s frame: %s", map_frame_.c_str(), ex.what());
        return geometry_msgs::msg::PoseStamped();
    }
}

// IsValidPose
bool NavigationCoordinator::IsValidPose(const geometry_msgs::msg::PoseStamped& pose) const {
    // Basic check for now
    return !pose.header.frame_id.empty();
}

}  // namespace sigyn_house_patroller
