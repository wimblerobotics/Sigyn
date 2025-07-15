#include "sigyn_house_patroller/behavior_tree/behavior_tree_manager.hpp"
#include <filesystem>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace sigyn_house_patroller {

BehaviorTreeManager::BehaviorTreeManager(const rclcpp::NodeOptions& options)
    : Node("behavior_tree_manager", options),
      current_state_(BehaviorTreeState::NORMAL_PATROL),
      is_switching_(false),
      system_health_score_(1.0),
      threat_level_(0),
      battery_critical_(false),
      maintenance_mode_(false) {
  
  // Declare parameters
  declare_parameter("config_directory", "config/behavior_trees");
  declare_parameter("default_behavior_tree", "normal_patrol.xml");
  declare_parameter("switch_cooldown", 5.0);
  declare_parameter("bt_navigator_node", "bt_navigator");
  declare_parameter("lifecycle_manager_node", "lifecycle_manager_navigation");
  declare_parameter("monitoring_frequency", 2.0);
  declare_parameter("status_frequency", 0.5);
  
  // Get parameters
  config_directory_ = get_parameter("config_directory").as_string();
  switch_cooldown_ = get_parameter("switch_cooldown").as_double();
  
  // Initialize with default behavior tree
  current_behavior_tree_ = get_parameter("default_behavior_tree").as_string();
  target_behavior_tree_ = current_behavior_tree_;
  
  // Load behavior tree configurations
  LoadBehaviorTreeConfigs();
  
  // Publishers
  bt_status_pub_ = create_publisher<std_msgs::msg::String>(
    "~/behavior_tree_status", rclcpp::QoS(10).reliable());
  
  // Subscribers
  threat_alert_sub_ = create_subscription<msg::ThreatAlert>(
    "/sigyn_house_patroller/threat_alerts", rclcpp::QoS(10).reliable(),
    std::bind(&BehaviorTreeManager::ThreatAlertCallback, this, std::placeholders::_1));
  
  system_health_sub_ = create_subscription<msg::SystemHealth>(
    "/sigyn_house_patroller/system_health", rclcpp::QoS(10).reliable(),
    std::bind(&BehaviorTreeManager::SystemHealthCallback, this, std::placeholders::_1));
  
  patrol_state_sub_ = create_subscription<msg::PatrolState>(
    "/sigyn_house_patroller/patrol_state", rclcpp::QoS(10).reliable(),
    std::bind(&BehaviorTreeManager::PatrolStateCallback, this, std::placeholders::_1));
  
  // Service clients
  lifecycle_client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>(
    "/lifecycle_manager_navigation/manage_nodes");
  
  set_params_client_ = create_client<rcl_interfaces::srv::SetParameters>(
    "/bt_navigator/set_parameters");
  
  change_state_client_ = create_client<lifecycles_msgs::srv::ChangeState>(
    "/bt_navigator/change_state");
  
  // Service server
  switch_service_ = create_service<std_srvs::srv::SetBool>(
    "~/switch_behavior_tree",
    std::bind(&BehaviorTreeManager::SwitchServiceCallback, this, 
              std::placeholders::_1, std::placeholders::_2));
  
  // Timers
  monitoring_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / get_parameter("monitoring_frequency").as_double())),
    std::bind(&BehaviorTreeManager::MonitoringTimerCallback, this));
  
  status_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / get_parameter("status_frequency").as_double())),
    std::bind(&BehaviorTreeManager::StatusTimerCallback, this));
  
  last_switch_time_ = std::chrono::steady_clock::now();
  
  RCLCPP_INFO(get_logger(), "Behavior Tree Manager initialized - Current BT: %s", 
              current_behavior_tree_.c_str());
}

void BehaviorTreeManager::LoadBehaviorTreeConfigs() {
  try {
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("sigyn_house_patroller");
    std::string full_config_dir = package_share_dir + "/" + config_directory_;
    
    // Define behavior tree configurations
    behavior_trees_["normal_patrol.xml"] = {
      full_config_dir + "/normal_patrol.xml",
      "Standard house patrol behavior tree",
      1,
      false
    };
    
    behavior_trees_["threat_investigation.xml"] = {
      full_config_dir + "/threat_investigation.xml",
      "Behavior tree for investigating detected threats",
      3,
      true
    };
    
    behavior_trees_["emergency_response.xml"] = {
      full_config_dir + "/emergency_response.xml",
      "Emergency response behavior tree for high-priority threats",
      5,
      true
    };
    
    behavior_trees_["battery_critical.xml"] = {
      full_config_dir + "/battery_critical.xml",
      "Behavior tree for critical battery situations",
      4,
      true
    };
    
    behavior_trees_["maintenance_mode.xml"] = {
      full_config_dir + "/maintenance_mode.xml",
      "Behavior tree for maintenance and diagnostics",
      2,
      false
    };
    
    RCLCPP_INFO(get_logger(), "Loaded %zu behavior tree configurations", behavior_trees_.size());
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to load behavior tree configs: %s", e.what());
  }
}

void BehaviorTreeManager::ThreatAlertCallback(const msg::ThreatAlert::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  // Update threat level based on alert severity
  if (msg->severity == msg::ThreatAlert::SEVERITY_CRITICAL) {
    threat_level_ = 3;
  } else if (msg->severity == msg::ThreatAlert::SEVERITY_HIGH) {
    threat_level_ = 2;
  } else if (msg->severity == msg::ThreatAlert::SEVERITY_WARNING) {
    threat_level_ = 1;
  }
  
  RCLCPP_INFO(get_logger(), "Threat alert received: %s (level %d)", 
              msg->description.c_str(), threat_level_);
  
  // Trigger behavior tree evaluation
  DetermineOptimalBehaviorTree();
}

void BehaviorTreeManager::SystemHealthCallback(const msg::SystemHealth::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  system_health_score_ = msg->overall_health;
  
  // Check for critical battery
  for (const auto& status : msg->component_status) {
    if (status.find("battery") != std::string::npos && 
        status.find("CRITICAL") != std::string::npos) {
      battery_critical_ = true;
      break;
    }
  }
  
  // Trigger behavior tree evaluation if health is degraded
  if (system_health_score_ < 0.8) {
    DetermineOptimalBehaviorTree();
  }
}

void BehaviorTreeManager::PatrolStateCallback(const msg::PatrolState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  // Update maintenance mode based on patrol state
  maintenance_mode_ = (msg->current_state == "maintenance");
  
  DetermineOptimalBehaviorTree();
}

void BehaviorTreeManager::MonitoringTimerCallback() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  // Periodic evaluation of optimal behavior tree
  DetermineOptimalBehaviorTree();
  
  // Reset threat level if no recent alerts
  static auto last_threat_reset = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::seconds>(now - last_threat_reset).count() > 30) {
    if (threat_level_ > 0) {
      threat_level_--;
      last_threat_reset = now;
    }
  }
}

void BehaviorTreeManager::StatusTimerCallback() {
  PublishCurrentStatus();
}

void BehaviorTreeManager::DetermineOptimalBehaviorTree() {
  BehaviorTreeState new_state = EvaluateSystemState();
  std::string optimal_tree = GetBehaviorTreeForState(new_state);
  
  if (optimal_tree != current_behavior_tree_ && ShouldSwitchBehaviorTree(optimal_tree)) {
    target_behavior_tree_ = optimal_tree;
    current_state_ = new_state;
    
    RCLCPP_INFO(get_logger(), "Scheduling behavior tree switch: %s -> %s", 
                current_behavior_tree_.c_str(), optimal_tree.c_str());
    
    // Execute the switch asynchronously
    std::thread([this]() {
      ExecuteBehaviorTreeSwitch(target_behavior_tree_);
    }).detach();
  }
}

BehaviorTreeManager::BehaviorTreeState BehaviorTreeManager::EvaluateSystemState() {
  // Priority order: Emergency > Battery Critical > Maintenance > Threat Investigation > Normal
  
  if (threat_level_ >= 3 || system_health_score_ < 0.3) {
    return BehaviorTreeState::EMERGENCY_RESPONSE;
  }
  
  if (battery_critical_) {
    return BehaviorTreeState::BATTERY_CRITICAL;
  }
  
  if (maintenance_mode_) {
    return BehaviorTreeState::MAINTENANCE_MODE;
  }
  
  if (threat_level_ >= 1) {
    return BehaviorTreeState::THREAT_INVESTIGATION;
  }
  
  return BehaviorTreeState::NORMAL_PATROL;
}

std::string BehaviorTreeManager::GetBehaviorTreeForState(BehaviorTreeState state) {
  switch (state) {
    case BehaviorTreeState::EMERGENCY_RESPONSE:
      return "emergency_response.xml";
    case BehaviorTreeState::BATTERY_CRITICAL:
      return "battery_critical.xml";
    case BehaviorTreeState::MAINTENANCE_MODE:
      return "maintenance_mode.xml";
    case BehaviorTreeState::THREAT_INVESTIGATION:
      return "threat_investigation.xml";
    case BehaviorTreeState::NORMAL_PATROL:
    default:
      return "normal_patrol.xml";
  }
}

bool BehaviorTreeManager::ShouldSwitchBehaviorTree(const std::string& new_tree) {
  // Check cooldown period
  auto now = std::chrono::steady_clock::now();
  auto time_since_last_switch = std::chrono::duration_cast<std::chrono::seconds>(
    now - last_switch_time_).count();
  
  if (time_since_last_switch < switch_cooldown_) {
    return false;
  }
  
  // Check if already switching
  if (is_switching_) {
    return false;
  }
  
  // Check if new tree exists
  if (!ValidateBehaviorTreeExists(new_tree)) {
    return false;
  }
  
  // Check priority (higher priority can interrupt lower priority)
  if (behavior_trees_.find(new_tree) != behavior_trees_.end() &&
      behavior_trees_.find(current_behavior_tree_) != behavior_trees_.end()) {
    return behavior_trees_[new_tree].priority > behavior_trees_[current_behavior_tree_].priority;
  }
  
  return true;
}

bool BehaviorTreeManager::ExecuteBehaviorTreeSwitch(const std::string& tree_name) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  if (is_switching_) {
    RCLCPP_WARN(get_logger(), "Already switching behavior tree, ignoring request");
    return false;
  }
  
  is_switching_ = true;
  std::string old_tree = current_behavior_tree_;
  
  try {
    // Update bt_navigator parameter
    if (!UpdateBTNavigatorParameter("default_bt_xml_filename", 
                                   behavior_trees_[tree_name].xml_path)) {
      throw std::runtime_error("Failed to update bt_navigator parameter");
    }
    
    // Restart bt_navigator if required
    if (behavior_trees_[tree_name].requires_restart) {
      if (!RestartBTNavigator()) {
        throw std::runtime_error("Failed to restart bt_navigator");
      }
    }
    
    // Update current tree
    current_behavior_tree_ = tree_name;
    last_switch_time_ = std::chrono::steady_clock::now();
    
    LogBehaviorTreeSwitch(old_tree, tree_name, "State change");
    
    is_switching_ = false;
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to switch behavior tree: %s", e.what());
    is_switching_ = false;
    return false;
  }
}

bool BehaviorTreeManager::UpdateBTNavigatorParameter(const std::string& param_name, 
                                                     const std::string& value) {
  if (!set_params_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "bt_navigator parameter service not available");
    return false;
  }
  
  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  
  rcl_interfaces::msg::Parameter param;
  param.name = param_name;
  param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  param.value.string_value = value;
  
  request->parameters.push_back(param);
  
  auto future = set_params_client_->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    if (!response->results.empty() && response->results[0].successful) {
      return true;
    }
  }
  
  return false;
}

bool BehaviorTreeManager::RestartBTNavigator() {
  // Implementation depends on your Nav2 setup
  // This is a simplified version - you may need to adapt based on your lifecycle management
  
  RCLCPP_INFO(get_logger(), "Restarting bt_navigator for behavior tree switch");
  
  // For now, just log that we would restart
  // In a real implementation, you'd use the lifecycle management services
  
  return true;
}

void BehaviorTreeManager::PublishCurrentStatus() {
  std_msgs::msg::String status_msg;
  status_msg.data = "current_bt:" + current_behavior_tree_ + 
                   ",target_bt:" + target_behavior_tree_ + 
                   ",switching:" + (is_switching_ ? "true" : "false") + 
                   ",threat_level:" + std::to_string(threat_level_) + 
                   ",health:" + std::to_string(system_health_score_);
  
  bt_status_pub_->publish(status_msg);
}

bool BehaviorTreeManager::ValidateBehaviorTreeExists(const std::string& tree_name) {
  auto it = behavior_trees_.find(tree_name);
  if (it == behavior_trees_.end()) {
    return false;
  }
  
  return std::filesystem::exists(it->second.xml_path);
}

void BehaviorTreeManager::LogBehaviorTreeSwitch(const std::string& from, 
                                                const std::string& to, 
                                                const std::string& reason) {
  RCLCPP_INFO(get_logger(), "Behavior tree switched: %s -> %s (reason: %s)", 
              from.c_str(), to.c_str(), reason.c_str());
}

void BehaviorTreeManager::SwitchServiceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  
  // For manual switching, we'll use the request->data as a simple toggle
  // In a real implementation, you'd want a more sophisticated service
  
  std::string target_tree = request->data ? "threat_investigation.xml" : "normal_patrol.xml";
  
  bool success = ExecuteBehaviorTreeSwitch(target_tree);
  
  response->success = success;
  response->message = success ? "Behavior tree switched successfully" : "Failed to switch behavior tree";
}

bool BehaviorTreeManager::SwitchBehaviorTree(const std::string& tree_name) {
  return ExecuteBehaviorTreeSwitch(tree_name);
}

std::string BehaviorTreeManager::GetCurrentBehaviorTree() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_behavior_tree_;
}

bool BehaviorTreeManager::IsSwitching() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return is_switching_;
}

}  // namespace sigyn_house_patroller
