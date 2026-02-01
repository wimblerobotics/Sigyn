// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

#include <memory>
#include <string>
#include <chrono>
#include <signal.h>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "can_do_challenge/bt_nodes_real.hpp"

using namespace std::chrono_literals;

// Global flag for signal handling
std::atomic<bool> g_shutdown_requested{false};

void signal_handler(int signal) {
  (void)signal;
  g_shutdown_requested = true;
  RCLCPP_INFO(rclcpp::get_logger("can_do_challenge"), "Shutdown requested");
}

namespace can_do_challenge
{

class CanDoChallengeNode : public rclcpp::Node
{
public:
  CanDoChallengeNode()
  : Node("can_do_challenge_node"),
    step_manually_(false),
    tick_requested_(false)
  {
    // Create a separate callback group for the BT execution to avoid blocking sensor callbacks
    bt_timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Note: use_sim_time is automatically declared by rclcpp::Node, don't declare it again
    this->declare_parameter<std::string>("bt_xml_filename", "");
    this->declare_parameter<bool>("enable_groot_monitoring", true);
    this->declare_parameter<int>("groot_port", 1667);
    this->declare_parameter<bool>("step_manually", false);
    
    std::string bt_xml_filename = this->get_parameter("bt_xml_filename").as_string();
    bool enable_groot = this->get_parameter("enable_groot_monitoring").as_bool();
    int groot_port = this->get_parameter("groot_port").as_int();
    step_manually_ = this->get_parameter("step_manually").as_bool();

    if (bt_xml_filename.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'bt_xml_filename' is not set.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loading Behavior Tree from: %s", bt_xml_filename.c_str());

    BT::BehaviorTreeFactory factory;

    // Create shared pointer to this node for BT nodes that need ROS access
    auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});

    // --- Register all custom nodes ---
    
    // Safety Conditions
    factory.registerNodeType<BatteryAboveChargingVoltage>("BatteryAboveChargingVoltage");
    factory.registerNodeType<BatteryAboveCriticalVoltage>("BatteryAboveCriticalVoltage");
    factory.registerNodeType<RobotIsEstopped>("RobotIsEstopped");
    factory.registerNodeType<RobotTiltedCritically>("RobotTiltedCritically");
    factory.registerNodeType<RobotTiltedWarning>("RobotTiltedWarning");
    
    // Vision Conditions
    factory.registerNodeType<CanDetectedByOAKD>("CanDetectedByOAKD");
    factory.registerNodeType<CanDetectedByPiCamera>("CanDetectedByPiCamera");
    factory.registerNodeType<CanCenteredInPiCamera>("CanCenteredInPiCamera");
    factory.registerNodeType<CanWithinReach>("CanWithinReach");
    factory.registerNodeType<CanIsGrasped>("CanIsGrasped");
    factory.registerNodeType<WaitForNewPiFrameProcessed>("WaitForNewPiFrameProcessed");
    factory.registerNodeType<ElevatorAtHeight>("ElevatorAtHeight");

    {
      const auto& manifests = factory.manifests();
      auto it = manifests.find("CanWithinReach");
      if (it != manifests.end()) {
        std::string ports;
        for (const auto& entry : it->second.ports) {
          if (!ports.empty()) {
            ports += ", ";
          }
          ports += entry.first;
        }
        RCLCPP_INFO(this->get_logger(), "CanWithinReach ports: [%s]", ports.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "CanWithinReach manifest not found in factory");
      }
    }
    
    // Navigation Actions
    factory.registerNodeType<ComputePathToCanLocation>("ComputePathToCanLocation");
    factory.registerNodeType<ComputePathToPose>("ComputePathToPose");
    factory.registerNodeType<FollowPath>("FollowPath");
    factory.registerNodeType<NavigateToPoseAction>("NavigateToPoseAction");
    factory.registerNodeType<ComputeApproachGoalToCan>("ComputeApproachGoalToCan");
    factory.registerNodeType<MoveTowardsCan>("MoveTowardsCan");
    factory.registerNodeType<RotateRobot>("RotateRobot");
    
    // Gripper/Elevator Actions
    factory.registerNodeType<LowerElevator>("LowerElevator");
    factory.registerNodeType<LowerElevatorSafely>("LowerElevatorSafely");
    factory.registerNodeType<LowerElevatorToTable>("LowerElevatorToTable");
    factory.registerNodeType<MoveElevatorToHeight>("MoveElevatorToHeight");
    factory.registerNodeType<StepElevatorUp>("StepElevatorUp");
    factory.registerNodeType<BackAwayFromTable>("BackAwayFromTable");
    factory.registerNodeType<ComputeElevatorHeight>("ComputeElevatorHeight");
    factory.registerNodeType<RetractExtender>("RetractExtender");
    factory.registerNodeType<RetractGripper>("RetractGripper");
    factory.registerNodeType<OpenGripper>("OpenGripper");
    factory.registerNodeType<CloseGripperAroundCan>("CloseGripperAroundCan");
    factory.registerNodeType<ExtendTowardsCan>("ExtendTowardsCan");
    factory.registerNodeType<AdjustExtenderToCenterCan>("AdjustExtenderToCenterCan");
    
    // Setup/Utility Actions
    factory.registerNodeType<SaveRobotPose>("SaveRobotPose");
    factory.registerNodeType<LoadCanLocation>("LoadCanLocation");
    factory.registerNodeType<ChargeBattery>("ChargeBattery");
    factory.registerNodeType<ShutdownSystem>("ShutdownSystem");
    factory.registerNodeType<SoftwareEStop>("SoftwareEStop");
    factory.registerNodeType<WaitForDetection>("WaitForDetection");
    factory.registerNodeType<WaitForNewOAKDFrame>("WaitForNewOAKDFrame");
    factory.registerNodeType<SleepSeconds>("SleepSeconds");
    factory.registerNodeType<ReportGraspFailure>("ReportGraspFailure");
    factory.registerNodeType<SaySomething>("SaySomething");
    
    // Custom Decorator
    factory.registerNodeType<ReactiveRepeatUntilSuccessOrCount>("ReactiveRepeatUntilSuccessOrCount");
    factory.registerNodeType<CheckBoolFlag>("CheckBoolFlag");

    // Create the behavior tree
    try {
      tree_ = factory.createTreeFromFile(bt_xml_filename);
      
      // Set ROS node for all custom nodes
      for (auto& node : tree_.nodes) {
        if (auto ros_node = dynamic_cast<RosNodeBT*>(node.get())) {
          ros_node->setRosNode(node_ptr);
        }
      }
      
      RCLCPP_INFO(this->get_logger(), "Behavior Tree loaded successfully");
      
      // Setup Groot monitoring if enabled
      if (enable_groot) {
        groot_publisher_ = std::make_unique<BT::PublisherZMQ>(tree_, 10, groot_port, groot_port + 1);
        RCLCPP_INFO(this->get_logger(), "Groot monitoring enabled on port %d", groot_port);
      }
      
      // Setup manual stepping if enabled
      if (step_manually_) {
        RCLCPP_INFO(this->get_logger(), "Manual stepping mode enabled");
        tick_sub_ = this->create_subscription<std_msgs::msg::Bool>(
          "/can_do_challenge/bt_tick",
          10,
          std::bind(&CanDoChallengeNode::tickCommandCallback, this, std::placeholders::_1));
        
        // Create timer that checks for tick requests
        timer_ = this->create_wall_timer(
          10ms,  // Check frequently for tick requests
          std::bind(&CanDoChallengeNode::checkForTickRequest, this),
          bt_timer_cb_group_);
      } else {
        // Automatic mode - tick continuously
        RCLCPP_INFO(this->get_logger(), "Automatic execution mode");
        timer_ = this->create_wall_timer(
          100ms,
          std::bind(&CanDoChallengeNode::tickTree, this),
          bt_timer_cb_group_);
      }
        
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error loading behavior tree: %s", e.what());
      rclcpp::shutdown();
    }
  }

private:
  void tickCommandCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && step_manually_) {
      tick_requested_ = true;
      RCLCPP_INFO(this->get_logger(), "Manual tick requested");
    }
  }
  
  void checkForTickRequest()
  {
    if (tick_requested_) {
      tick_requested_ = false;
      tickTree();
    }
  }
  void tickTree()
  {
    if (g_shutdown_requested) {
      RCLCPP_INFO(this->get_logger(), "Stopping behavior tree execution");
      timer_->cancel();
      rclcpp::shutdown();
      return;
    }

    BT::NodeStatus status = tree_.tickRoot();
    
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Behavior tree completed successfully!");
      timer_->cancel();
      rclcpp::shutdown();
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(this->get_logger(), "Behavior tree failed!");
      timer_->cancel();
      rclcpp::shutdown();
    } else if (status == BT::NodeStatus::IDLE) {
      RCLCPP_WARN(this->get_logger(), "Behavior tree returned IDLE. Shutting down.");
      timer_->cancel();
      rclcpp::shutdown();
    }
    // If RUNNING, continue ticking
  }

  BT::Tree tree_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr bt_timer_cb_group_;
  std::unique_ptr<BT::PublisherZMQ> groot_publisher_;
  
  // Manual stepping support
  bool step_manually_;
  std::atomic<bool> tick_requested_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tick_sub_;
};

}  // namespace can_do_challenge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // Setup signal handler
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  
  auto node = std::make_shared<can_do_challenge::CanDoChallengeNode>();

  RCLCPP_INFO(node->get_logger(), "Can Do Challenge node started");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  
  return 0;
}
