#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/wait.hpp"

using WaitAction = nav2_msgs::action::Wait;

class Wait : public TimedBehavior<WaitAction> {
 public:
  /**
   * @brief A constructor for nav2_behaviors::Wait
   */
  Wait(std::string name, const BT::NodeConfiguration& config);
  ~Wait();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  ResultStatus onRun(const std::shared_ptr<const WaitActionGoal> command) override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  ResultStatus onCycleUpdate() override;

  /**
   * @brief Method to determine the required costmap info
   * @return costmap resources needed
   */
  CostmapInfoType getResourceInfo() override { return CostmapInfoType::LOCAL; }

 protected:
  rclcpp::Time wait_end_;
  WaitAction::Feedback::SharedPtr feedback_;
};