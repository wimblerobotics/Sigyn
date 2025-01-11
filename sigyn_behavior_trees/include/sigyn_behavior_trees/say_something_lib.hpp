// Copyright (c) 2020 Samsung Research America
// This code is licensed under MIT license (see LICENSE.txt for details)

#ifndef NAV2_SMS_RECOVEY__SMS_RECOVERY_HPP_
#define NAV2_SMS_RECOVEY__SMS_RECOVERY_HPP_

// #include <chrono>
// #include <string>
// #include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "sigyn_behavior_trees/action/say_something.hpp"

namespace sigyn_behavior_trees
{

using namespace nav2_behaviors;  // NOLINT
using Action = sigyn_behavior_trees::action::SaySomething;

class SaySomething : public TimedBehavior<Action>
{
public:
  SaySomething();
  ~SaySomething();

  ResultStatus onRun(const std::shared_ptr<const Action::Goal> command) override;

  ResultStatus onCycleUpdate() override;

  void onConfigure() override;

  /**
   * @brief Method to determine the required costmap info
   * @return costmap resources needed
   */
  nav2_core::CostmapInfoType getResourceInfo() override {return nav2_core::CostmapInfoType::NONE;}

protected:
  // std::string _account_sid;
  // std::string _auth_token;
  // std::string _from_number;
  // std::string _to_number;
  // std::shared_ptr<twilio::Twilio> _twilio;
};

}  // namespace sigyn_behavior_trees

#endif  // NAV2_SMS_RECOVEY__SMS_RECOVERY_HPP_