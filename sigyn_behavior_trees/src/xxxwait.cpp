// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_behaviors/plugins/wait.hpp"

#include <chrono>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "wait.hpp"

Wait::Wait(std::string name, const BT::NodeConfiguration& config);
    : TimedBehavior<WaitAction>(), feedback_(std::make_shared<WaitAction::Feedback>()) {}

    Wait::~Wait() = default;

    ResultStatus Wait::onRun(const std::shared_ptr<const WaitAction::Goal> command) {
      wait_end_ = node_.lock()->now() + rclcpp::Duration(command->time);
      return ResultStatus{Status::SUCCEEDED};
    }

    ResultStatus Wait::onCycleUpdate() {
      auto current_point = node_.lock()->now();
      auto time_left = wait_end_ - current_point;

      feedback_->time_left = time_left;
      action_server_->publish_feedback(feedback_);

      if (time_left.nanoseconds() > 0) {
        return ResultStatus{Status::RUNNING};
      } else {
        return ResultStatus{Status::SUCCEEDED};
      }
    }

#include "pluginlib/class_list_macros.hpp"
    PLUGINLIB_EXPORT_CLASS(nav2_behaviors::Wait, nav2_core::Behavior)
