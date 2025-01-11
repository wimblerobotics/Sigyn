#include "sigyn_behavior_trees/say_something_lib.hpp"
namespace sigyn_behavior_trees
{

SaySomething::SaySomething()
: TimedBehavior<Action>()
{
}

SaySomething::~SaySomething()
{
}

void SaySomething::onConfigure()
{
  auto node = node_.lock();
  // node->declare_parameter("account_sid","");
  // _account_sid = node->get_parameter("account_sid").as_string();
  // node->declare_parameter("auth_token","");
  // _auth_token = node->get_parameter("auth_token").as_string();
  // node->declare_parameter("from_number","");
  // _from_number = node->get_parameter("from_number").as_string();
  // node->declare_parameter("to_number","");
  // _to_number = node->get_parameter("to_number").as_string();
  // _twilio = std::make_shared<twilio::Twilio>(_account_sid, _auth_token);
}

ResultStatus SaySomething::onRun(const std::shared_ptr<const Action::Goal> command)
{
  // auto node = node_.lock();
  // std::string response;
  // bool message_success = _twilio->send_message(
  //   _to_number,
  //   _from_number,
  //   command->message,
  //   response,
  //   "",
  //   false);
  // if (!message_success) {
  //   RCLCPP_INFO(node->get_logger(), "SMS send failed.");
  //   return ResultStatus{Status::FAILED};
  // }

  // RCLCPP_INFO(node->get_logger(), "SMS sent successfully!");
  return ResultStatus{Status::SUCCEEDED};
}

ResultStatus SaySomething::onCycleUpdate()
{
  return ResultStatus{Status::SUCCEEDED};
}

}  // namespace nav2_sms_behavior

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(sigyn_behavior_trees::SaySomething, nav2_core::Behavior)

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::WaitAction>(name, "wait", config);
    };

  factory.registerBuilder<nav2_behavior_tree::WaitAction>("Wait", builder);
}