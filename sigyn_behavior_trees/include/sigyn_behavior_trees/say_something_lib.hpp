#pragma once
#include <nav2_behavior_tree/bt_action_node.hpp>

#include "sigyn_behavior_trees/action/say_something.hpp"

namespace sigyn_behavior_trees {

class SaySomething : public nav2_behavior_tree::BtActionNode<sigyn_behavior_trees::action::SaySomething> {
 public:
  SaySomething(const std::string& xml_tag_name, const std::string& action_name,
               const BT::NodeConfiguration& conf);
  ~SaySomething();

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Function to perform some user-defined operation upon cancellation of the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * \brief Override required by the a BT action. Cancel the action and set the path output
   */
  void halt() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() {
    return providedBasicPorts({
        BT::InputPort<std::string>("message", "Message to log"),
    });
  }
};

}  // namespace sigyn_behavior_trees