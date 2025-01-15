#pragma once
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "sigyn_behavior_trees/action/move_a_short_distance_ahead.hpp"

namespace sigyn_behavior_trees {

class MoveAShortDistanceAhead
    : public nav2_behavior_tree::BtActionNode<sigyn_behavior_trees::action::MoveAShortDistanceAhead> {
 public:
  MoveAShortDistanceAhead(const std::string& xml_tag_name, const std::string& action_name,
                          const BT::NodeConfiguration& conf);
  ~MoveAShortDistanceAhead();

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  // /**
  //  * @brief Function to perform some user-defined operation upon successful completion of the
  //  action
  //  */
  // BT::NodeStatus on_success() override;

  // /**
  //  * @brief Function to perform some user-defined operation upon abortion of the action
  //  */
  // BT::NodeStatus on_aborted() override;

  // /**
  //  * @brief Function to perform some user-defined operation upon cancellation of the action
  //  */
  // BT::NodeStatus on_cancelled() override;


  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("distance", 0.0, "Distance to move ahead"),
      });
  }
};

}  // namespace sigyn_behavior_trees