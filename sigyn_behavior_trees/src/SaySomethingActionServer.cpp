#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sigyn_behavior_trees/action/say_something.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace sigyn_behavior_trees {
/**
 * @class sigyn_behavior_trees::SaySomethingServer
 * @brief An action server which implements charger docking node for AMRs
 */
class SaySomethingActionServer : public rclcpp::Node {
 public:
  using SaySomething = sigyn_behavior_trees::action::SaySomething;
  using GoalHandleSaySomething = rclcpp_action::ServerGoalHandle<SaySomething>;

  /**
   * @brief A constructor for sigyn_behavior_trees::SaySomethingServer
   * @param options Additional options to control creation of the node.
   */
  explicit SaySomethingActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("say_something_server", options) {
    using namespace std::placeholders;

    auto handle_goal = [this](const rclcpp_action::GoalUUID& uuid,
                              std::shared_ptr<const SaySomething::Goal> goal) {
      // RCLCPP_INFO(this->get_logger(), "Received goal request with message %s",
      // goal->message.c_str());
      (void)goal;
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel = [this](const std::shared_ptr<GoalHandleSaySomething> goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted = [this](const std::shared_ptr<GoalHandleSaySomething> goal_handle) {
      // this needs to return quickly to avoid blocking the executor,
      // so we declare a lambda function to be called inside a new thread
      auto execute_in_thread = [this, goal_handle]() { return this->execute(goal_handle); };
      std::thread{execute_in_thread}.detach();
    };

    this->action_server_ = rclcpp_action::create_server<SaySomething>(
        this, "say_something", handle_goal, handle_cancel, handle_accepted);
  }

  /**
   * @brief A destructor for sigyn_behavior_trees::SaySomethingServer
   */
  ~SaySomethingActionServer() = default;

 private:
  rclcpp_action::Server<SaySomething>::SharedPtr action_server_;

  void execute(const std::shared_ptr<GoalHandleSaySomething> goal_handle) {
    const auto message = goal_handle->get_goal()->message;
    const auto pose = goal_handle->get_goal()->pose;
    tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
                      pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), "[SaySomething] [x: %4.3f, y: %4.3f, z-rad: %4.3f] %s",
                pose.pose.position.x, pose.pose.position.y, yaw, message.c_str());
  }
};

}  // namespace sigyn_behavior_trees

RCLCPP_COMPONENTS_REGISTER_NODE(sigyn_behavior_trees::SaySomethingActionServer)