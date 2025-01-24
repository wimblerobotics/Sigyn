#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class UnknownPointsFinder : public rclcpp::Node
{
public:
  UnknownPointsFinder() : Node("unknown_points_finder_node")
  {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", qos, std::bind(&UnknownPointsFinder::mapCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&UnknownPointsFinder::odomCallback, this, std::placeholders::_1));
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
      this, "/compute_path_to_pose");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Received a map with %d x %d cells",
      map_.info.width, map_.info.height);
    processMap();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "Received odometry: (%.2f, %.2f)",
    //   msg->pose.pose.position.x, msg->pose.pose.position.y);
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
  }

  void processMap()
  {
    if (map_.data.empty())
      return;

    std::vector<std::pair<float, float>> unknown_points;
    unknown_points.reserve(map_.data.size());

    for (size_t i = 0; i < map_.data.size(); ++i) {
      if (map_.data[i] == -1) { // Unknown cell
        // Convert index to world coordinates
        int row = i / map_.info.width;
        int col = i % map_.info.width;
        float world_x = map_.info.origin.position.x + (col + 0.5f) * map_.info.resolution;
        float world_y = map_.info.origin.position.y + (row + 0.5f) * map_.info.resolution;

        // Check for obstacles between the robot and the point
        if (!isObstacleBetween(robot_x_, robot_y_, world_x, world_y)) {
          unknown_points.push_back({world_x, world_y});
        }
      }
    }

    // Sort by distance to robot
    std::sort(unknown_points.begin(), unknown_points.end(),
      [this](auto &a, auto &b) {
        float distA = std::hypot(a.first - robot_x_, a.second - robot_y_);
        float distB = std::hypot(b.first - robot_x_, b.second - robot_y_);
        return distA < distB;
      });

    RCLCPP_INFO(this->get_logger(), "Unknown points (sorted by distance):");
    uint32_t counter = 0;
    for (auto &pt: unknown_points) {
      float dist = std::hypot(pt.first - robot_x_, pt.second - robot_y_);
      RCLCPP_INFO(this->get_logger(), "  (%.2f, %.2f), dist=%.2f", pt.first, pt.second, dist);
      if (++counter >= 5)
        break;
    }

    if (!unknown_points.empty()) {
      RCLCPP_INFO(this->get_logger(), "Sending goal to the closest unknown point");
      sendGoal(unknown_points.front());
    } else {
      RCLCPP_INFO(this->get_logger(), "No unknown points found");
    }
  }

  bool isObstacleBetween(float x1, float y1, float x2, float y2)
  {
    int steps = static_cast<int>(std::hypot(x2 - x1, y2 - y1) / map_.info.resolution);
    for (int i = 0; i <= steps; ++i) {
      float t = static_cast<float>(i) / steps;
      float x = x1 + t * (x2 - x1);
      float y = y1 + t * (y2 - y1);

      int col = static_cast<int>((x - map_.info.origin.position.x) / map_.info.resolution);
      int row = static_cast<int>((y - map_.info.origin.position.y) / map_.info.resolution);
      int index = row * map_.info.width + col;

      if (index >= 0 && index < static_cast<int>(map_.data.size()) && map_.data[index] == 100) {
        return true; // Obstacle found
      }
    }
    return false; // No obstacle found
  }

  void sendGoal(const std::pair<float, float>& target)
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
    goal_msg.goal.pose.position.x = target.first;
    goal_msg.goal.pose.position.y = target.second;
    goal_msg.goal.pose.orientation.w = 1.0; // Neutral orientation

    RCLCPP_INFO(this->get_logger(), "Sending goal: (%.2f, %.2f)", target.first, target.second);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto) {
      RCLCPP_INFO(this->get_logger(), "Goal result received");
    };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::OccupancyGrid map_;
  float robot_x_{0.0f};
  float robot_y_{0.0f};
  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr action_client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UnknownPointsFinder>());
  rclcpp::shutdown();
  return 0;
}