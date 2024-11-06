#include <getopt.h>
#include <rcutils/logging_macros.h>
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <rclcpp/rclcpp.hpp>

// #include <exception>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fstream>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

#include "std_msgs/msg/string.hpp"

uint32_t count_wheel_odometry_callbacks = 0;
double last_x = 0.0;
double last_y = 0.0;
double last_orientation_w = 0.0;
double last_orientation_x = 0.0;
double last_orientation_y = 0.0;
double last_orientation_z = 0.0;
rclcpp::Time last_timestamp = rclcpp::Time(0, 0);
double loop_rate_hz;
double max_skew = 0.0;
rclcpp::Time previous_timestamp = rclcpp::Time(0, 0);
rclcpp::QoS qos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
rclcpp::Node::SharedPtr ros_node;
const uint8_t kNumberSkews = 20;
uint32_t skews_by_10ms[kNumberSkews] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teensy_diagnostics_subscriber;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teensy_stats_subscriber;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odometry_subscriber;

std::vector<std::string> diagnostic_messages;
std::vector<std::string> stats_messages;
bool new_max_skew = false;

// template <typename T>
// T getTopicElementValue(YAML::Node topics, int sequence, std::string elementName) {
//   try {
//     return topics[sequence][elementName.c_str()].as<T>();
//   } catch (std::exception &e) {
//     RCUTILS_LOG_FATAL(
//         "[motor_characterization::getTopicElementValue] topic sequence %d does not have a valid "
//         "'%s' value",
//         (int)sequence, elementName.c_str());
//     exit(-1);
//   }
// }

void processConfiguration(int argc, char *argv[]) {
  static const struct option long_options[] = {{"config", required_argument, nullptr, 'c'},
                                               {"ros-args", optional_argument, nullptr, 'r'},
                                               {nullptr, 0, nullptr, 0}};

  char *yaml_path = nullptr;
  int opt = 0;
  int option_index = 0;
  while ((opt = getopt_long_only(argc, argv, "c:r", long_options, &option_index)) != -1) {
    switch (opt) {
      case 'c':
        yaml_path = optarg;
        break;
    }
  }

  std::ifstream yaml_file;
  yaml_file.open(yaml_path, std::ifstream::in);
  YAML::Node config = YAML::Load(yaml_file);
  if (!config.IsMap()) {
    RCUTILS_LOG_FATAL(
        "[motor_characterization::processConfiguration] YAML file is not a map kind, file: %s",
        yaml_path);
    exit(-1);
  }

  YAML::Node motor_characterization_node = config["motor_characterization"];
  if (!motor_characterization_node.IsMap()) {
    RCUTILS_LOG_FATAL(
        "[motor_characterization::processConfiguration] YAML file does not contain a "
        "'motor_characterization' map");
    exit(-1);
  }

  YAML::Node ros_parameters_node = motor_characterization_node["ros__parameters"];
  if (!ros_parameters_node.IsMap()) {
    RCUTILS_LOG_FATAL(
        "[motor_characterization::processConfiguration] YAML file does not contain a "
        "'ros__parameters' map within the "
        "'motor_characterization' map");
    exit(-1);
  }

  YAML::Node loop_rate_node = ros_parameters_node["loop_rate"];
  if (!loop_rate_node.IsScalar()) {
    RCUTILS_LOG_FATAL(
        "[motor_characterization::processConfiguration] YAML file 'loop_roate' is an not an "
        "scalar");
    exit(-1);
  }

  loop_rate_hz = loop_rate_node.as<double>();

  // YAML::Node topics = rosParameters["topics"];
  // if (!topics.IsSequence() || (topics.size() < 1)) {
  //   RCUTILS_LOG_FATAL(
  //       "[motor_characterization::processConfiguration] YAML file 'topics' is an not an array of
  //       at least one entry");
  //   exit(-1);
  // }

  // for (std::size_t i = 0; i < topics.size(); i++) {
  //   std::string name = getTopicElementValue<std::string>(topics, i, "name");
  //   std::string topic = getTopicElementValue<std::string>(topics, i, "topic");
  //   int priority = getTopicElementValue<int>(topics, i, "priority");
  //   float timeout = getTopicElementValue<float>(topics, i, "timeout");

  //   // rclcpp::Subscription<geometry_msgs::Twist>::SharedPtr
  //   CmdVelSubscriber *subscriber = new CmdVelSubscriber(name);
  //   subscriber->init(topic, priority, timeout);
  //   MultiplexerElement element(name, topic, priority, timeout, subscriber);
  //   multiplexerElements.push_back(element);
  // }
}

void teensyDiagnosticsCallback(const std_msgs::msg::String::SharedPtr msg) {
  // RCUTILS_LOG_INFO("[teensyDiagnosticsCallback] called");
  diagnostic_messages.push_back(msg->data);
}

void teensyStatsCallback(const std_msgs::msg::String::SharedPtr msg) {
  // RCUTILS_LOG_INFO("[teensyStatsCallback] called");
  stats_messages.push_back(msg->data);
}

void wheelOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // RCUTILS_LOG_INFO("[wheelOdometryCallback] called");
  static nav_msgs::msg::Odometry prev_msg;
  last_x = msg->pose.pose.position.x;
  last_y = msg->pose.pose.position.y;
  last_orientation_w = msg->pose.pose.orientation.w;
  last_orientation_x = msg->pose.pose.orientation.x;
  last_orientation_y = msg->pose.pose.orientation.y;
  last_orientation_z = msg->pose.pose.orientation.z;
  tf2::Quaternion q(last_orientation_x, last_orientation_y, last_orientation_z, last_orientation_w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  last_timestamp = msg->header.stamp;
  double delta_time = (last_timestamp - previous_timestamp).seconds();
  if (delta_time > max_skew) {
    max_skew = delta_time;
    new_max_skew = true;
  }

  previous_timestamp = last_timestamp;
  uint8_t skew_index = (uint8_t)(delta_time * 100);
  if (skew_index >= kNumberSkews) {
    skew_index = kNumberSkews - 1;
    RCUTILS_LOG_ERROR(
        "[wheelOdometryCallback] skew_index is greater than or equal to %u, skew_index: %u"
        ", prev header.stamp.secs: %d, prev header.stamp.nanosecs: %u, current header.stamp.secs: "
        "%d, curret header.stamp.nanosecs: %u, delta_time: %3.4f",
        kNumberSkews, skew_index, prev_msg.header.stamp.sec, prev_msg.header.stamp.nanosec,
        msg->header.stamp.sec, msg->header.stamp.nanosec, delta_time);
  }

  skews_by_10ms[skew_index]++;  // increment the skew index

  // RCUTILS_LOG_INFO("[wheelOdometryCallback] timestamp: %3.4f", last_timestamp.seconds());
  // RCUTILS_LOG_INFO("[wheelOdometryCallback] roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
  // RCUTILS_LOG_INFO("[wheelOdometryCallback] x: %f, y: %f, w: %f, x: %f, y: %f, z: %f", last_x,
  //                  last_y, last_orientation_w, last_orientation_x, last_orientation_y,
  //                  last_orientation_z);
  count_wheel_odometry_callbacks++;
  prev_msg = *msg;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  ros_node = rclcpp::Node::make_shared("motor_characterization");

  processConfiguration(argc, argv);

  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  qos.avoid_ros_namespace_conventions(false);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher =
      ros_node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  teensy_diagnostics_subscriber = ros_node->create_subscription<std_msgs::msg::String>(
      "/teensy_diagnostics", qos, teensyDiagnosticsCallback);

  teensy_stats_subscriber = ros_node->create_subscription<std_msgs::msg::String>(
      "/teensy_stats", qos, teensyStatsCallback);

  previous_timestamp = ros_node->now();
  wheel_odometry_subscriber = ros_node->create_subscription<nav_msgs::msg::Odometry>(
      "/wheel_odom", qos, wheelOdometryCallback);

  // rclcpp::Rate loopRate(loopRateHz);
  // while (rclcpp::ok())
  // {
  //     if (!messageQueue.empty())
  //     {
  //         currentMessage = messageQueue.top();
  //         twistPublisher->publish(currentMessage.message);
  //         messageQueue.pop();
  //         lastCommandPoppedTime = std::chrono::steady_clock::now();
  //     }
  //     rclcpp::spin_some(rosNode);
  //     loopRate.sleep();
  // }

  auto timer_callback = []() -> void {
    RCUTILS_LOG_INFO("[timer_callback] Timer triggered");
    for (uint8_t i = 0; i < kNumberSkews; i++) {
      RCUTILS_LOG_INFO("[timer_callback] skew %d: %d", i, skews_by_10ms[i]);
    }
    RCUTILS_LOG_INFO("[timer_callback] count_wheel_odometry_callbacks: %d, max_skew(ms): %3.4f",
                     count_wheel_odometry_callbacks, max_skew * 1000);
    count_wheel_odometry_callbacks = 0;
    if (new_max_skew) {
      for (auto &msg : diagnostic_messages) {
        RCUTILS_LOG_INFO("[timer_callback] diagnostic message: %s", msg.c_str());
      }
      for (auto &msg : stats_messages) {
        RCUTILS_LOG_INFO("[timer_callback] stats message: %s", msg.c_str());
      }
      new_max_skew = false;
      // max_skew = 0.0;
      for (uint8_t i = 0; i < kNumberSkews; i++) {
        skews_by_10ms[i] = 0;
      }
    }

    diagnostic_messages.clear();
    stats_messages.clear();
  };

  auto timer = ros_node->create_wall_timer(std::chrono::seconds(1), timer_callback);
  rclcpp::Rate loop_rate(loop_rate_hz);
  while (rclcpp::ok()) {
    rclcpp::spin_some(ros_node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}