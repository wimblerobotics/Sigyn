#include <getopt.h>
#include <rcutils/logging_macros.h>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>

// #include <exception>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fstream>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

double last_x = 0.0;
double last_y = 0.0;
double last_orientation_w = 0.0;
double last_orientation_x = 0.0;
double last_orientation_y = 0.0;
double last_orientation_z = 0.0;
rclcpp::Time last_timestamp = rclcpp::Time(0, 0);
double loop_rate_hz;
rclcpp::QoS qos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
rclcpp::Node::SharedPtr ros_node;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odometry_subscriber;

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

void wheelOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  RCUTILS_LOG_INFO("[wheelOdometryCallback] called");
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
  RCUTILS_LOG_INFO("[wheelOdometryCallback] timestamp: %3.4f", last_timestamp.seconds());
  RCUTILS_LOG_INFO("[wheelOdometryCallback] roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
  RCUTILS_LOG_INFO("[wheelOdometryCallback] x: %f, y: %f, w: %f, x: %f, y: %f, z: %f", last_x,
                   last_y, last_orientation_w, last_orientation_x, last_orientation_y,
                   last_orientation_z);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  ros_node = rclcpp::Node::make_shared("motor_characterization");
  RCUTILS_LOG_INFO("[motor_characterization] Hello world");

  processConfiguration(argc, argv);

  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  qos.avoid_ros_namespace_conventions(false);

  RCUTILS_LOG_INFO("[motor_characterization] about to create publisher");
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher =
      ros_node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  RCUTILS_LOG_INFO("[motor_characterization] about to create subscriber");
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

  rclcpp::Rate loop_rate(loop_rate_hz);
  while (rclcpp::ok()) {
    rclcpp::spin_some(ros_node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}