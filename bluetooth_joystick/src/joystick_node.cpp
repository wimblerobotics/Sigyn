#include <fcntl.h>
#include <linux/joystick.h>
#include <rcutils/logging_macros.h>
#include <stdio.h>
#include <unistd.h>

#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "msgs/msg/bluetooth_joystick.hpp"

static const double axis_range_normalizer = 32768.0;

rclcpp::Publisher<msgs::msg::BluetoothJoystick>::SharedPtr
    bluetooth_joystick_publisher;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdvel_publisher;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr gripper_publisher;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twister_publisher;
std::shared_ptr<rclcpp::TimerBase> publish_timer;
std::shared_ptr<rclcpp::TimerBase> cmdvel_publish_timer;
std::shared_ptr<rclcpp::TimerBase> gripper_publish_timer;
double dead_zone = 0.0;
std::string device_name;
int cmdvel_message_rate = 30;
int gripper_message_rate = 600;
int joystick_message_rate = 100;
int gripper_open_value = 1000;
int gripper_close_value = -1000;
std::string cmdvel_twister_topic;
std::shared_ptr<rclcpp::Node> node;
double scale_x;
double scale_z;
bool should_publish_cmdvel = false;
bool should_publish_gripper = false;

/**
 * Reads a joystick event from the joystick device.
 *
 * Returns 0 on success. Otherwise -1 is returned.
 */
int ReadEvent(int fd, struct js_event *event) {
  ssize_t bytes;

  bytes = read(fd, event, sizeof(*event));

  if (bytes == sizeof(*event)) return 0;

  /* Error, could not read full event. */
  return -1;
}

/**
 * Returns the number of axes on the controller or 0 if an error occurs.
 */
size_t GetAxisCount(int fd) {
  __u8 axes;

  if (ioctl(fd, JSIOCGAXES, &axes) == -1) return 0;

  return axes;
}

/**
 * Returns the number of buttons on the controller or 0 if an error occurs.
 */
size_t GetButtonCount(int fd) {
  __u8 buttons;
  if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1) return 0;

  return buttons;
}

/**
 * Current state of an axis.
 */
struct AxisState {
  short x, y;
};

/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
size_t GetAxisState(struct js_event *event, struct AxisState axes[3]) {
  size_t axis = event->number / 2;

  if (axis < 3) {
    if (event->number % 2 == 0)
      axes[axis].x = event->value;
    else
      axes[axis].y = event->value;
  }

  return axis;
}

msgs::msg::BluetoothJoystick message;
bool some_button_changed_state = false;
std::mutex some_button_changed_state_guard;

// Helper to check if a joystick is at zero
bool IsStickZero(int16_t lr, int16_t ud) {
  return lr == 0 && ud == 0;
}

bool ShouldPublishMessages() {
  return some_button_changed_state || !IsStickZero(message.axis0_lr, message.axis0_ud) || !IsStickZero(message.axis1_lr, message.axis1_ud);
}

void PublishCmdvelTimerCallback() {
  static bool was_active = false;
  if (!IsStickZero(message.axis0_lr, message.axis0_ud)) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = (double)message.axis0_ud / axis_range_normalizer;
    twist.angular.z = (double)message.axis0_lr / axis_range_normalizer;
    if (abs(twist.linear.x) < dead_zone) twist.linear.x = 0.0;
    if (abs(twist.angular.z) < dead_zone) twist.angular.z = 0.0;
    twist.linear.x *= scale_x;
    twist.angular.z *= scale_z;
    cmdvel_publisher->publish(twist);
    was_active = true;
  } else if (was_active) {
    // Deadman stick released, send zero message
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    cmdvel_publisher->publish(twist);
    was_active = false;
  }
}

void PublishGripperTimerCallback() {
  if (!IsStickZero(message.axis1_lr, message.axis1_ud)) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = (double)message.axis1_ud / axis_range_normalizer;
    twist.angular.z = (double)message.axis1_lr / axis_range_normalizer;
    gripper_publisher->publish(twist);
  }
}

void PublishJoystickMessages() {
  bluetooth_joystick_publisher->publish(message);
}

void PublishMessagesTimerCallback() {
  // Only publish if a control surface is touched
  if (some_button_changed_state || !IsStickZero(message.axis0_lr, message.axis0_ud) || !IsStickZero(message.axis1_lr, message.axis1_ud)) {
    PublishJoystickMessages();
    const std::lock_guard<std::mutex> lock(some_button_changed_state_guard);
    some_button_changed_state = false;
  }
}

void CaptureJoystickEvent() {
  const char *device;
  int js = -1;
  struct js_event event;
  struct AxisState axes[3] = {0, 0};
  size_t axis;

  device = device_name.c_str();

  while (rclcpp::ok()) {
    if (js == -1) {
      js = open(device, O_RDONLY);

      if (js == -1) {
        RCUTILS_LOG_ERROR("Waiting for joystick device: %s", device);
        rclcpp::sleep_for(std::chrono::seconds(1));
      } else {
        RCUTILS_LOG_INFO("Successfully opened device: %s", device);
      }
    }

    int8_t a = 0;
    int8_t b = 0;
    int8_t l1 = 0;
    int8_t l2 = 0;
    int8_t r1 = 0;
    int8_t r2 = 0;
    int8_t x = 0;
    int8_t y = 0;
    int16_t lr0 = 0;
    int16_t ud0 = 0;
    int16_t lr1 = 0;
    int16_t ud1 = 0;
    int16_t lr2 = 0;
    int16_t ud2 = 0;
    int16_t dpad_lr = 0;
    int16_t dpad_ud = 0;
    while (ReadEvent(js, &event) == 0) {
      message.axis0_lr = lr0;
      message.axis0_ud = ud0;
      message.axis1_lr = lr1;
      message.axis1_ud = ud1;
      message.axis2_lr = lr2;
      message.axis2_ud = ud2;
      message.dpad_lr = dpad_lr;
      message.dpad_ud = dpad_ud;
      message.button_a = a;
      message.button_b = b;
      message.button_x = x;
      message.button_y = y;
      message.button_l1 = l1;
      message.button_l2 = l2;
      message.button_r1 = r1;
      message.button_r2 = r2;
      switch (event.type) {
        case JS_EVENT_BUTTON: {
          const std::lock_guard<std::mutex> lock(some_button_changed_state_guard);
          some_button_changed_state = true;
        }
          switch (event.number) {
            case 0:
              message.button_a = a = event.value ? 1 : 0;
              break;
            case 1:
              message.button_b = b = event.value ? 1 : 0;
              break;
            case 2:
              message.button_x = x = event.value ? 1 : 0;
              break;
            case 3:
              message.button_y = y = event.value ? 1 : 0;
              break;
            case 4:
              message.button_l1 = l1 = event.value ? 1 : 0;
              break;
            case 5:
              message.button_r1 = r1 = event.value ? 1 : 0;
              break;
            case 6:
              message.button_l2 = l2 = event.value ? 1 : 0;
              break;
            case 7:
              message.button_r2 = r2 = event.value ? 1 : 0;
              break;
          }
          break;
        case JS_EVENT_AXIS:
          axis = GetAxisState(&event, axes);
          if (axis < 3) {
            if (axis == 0) {
              message.axis0_lr = lr0 = -axes[axis].x;
              message.axis0_ud = ud0 = axes[axis].y;
            } else if (axis == 1) {
              message.axis1_lr = lr1 = -axes[axis].x;
              message.axis1_ud = ud1 = axes[axis].y;
            } else if (axis == 2) {
              message.axis2_lr = lr2 = -axes[axis].x;
              message.axis2_ud = ud2 = axes[axis].y;
            }
          } else if (axis == 3) {
            message.dpad_lr = dpad_lr = axes[axis].x;
            message.dpad_ud = dpad_ud = axes[axis].y;
          }
          break;
        default:
          /* Ignore init events. */
          break;
      }  // switch (event.type)
    }    // while (ReadEvent(js, &event) == 0)

    js = -1;
  }  // while (rclcpp::ok())

  close(js);
  RCUTILS_LOG_INFO("[bluetooth_joystick_node] device loop shutdown");
}

void TwisterButtonThread() {
  rclcpp::Rate rate(10);
  int last_r1 = 0;
  int last_r2 = 0;
  while (rclcpp::ok()) {
    int r1, r2;
    {
      // Lock mutex to safely read button states
      const std::lock_guard<std::mutex> lock(some_button_changed_state_guard);
      r1 = message.button_r1;
      r2 = message.button_r2;
    }
    // Only send if button_r1 or button_r2 is pressed (edge or level)
    if (r1 && !last_r1) {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = gripper_open_value;
      twister_publisher->publish(twist);
    }
    if (r2 && !last_r2) {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = gripper_close_value;
      twister_publisher->publish(twist);
    }
    last_r1 = r1;
    last_r2 = r2;
    rate.sleep();
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("bluetooth_joystick_node");

  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  qos.avoid_ros_namespace_conventions(false);

  bluetooth_joystick_publisher =
      node->create_publisher<msgs::msg::BluetoothJoystick>("bluetoothJoystick",
                                                           qos);

  std::string cmdvel_topic;
  node->declare_parameter<std::string>("cmdvel_topic",
                                       "!!NO_JOYSTICK_TOPIC_SPECIFIED");
  node->get_parameter("cmdvel_topic", cmdvel_topic);
  RCUTILS_LOG_INFO("[bluetooth_joystick_node] cmd_vel_topic: %s", cmdvel_topic.c_str());
  cmdvel_publisher =
      node->create_publisher<geometry_msgs::msg::Twist>(cmdvel_topic, qos);

  std::string gripper_topic;
  node->declare_parameter<std::string>("gripper_topic", "cmd_vel_gripper");
  node->get_parameter("gripper_topic", gripper_topic);
  RCUTILS_LOG_INFO("[bluetooth_joystick_node] gripper_topic: %s", gripper_topic.c_str());
  gripper_publisher =
      node->create_publisher<geometry_msgs::msg::Twist>(gripper_topic, qos);

  double deadzone_percent;
  node->declare_parameter<double>("deadzone_percent", 5.0);
  node->get_parameter("deadzone_percent", deadzone_percent);
  RCUTILS_LOG_INFO("[bluetooth_joystick_node] deadzone_percent: %f",
                   deadzone_percent);
  dead_zone = deadzone_percent / 100.0;

  node->declare_parameter<std::string>("device_name", "missingDeviceName");
  node->get_parameter("device_name", device_name);
  RCUTILS_LOG_INFO("[bluetooth_joystick_node] device_name: %s",
                   device_name.c_str());

  node->declare_parameter<int>("cmdvel_message_rate", 30);
  node->get_parameter("cmdvel_message_rate", cmdvel_message_rate);
  node->declare_parameter<int>("gripper_message_rate", 600);
  node->get_parameter("gripper_message_rate", gripper_message_rate);
  RCUTILS_LOG_INFO("[bluetooth_joystick_node] cmdvel_message_rate: %d", cmdvel_message_rate);
  RCUTILS_LOG_INFO("[bluetooth_joystick_node] gripper_message_rate: %d", gripper_message_rate);

  node->declare_parameter<int>("joystick_message_rate", 100);
  node->get_parameter("joystick_message_rate", joystick_message_rate);
  RCUTILS_LOG_INFO("[bluetooth_joystick_node] joystick_message_rate: %d", joystick_message_rate);

  node->declare_parameter<double>("scale_x", 1.0);
  node->get_parameter("scale_x", scale_x);
  RCUTILS_LOG_INFO("[bluetooth_joystick_node] scale_x: %f", scale_x);

  node->declare_parameter<double>("scale_z", 1.0);
  node->get_parameter("scale_z", scale_z);
  RCUTILS_LOG_INFO("[bluetooth_joystick_node] scale_z: %f", scale_z);

  node->declare_parameter<int>("gripper_open_value", 1000);
  node->get_parameter("gripper_open_value", gripper_open_value);
  node->declare_parameter<int>("gripper_close_value", -1000);
  node->get_parameter("gripper_close_value", gripper_close_value);
  node->declare_parameter<std::string>("cmdvel_twister_topic", "cmd_vel_testicle_twister");
  node->get_parameter("cmdvel_twister_topic", cmdvel_twister_topic);
  twister_publisher = node->create_publisher<geometry_msgs::msg::Twist>(cmdvel_twister_topic, qos);

  // Start the joystick event thread
  std::thread deviceEventThread(CaptureJoystickEvent);

  // Start the timers for publishing messages
  double cmdvel_period_sec = 1.0 / static_cast<double>(cmdvel_message_rate);
  double gripper_period_sec = 1.0 / static_cast<double>(gripper_message_rate);
  cmdvel_publish_timer = node->create_wall_timer(
      std::chrono::duration<double>(cmdvel_period_sec), PublishCmdvelTimerCallback);
  gripper_publish_timer = node->create_wall_timer(
      std::chrono::duration<double>(gripper_period_sec), PublishGripperTimerCallback);

  // BluetoothJoystick message publisher (unchanged, can be at any rate)
  double joy_period_sec = 1.0 / static_cast<double>(joystick_message_rate);
  publish_timer = node->create_wall_timer(
      std::chrono::duration<double>(joy_period_sec), PublishJoystickMessages);

  std::thread twisterThread(TwisterButtonThread);

  rclcpp::spin(node);
  deviceEventThread.join();
  twisterThread.join();
  rclcpp::shutdown();
  return 0;
}