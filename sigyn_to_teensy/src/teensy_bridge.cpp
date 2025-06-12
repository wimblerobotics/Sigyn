#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <fstream>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sigyn_interfaces/srv/teensy_sd_get_dir.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <string>

class TeensyBridge : public rclcpp::Node {
 public:
  TeensyBridge() : Node("teensy_bridge"), serial_fd_(-1) {
    // Initialize serial connection
    if (!initSerial("/dev/teensy_sensor", 115200)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to initialize serial connection");
      return;
    }

    // Create subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&TeensyBridge::cmdVelCallback, this, std::placeholders::_1));

    // Create publishers
    odom_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
    roboclaw_status_pub_ =
        this->create_publisher<std_msgs::msg::String>("roboclaw_status", 10);
    teensy_diagnostics_pub_ =
        this->create_publisher<std_msgs::msg::String>("teensy_diagnostics", 10);

    battery_state_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
        "/main_battery_state", 10);

    // Timer to read from serial
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(10),
                                std::bind(&TeensyBridge::readSerial, this));

    // Create service for SD card directory listing
    sd_getdir_service_ = this->create_service<sigyn_interfaces::srv::TeensySdGetDir>(
        "teensy_sd_getdir",
        std::bind(&TeensyBridge::handleSdGetDirRequest, this, 
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Teensy bridge node started");
  }

  ~TeensyBridge() {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

 private:
  bool initSerial(const std::string& device, int baud_rate) {
    serial_fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial device: %s",
                   device.c_str());
      return false;
    }

    struct termios options;
    tcgetattr(serial_fd_, &options);

    // Set baud rate
    cfsetispeed(&options, B921600);
    cfsetospeed(&options, B921600);

    // Configure serial options
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;

    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    options.c_oflag &= ~OPOST;

    tcsetattr(serial_fd_, TCSANOW, &options);

    return true;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (serial_fd_ < 0) return;

    // Send twist message to Teensy
    std::ostringstream oss;
    oss << "TWIST:" << msg->linear.x << "," << msg->angular.z << "\n";
    std::string message = oss.str();

    ssize_t bytes_written =
        write(serial_fd_, message.c_str(), message.length());
    if (bytes_written < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write to serial");
    }
  }

  void readSerial() {
    if (serial_fd_ < 0) return;

    char buffer[256];
    ssize_t bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1);

    if (bytes_read > 0) {
      buffer[bytes_read] = '\0';
      serial_buffer_ += std::string(buffer);

      // Process complete lines
      size_t pos;
      while ((pos = serial_buffer_.find('\n')) != std::string::npos) {
        std::string line = serial_buffer_.substr(0, pos);
        serial_buffer_.erase(0, pos + 1);
        processSerialMessage(line);
        // RCUTILS_LOG_INFO("Received serial message: %s", line.c_str());
      }
    }
  }

  void processSerialMessage(const std::string& message) {
    size_t colon_pos = message.find(':');
    if (colon_pos == std::string::npos) return;

    std::string type = message.substr(0, colon_pos);
    std::string data = message.substr(colon_pos + 1);

    if (type == "ROBOCLAW") {
      // Publish roboclaw diagnostics
      auto msg = std_msgs::msg::String();
      msg.data = data;
      roboclaw_status_pub_->publish(msg);
    } else if (type == "BATTERY") {
      // Parse and publish battery state
      parseBatteryMessage(data);
    } else if (type == "DIAG") {
      auto msg = std_msgs::msg::String();
      msg.data = data;
      teensy_diagnostics_pub_->publish(msg);
      
      // Check if this is an SD directory response
      if (data.find("SDIR:") == 0) {
        pending_sd_response_ = data;
      }
    } else if (type == "ODOM") {
      parseOdometryMessage(data);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown message type: %s", type.c_str());
    }
  }

  void parseOdometryMessage(const std::string& data) {
    static nav_msgs::msg::Odometry odom_msg;

    // Reset message
    odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Parse key=value pairs
    std::map<std::string, double> values;
    std::istringstream iss(data);
    std::string token;
    while (std::getline(iss, token, ',')) {
      auto eq_pos = token.find('=');
      if (eq_pos != std::string::npos) {
        std::string key = token.substr(0, eq_pos);
        double value = std::stod(token.substr(eq_pos + 1));
        values[key] = value;
      }
    }

    // Fill odom_msg fields
    odom_msg.pose.pose.position.x = values["px"];
    odom_msg.pose.pose.position.y = values["py"];
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = values["ox"];
    odom_msg.pose.pose.orientation.y = values["oy"];
    odom_msg.pose.pose.orientation.z = values["oz"];
    odom_msg.pose.pose.orientation.w = values["ow"];

    odom_msg.pose.covariance[0] = 0.001;
    odom_msg.pose.covariance[7] = 0.001;
    odom_msg.pose.covariance[35] = 0.001;

    odom_msg.twist.twist.linear.x = values["vx"];
    odom_msg.twist.twist.linear.y = values["vy"];
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = values["wz"];

    odom_pub_->publish(odom_msg);
  }

  // Parse battery message and publish BatteryState

  void parseBatteryMessage(const std::string& data) {
    std::istringstream iss(data);
    std::string voltage_str, percentage_str;

    if (std::getline(iss, voltage_str, ',') &&
        std::getline(iss, percentage_str, ',')) {
      auto battery_msg = sensor_msgs::msg::BatteryState();
      battery_msg.header.stamp = this->get_clock()->now();
      battery_msg.header.frame_id = "base_link";

      battery_msg.voltage = std::stof(voltage_str);
      battery_msg.percentage = std::stof(percentage_str);
      battery_msg.current = 0.0;          // Placeholder
      battery_msg.charge = 0.0;           // Placeholder
      battery_msg.capacity = 1.0;         // Placeholder
      battery_msg.design_capacity = 1.0;  // Placeholder
      battery_msg.power_supply_status =
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
      battery_msg.power_supply_health =
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
      battery_msg.power_supply_technology =
          sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
      battery_msg.present = true;

      battery_state_pub_->publish(battery_msg);
    }
  }

  void handleSdGetDirRequest(
      const std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Request> request,
      std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Response> response) {
    
    if (serial_fd_ < 0) {
      response->success = false;
      response->error_message = "Serial connection not available";
      response->directory_listing = "";
      return;
    }

    // Send SDDIR command to Teensy
    std::ostringstream oss;
    oss << "SDDIR:" << request->directory_path << "\n";
    std::string message = oss.str();

    ssize_t bytes_written = write(serial_fd_, message.c_str(), message.length());
    if (bytes_written < 0) {
      response->success = false;
      response->error_message = "Failed to send command to Teensy";
      response->directory_listing = "";
      RCLCPP_ERROR(this->get_logger(), "Failed to write SDDIR command to serial");
      return;
    }

    // Wait for response from Teensy
    // We need to wait for a DIAG message that starts with "SDIR:"
    std::string accumulated_response = "";
    auto start_time = this->get_clock()->now();
    auto timeout = rclcpp::Duration::from_nanoseconds(5000000000); // 5 seconds timeout
    
    bool response_received = false;
    while (!response_received && (this->get_clock()->now() - start_time) < timeout) {
      // Check if we have received a response
      if (!pending_sd_response_.empty()) {
        accumulated_response = pending_sd_response_;
        pending_sd_response_.clear();
        response_received = true;
        break;
      }
      
      // Sleep for a short time to avoid busy waiting
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    if (response_received) {
      response->success = true;
      response->error_message = "";
      response->directory_listing = accumulated_response;
      RCLCPP_INFO(this->get_logger(), "SD directory listing completed successfully");
    } else {
      response->success = false;
      response->error_message = "Timeout waiting for Teensy response";
      response->directory_listing = "";
      RCLCPP_ERROR(this->get_logger(), "Timeout waiting for SD directory response");
    }
  }

  // ROS2 components
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr roboclaw_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr teensy_diagnostics_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr
      battery_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<sigyn_interfaces::srv::TeensySdGetDir>::SharedPtr sd_getdir_service_;

  // Serial communication
  int serial_fd_;
  std::string serial_buffer_;
  std::string pending_sd_response_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeensyBridge>());
  rclcpp::shutdown();
  return 0;
}
