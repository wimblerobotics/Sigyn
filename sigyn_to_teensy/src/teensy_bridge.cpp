#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <future>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <map>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sigyn_interfaces/srv/teensy_sd_get_dir.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>

// Structure to hold pending service requests
struct PendingServiceRequest {
  std::string request_id;
  std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Response> response;
  rclcpp::Time request_time;
  std::shared_ptr<std::promise<bool>> completion_promise;
};

class TeensyBridge : public rclcpp::Node {
 public:
  TeensyBridge() : Node("teensy_bridge2"), serial_fd_(-1) {
    // Initialize serial connection
    if (!initSerial("/dev/teensy_sensor", 921600)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to initialize serial connection");
      return;
    }

    // Create callback groups - service will run in separate thread
    service_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

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

    // Create service for SD card directory listing with separate callback group
    sd_getdir_service_ =
        this->create_service<sigyn_interfaces::srv::TeensySdGetDir>(
            "teensy_sd_getdir",
            std::bind(&TeensyBridge::handleSdGetDirRequest, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            service_callback_group_);

    // Single timer for all serial communication - this ensures thread safety
    // by handling all I/O in one thread
    serial_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&TeensyBridge::serialCommunicationLoop, this));

    RCLCPP_INFO(this->get_logger(), "Teensy bridge2 node started");
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
    speed_t speed;
    switch (baud_rate) {
      case 921600:
        speed = B921600;
        break;
      case 115200:
        speed = B115200;
        break;
      default:
        speed = B921600;
        break;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

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

    // Set timeouts
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    tcsetattr(serial_fd_, TCSANOW, &options);

    return true;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Queue the twist message - all serial communication happens in the main loop
    std::ostringstream oss;
    oss << "TWIST:" << msg->linear.x << "," << msg->angular.z << "\n";
    
    std::lock_guard<std::mutex> lock(outgoing_queue_mutex_);
    outgoing_message_queue_.push(oss.str());
  }

  // Main serial communication loop - handles all I/O in single thread
  void serialCommunicationLoop() {
    if (serial_fd_ < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                           "Serial communication loop: serial_fd_ < 0");
      return;
    }

    // Read incoming data
    readSerialData();

    // Send outgoing messages
    sendQueuedMessages();
  }

  void readSerialData() {
    char buffer[1024];
    ssize_t bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1);

    if (bytes_read > 0) {
      buffer[bytes_read] = '\0';
      serial_buffer_ += std::string(buffer);

      // Process complete lines (handle both \n and \r\n)
      size_t pos;
      while ((pos = serial_buffer_.find('\n')) != std::string::npos) {
        std::string line = serial_buffer_.substr(0, pos);
        
        // Remove trailing \r if present
        if (!line.empty() && line.back() == '\r') {
          line.pop_back();
        }
        
        serial_buffer_.erase(0, pos + 1);
        
        if (!line.empty()) {
          processIncomingMessage(line);
        }
      }
    }
  }

  void sendQueuedMessages() {
    std::lock_guard<std::mutex> lock(outgoing_queue_mutex_);
    
    if (!outgoing_message_queue_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Processing message queue, size: %zu", 
                  outgoing_message_queue_.size());
    }
    
    while (!outgoing_message_queue_.empty()) {
      const std::string& message = outgoing_message_queue_.front();
      
      RCLCPP_INFO(this->get_logger(), "About to send serial message: '%s'", 
                   message.c_str());
      
      ssize_t bytes_written = write(serial_fd_, message.c_str(), message.length());
      if (bytes_written < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to serial");
        break;
      } else {
        // Force immediate transmission
        fsync(serial_fd_);
        RCLCPP_INFO(this->get_logger(), "Successfully sent %zd bytes: '%s'", 
                     bytes_written, message.c_str());
      }
      
      outgoing_message_queue_.pop();
    }
  }

  void processIncomingMessage(const std::string& message) {
    RCLCPP_INFO(this->get_logger(), "Processing message: %s", message.c_str());
    
    size_t colon_pos = message.find(':');
    if (colon_pos == std::string::npos) {
      RCLCPP_WARN(this->get_logger(), "Invalid serial message format: %s",
                  message.c_str());
      return;
    }

    std::string type = message.substr(0, colon_pos);
    std::string data = message.substr(colon_pos + 1);

    if (type == "ROBOCLAW") {
      publishRoboClawStatus(data);
    } else if (type == "BATTERY") {
      publishBatteryState(data);
    } else if (type == "DIAG") {
      publishDiagnostics(data);
    } else if (type == "SDIR") {
      handleSdirResponse(data);
    } else if (type == "ODOM") {
      publishOdometry(data);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown message type: %s", type.c_str());
    }
  }

  void publishRoboClawStatus(const std::string& data) {
    auto msg = std_msgs::msg::String();
    msg.data = data;
    roboclaw_status_pub_->publish(msg);
  }

  void publishDiagnostics(const std::string& data) {
    auto msg = std_msgs::msg::String();
    msg.data = data;
    teensy_diagnostics_pub_->publish(msg);
    
    // Check if this diagnostic message contains an SDIR response
    if (data.substr(0, 5) == "SDIR:") {
      RCLCPP_INFO(this->get_logger(), "Found SDIR response in DIAG message");
      // Extract the SDIR data (everything after "SDIR:")
      std::string sdir_data = data.substr(5);
      handleSdirResponse(sdir_data);
    }
  }

  void publishBatteryState(const std::string& data) {
    std::istringstream iss(data);
    std::string voltage_str, percentage_str;

    if (std::getline(iss, voltage_str, ',') &&
        std::getline(iss, percentage_str, ',')) {
      auto battery_msg = sensor_msgs::msg::BatteryState();
      battery_msg.header.stamp = this->get_clock()->now();
      battery_msg.header.frame_id = "base_link";

      try {
        battery_msg.voltage = std::stof(voltage_str);
        battery_msg.percentage = std::stof(percentage_str);
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse battery data: %s", e.what());
        return;
      }

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

  void publishOdometry(const std::string& data) {
    auto odom_msg = nav_msgs::msg::Odometry();
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
        try {
          double value = std::stod(token.substr(eq_pos + 1));
          values[key] = value;
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), "Failed to parse odom value %s: %s", 
                      token.c_str(), e.what());
        }
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

  void handleSdirResponse(const std::string& data) {
    RCLCPP_INFO(this->get_logger(), "Received SDIR response: %s", data.c_str());
    
    std::lock_guard<std::mutex> lock(service_mutex_);
    
    // Check if we have a pending service request
    if (!pending_service_requests_.empty()) {
      auto request = pending_service_requests_.front();
      pending_service_requests_.pop();
      
      // Set the response data - the data portion already contains the file listing
      request.response->success = true;
      request.response->directory_listing = data;  // This is the tab-separated file list
      request.response->error_message = "";
      
      // Signal completion using the promise
      if (request.completion_promise) {
        request.completion_promise->set_value(true);
      }
      
      RCLCPP_INFO(this->get_logger(), "Service request completed with %zu characters", 
                  data.length());
    } else {
      RCLCPP_WARN(this->get_logger(), "Received SDIR response but no pending request");
    }
  }

  void handleSdGetDirRequest(
      const std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Request> request,
      std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Response> response) {
    
    RCLCPP_INFO(this->get_logger(), "=== SERVICE HANDLER CALLED ===");
    (void)request;  // Unused parameter
    
    if (serial_fd_ < 0) {
      response->success = false;
      response->error_message = "Serial connection not available";
      response->directory_listing = "";
      return;
    }

    // Create a promise for async completion
    auto completion_promise = std::make_shared<std::promise<bool>>();
    auto completion_future = completion_promise->get_future();

    // Store the service request for async completion
    {
      std::lock_guard<std::mutex> lock(service_mutex_);
      pending_service_requests_.push({
        "sdir_request",
        response,
        this->get_clock()->now(),
        completion_promise
      });
    }
    
    // Queue the serial message
    {
      std::lock_guard<std::mutex> lock(outgoing_queue_mutex_);
      outgoing_message_queue_.push("SDDIR:\n");
      RCLCPP_INFO(this->get_logger(), "SDDIR message queued, queue size now: %zu", 
                  outgoing_message_queue_.size());
    }
    
    RCLCPP_INFO(this->get_logger(), "SDDIR request queued, waiting for response...");
    
    // Wait for completion with timeout
    auto status = completion_future.wait_for(std::chrono::seconds(10));
    
    if (status == std::future_status::timeout) {
      // Handle timeout
      std::lock_guard<std::mutex> lock(service_mutex_);
      
      // Remove the request from the queue if still there
      std::queue<PendingServiceRequest> temp_queue;
      while (!pending_service_requests_.empty()) {
        auto req = pending_service_requests_.front();
        if (req.response != response) {
          temp_queue.push(req);
        }
        pending_service_requests_.pop();
      }
      pending_service_requests_ = temp_queue;
      
      response->success = false;
      response->directory_listing = "";
      response->error_message = "Timeout waiting for response from Teensy";
      
      RCLCPP_WARN(this->get_logger(), "SDIR service request timed out");
    } else {
      RCLCPP_INFO(this->get_logger(), "SDIR service request completed successfully");
    }
  }

  // ROS2 components
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr roboclaw_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr teensy_diagnostics_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
  rclcpp::Service<sigyn_interfaces::srv::TeensySdGetDir>::SharedPtr sd_getdir_service_;
  
  // Single timer for all serial communication
  rclcpp::TimerBase::SharedPtr serial_timer_;

  // Serial communication
  int serial_fd_;
  std::string serial_buffer_;
  
  // Thread-safe message queue
  std::queue<std::string> outgoing_message_queue_;
  std::mutex outgoing_queue_mutex_;
  
  // Service request handling
  std::queue<PendingServiceRequest> pending_service_requests_;
  std::mutex service_mutex_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<TeensyBridge>();
  
  // Use MultiThreadedExecutor to allow service handlers to run on separate threads
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  RCLCPP_INFO(node->get_logger(), "Starting multi-threaded executor...");
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
