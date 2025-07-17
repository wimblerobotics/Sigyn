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
#include <sensor_msgs/msg/imu.hpp>
#include <sigyn_interfaces/srv/teensy_sd_get_dir.hpp>
#include <sigyn_interfaces/srv/teensy_sd_get_file.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>

// Structure to hold pending service requests
struct PendingServiceRequest {
  std::string request_id;
  std::string service_type; // "get_dir" or "get_file"
  std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Response> dir_response;
  std::shared_ptr<sigyn_interfaces::srv::TeensySdGetFile::Response> file_response;
  rclcpp::Time request_time;
  rclcpp::Time last_activity_time; // Last time we received data for this request
  std::shared_ptr<std::promise<bool>> completion_promise;
  std::string accumulated_content; // For file dump responses
};

class SigynToSensor : public rclcpp::Node {
 public:
  SigynToSensor() : Node("teensy_to_sensor"), serial_fd_(-1), serial_fd2_(-1) {
    // Initialize serial connection for first Teensy (motor control, odometry)
    if (!initSerial("/dev/teensy_sensor", 921600, serial_fd_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to initialize serial connection to /dev/teensy_sensor");
      return;
    }

    // Initialize serial connection for second Teensy (IMU sensors, battery monitoring)
    if (!initSerial("/dev/teensy_sensor2", 921600, serial_fd2_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to initialize serial connection to /dev/teensy_sensor2");
      return;
    }

    // Create callback groups - service will run in separate thread
    service_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&SigynToSensor::cmdVelCallback, this, std::placeholders::_1));

    // Create publishers with appropriate QoS profiles
    
    // High-frequency sensor data - use BEST_EFFORT for performance
    auto sensor_qos = rclcpp::QoS(10)
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile)
        .history(rclcpp::HistoryPolicy::KeepLast);
    
    // Navigation-critical data - use RELIABLE
    auto nav_qos = rclcpp::QoS(10)
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::Volatile)
        .history(rclcpp::HistoryPolicy::KeepLast);
    
    // Status/diagnostic data - use RELIABLE with smaller queue
    auto status_qos = rclcpp::QoS(5)
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::TransientLocal)
        .history(rclcpp::HistoryPolicy::KeepLast);

    // IMU data - high frequency sensor data, use BEST_EFFORT for low latency
    imu0_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu0", sensor_qos);
    imu1_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu1", sensor_qos);
    
    // Odometry - navigation critical, use RELIABLE
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", nav_qos);
    
    // Battery state - status data, use RELIABLE with transient local
    battery_state_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
        "/main_battery_state", status_qos);
    
    // Diagnostic messages - status data
    roboclaw_status_pub_ = this->create_publisher<std_msgs::msg::String>("roboclaw_status", status_qos);
    teensy_diagnostics_pub_ = this->create_publisher<std_msgs::msg::String>("teensy_sensor_diagnostics", status_qos);

    // Create service for SD card directory listing with separate callback group
    sd_getdir_service_ =
        this->create_service<sigyn_interfaces::srv::TeensySdGetDir>(
            "teensy_sensor_sd_getdir",
            std::bind(&SigynToSensor::handleSdGetDirRequest, this,
                      std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(),
            service_callback_group_);

    // Create service for SD card file dump with separate callback group
    sd_getfile_service_ =
        this->create_service<sigyn_interfaces::srv::TeensySdGetFile>(
            "teensy_sensor_sd_getfile",
            std::bind(&SigynToSensor::handleSdGetFileRequest, this,
                      std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(),
            service_callback_group_);

    // Single timer for all serial communication - this ensures thread safety
    // by handling all I/O in one thread
    serial_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&SigynToSensor::serialCommunicationLoop, this));

    RCLCPP_INFO(this->get_logger(), "Teensy to sensor node started - Teensy 1: motor control/odometry, Teensy 2: IMU/battery monitoring");
  }

  ~SigynToSensor() {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
    if (serial_fd2_ >= 0) {
      close(serial_fd2_);
    }
  }

 private:
  bool initSerial(const std::string& device, int baud_rate, int& fd) {
    fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial device: %s",
                   device.c_str());
      return false;
    }

    struct termios options;
    tcgetattr(fd, &options);

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

    tcsetattr(fd, TCSANOW, &options);

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
    if (serial_fd_ < 0 && serial_fd2_ < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                           "Serial communication loop: both serial connections unavailable");
      return;
    }

    // Read incoming data from both Teensy boards
    if (serial_fd_ >= 0) {
      readSerialData(serial_fd_, serial_buffer_, 1);  // Teensy 1
    }
    if (serial_fd2_ >= 0) {
      readSerialData(serial_fd2_, serial_buffer2_, 2);  // Teensy 2
    }

    // Send outgoing messages (currently only to Teensy 1)
    sendQueuedMessages();
  }

  void readSerialData(int fd, std::string& buffer, int teensy_id) {
    char read_buffer[1024];
    ssize_t bytes_read = read(fd, read_buffer, sizeof(read_buffer) - 1);

    if (bytes_read > 0) {
      read_buffer[bytes_read] = '\0';
      buffer += std::string(read_buffer);

      // Process complete lines (handle both \n and \r\n)
      size_t pos;
      while ((pos = buffer.find('\n')) != std::string::npos) {
        std::string line = buffer.substr(0, pos);
        
        // Remove trailing \r if present
        if (!line.empty() && line.back() == '\r') {
          line.pop_back();
        }
        
        buffer.erase(0, pos + 1);
        
        if (!line.empty()) {
          processIncomingMessage(line, teensy_id);
        }
      }
    }
  }

  void sendQueuedMessages() {
    std::lock_guard<std::mutex> lock(outgoing_queue_mutex_);
    
    // if (!outgoing_message_queue_.empty()) {
    //   RCLCPP_INFO(this->get_logger(), "Processing message queue, size: %zu", 
    //               outgoing_message_queue_.size());
    // }
    
    while (!outgoing_message_queue_.empty()) {
      const std::string& message = outgoing_message_queue_.front();
      
      // RCLCPP_INFO(this->get_logger(), "About to send serial message: '%s'", 
      //              message.c_str());
      
      ssize_t bytes_written = write(serial_fd_, message.c_str(), message.length());
      if (bytes_written < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to serial");
        break;
      } else {
        // Force immediate transmission
        fsync(serial_fd_);
        // RCLCPP_INFO(this->get_logger(), "Successfully sent %zd bytes: '%s'", 
        //              bytes_written, message.c_str());
      }
      
      outgoing_message_queue_.pop();
    }
  }

  void processIncomingMessage(const std::string& message, int teensy_id) {
    // RCLCPP_INFO(this->get_logger(), "Processing message from Teensy %d: %s", teensy_id, message.c_str());
    
    size_t colon_pos = message.find(':');
    if (colon_pos == std::string::npos) {
      RCLCPP_WARN(this->get_logger(), "Invalid serial message format from Teensy %d: %s",
                  teensy_id, message.c_str());
      return;
    }

    std::string type = message.substr(0, colon_pos);
    std::string data = message.substr(colon_pos + 1);

    // Handle messages from Teensy 1 (motor control board)
    if (teensy_id == 1) {
      if (type == "ROBOCLAW") {
        publishRoboClawStatus(data);
      } else if (type == "DIAG") {
        publishDiagnostics(data);
      } else if (type == "SDIR") {
        handleSdirResponse(data);
      } else if (type == "SDLINE") {
        handleSdlineResponse(data);
      } else if (type == "SDEOF") {
        handleSdeofResponse(data);
      } else if (type == "ODOM") {
        publishOdometry(data);
      } else {
        RCLCPP_WARN(this->get_logger(), "Unknown message type from Teensy 1: %s", type.c_str());
      }
    }
    // Handle messages from Teensy 2 (IMU board and battery monitoring)
    else if (teensy_id == 2) {
      if (type == "DIAG") {
        // Check if this is an IMU data message
        if (data.substr(0, 9) == "IMU_DATA:") {
          publishImuData(data.substr(9)); // Remove "IMU_DATA:" prefix
        }
        // Check if this is a battery data message
        else if (data.substr(0, 8) == "BATTERY:") {
          publishBatteryState(data.substr(8)); // Remove "BATTERY:" prefix
        } else {
          // Regular diagnostic message
          publishTeensy2Diagnostics(data);
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Unknown message type from Teensy 2: %s", type.c_str());
      }
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
      // RCLCPP_INFO(this->get_logger(), "Found SDIR response in DIAG message");
      // Extract the SDIR data (everything after "SDIR:")
      std::string sdir_data = data.substr(5);
      handleSdirResponse(sdir_data);
    }
    // Check if this diagnostic message contains an SDLINE response
    else if (data.substr(0, 7) == "SDLINE:") {
      // RCLCPP_INFO(this->get_logger(), "Found SDLINE response in DIAG message");
      // Extract the SDLINE data (everything after "SDLINE:")
      std::string sdline_data = data.substr(7);
      handleSdlineResponse(sdline_data);
    }
    // Check if this diagnostic message contains an SDEOF response
    else if (data.substr(0, 6) == "SDEOF:") {
      // RCLCPP_INFO(this->get_logger(), "Found SDEOF response in DIAG message");
      // Extract the SDEOF data (everything after "SDEOF:")
      std::string sdeof_data = data.substr(6);
      handleSdeofResponse(sdeof_data);
    }
  }

  void publishTeensy2Diagnostics(const std::string& data) {
    auto msg = std_msgs::msg::String();
    msg.data = "Teensy2: " + data;
    teensy_diagnostics_pub_->publish(msg);
  }

  void publishImuData(const std::string& data) {
    // Parse IMU data: sensor_id,qx,qy,qz,qw,gx,gy,gz,ax,ay,az,yaw,roll,pitch,read_time
    std::istringstream iss(data);
    std::string token;
    std::vector<std::string> tokens;
    
    while (std::getline(iss, token, ',')) {
      tokens.push_back(token);
    }
    
    if (tokens.size() < 15) {
      RCLCPP_WARN(this->get_logger(), "Invalid IMU data format: expected 15 fields, got %zu", tokens.size());
      return;
    }
    
    try {
      int sensor_id = std::stoi(tokens[0]);
      
      // Create IMU message
      auto imu_msg = sensor_msgs::msg::Imu();
      imu_msg.header.stamp = this->get_clock()->now();
      
      if (sensor_id == 0) {
        imu_msg.header.frame_id = "imu0_link";
      } else if (sensor_id == 1) {
        imu_msg.header.frame_id = "imu1_link";
      } else {
        RCLCPP_WARN(this->get_logger(), "Invalid IMU sensor ID: %d", sensor_id);
        return;
      }
      
      // Parse quaternion (x, y, z, w)
      imu_msg.orientation.x = std::stod(tokens[1]);
      imu_msg.orientation.y = std::stod(tokens[2]);
      imu_msg.orientation.z = std::stod(tokens[3]);
      imu_msg.orientation.w = std::stod(tokens[4]);
      
      // Parse angular velocity (rad/s)
      imu_msg.angular_velocity.x = std::stod(tokens[5]);
      imu_msg.angular_velocity.y = std::stod(tokens[6]);
      imu_msg.angular_velocity.z = std::stod(tokens[7]);
      
      // Parse linear acceleration (m/s²)
      imu_msg.linear_acceleration.x = std::stod(tokens[8]);
      imu_msg.linear_acceleration.y = std::stod(tokens[9]);
      imu_msg.linear_acceleration.z = std::stod(tokens[10]);
      
      // Set covariance matrices based on BNO055 typical values
      // Orientation covariance (quaternion is quite accurate)
      std::array<double, 9> orientation_cov = {
        0.001, 0.0, 0.0,     // Small variance in orientation
        0.0, 0.001, 0.0,
        0.0, 0.0, 0.001
      };
      std::copy(orientation_cov.begin(), orientation_cov.end(), imu_msg.orientation_covariance.begin());
      
      // Angular velocity covariance (gyroscope noise)
      std::array<double, 9> angular_vel_cov = {
        0.0001, 0.0, 0.0,    // BNO055 gyro noise ~0.01 rad/s
        0.0, 0.0001, 0.0,
        0.0, 0.0, 0.0001
      };
      std::copy(angular_vel_cov.begin(), angular_vel_cov.end(), imu_msg.angular_velocity_covariance.begin());
      
      // Linear acceleration covariance (accelerometer noise)
      std::array<double, 9> linear_acc_cov = {
        0.01, 0.0, 0.0,      // BNO055 accel noise ~0.1 m/s²
        0.0, 0.01, 0.0,
        0.0, 0.0, 0.01
      };
      std::copy(linear_acc_cov.begin(), linear_acc_cov.end(), imu_msg.linear_acceleration_covariance.begin());
      
      // Publish to appropriate topic
      if (sensor_id == 0) {
        imu0_pub_->publish(imu_msg);
      } else if (sensor_id == 1) {
        imu1_pub_->publish(imu_msg);
      }
      
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Failed to parse IMU data: %s", e.what());
    }
  }

  void publishBatteryState(const std::string& data) {
    // Parse new format: "device_id,voltage_V,percentage_%,current_A"
    // Example: "0,40.07V,80.72%,1.52A"
    std::istringstream iss(data);
    std::string device_id_str, voltage_str, percentage_str, current_str;

    if (std::getline(iss, device_id_str, ',') &&
        std::getline(iss, voltage_str, ',') &&
        std::getline(iss, percentage_str, ',') &&
        std::getline(iss, current_str, ',')) {
      
      auto battery_msg = sensor_msgs::msg::BatteryState();
      battery_msg.header.stamp = this->get_clock()->now();
      battery_msg.header.frame_id = "base_link";

      try {
        // Parse device ID
        int device_id = std::stoi(device_id_str);
        
        // Parse voltage (remove 'V' suffix)
        std::string voltage_value = voltage_str;
        if (!voltage_value.empty() && voltage_value.back() == 'V') {
          voltage_value.pop_back();
        }
        battery_msg.voltage = std::stof(voltage_value);
        
        // Parse percentage (remove '%' suffix and convert to 0-1 range)
        std::string percentage_value = percentage_str;
        if (!percentage_value.empty() && percentage_value.back() == '%') {
          percentage_value.pop_back();
        }
        float percentage_raw = std::stof(percentage_value);
        battery_msg.percentage = percentage_raw / 100.0f; // Convert to 0-1 range
        
        // Parse current (remove 'A' suffix)
        std::string current_value = current_str;
        if (!current_value.empty() && current_value.back() == 'A') {
          current_value.pop_back();
        }
        battery_msg.current = std::stof(current_value);
        
        // Set battery properties based on device ID
        if (device_id == 0) {
          // 36V LIPO battery (10-cell, 30Ah)
          battery_msg.design_capacity = 30.0f;  // 30Ah design capacity
          battery_msg.capacity = 30.0f;         // Assume full capacity for now (would need battery health monitoring)
          battery_msg.charge = battery_msg.capacity * battery_msg.percentage; // Current charge in Ah
          
          // For LIPO battery, current IS available from Teensy monitoring
          // Don't overwrite the parsed current value
          
          // Determine power supply status based on current flow
          if (battery_msg.current > 0.1f) {
            battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
          } else if (battery_msg.current < -0.1f) {
            battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
          } else {
            battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
          }
          
          // Determine battery health based on voltage
          float cell_voltage = battery_msg.voltage / 10.0f;  // 10-cell battery
          if (cell_voltage > 4.0f) {
            battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
          } else if (cell_voltage < 3.0f) {
            battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
          } else if (cell_voltage < 3.2f) {
            battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_COLD; // Using as low voltage indicator
          } else {
            battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
          }
          
          battery_msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
        } else {
          // Power supply monitoring (future expansion)
          battery_msg.design_capacity = 0.0f;  // Power supplies don't have capacity
          battery_msg.capacity = 0.0f;
          battery_msg.charge = 0.0f;
          battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
          battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
          battery_msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
        }
        
        battery_msg.present = true;
        
        // Set location based on device ID
        if (device_id == 0) {
          battery_msg.location = "main_battery";
        } else {
          battery_msg.location = "power_supply_" + std::to_string(device_id);
        }
        
        // Add device ID to the serial number for identification
        battery_msg.serial_number = "SIGYN_BAT_" + std::to_string(device_id);

        battery_state_pub_->publish(battery_msg);
        
        // Log battery status for debugging
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                             "Battery %d: %.2fV (%.1f%%), %.2fA, Charge: %.1fAh/%.1fAh, Status: %d, Health: %d",
                             device_id, battery_msg.voltage, battery_msg.percentage * 100.0f, 
                             battery_msg.current, battery_msg.charge, battery_msg.capacity,
                             battery_msg.power_supply_status, battery_msg.power_supply_health);
        
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse battery data '%s': %s", data.c_str(), e.what());
        return;
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid battery data format '%s', expected 'device_id,voltage_V,percentage_%%,current_A'", data.c_str());
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
    // RCLCPP_INFO(this->get_logger(), "Received SDIR response: %s", data.c_str());
    
    std::lock_guard<std::mutex> lock(service_mutex_);
    
    // Check if we have a pending service request for directory listing
    if (!pending_service_requests_.empty()) {
      auto request = pending_service_requests_.front();
      
      if (request.service_type == "get_dir") {
        pending_service_requests_.pop();
        
        // Set the response data - the data portion already contains the file listing
        request.dir_response->success = true;
        request.dir_response->directory_listing = data;  // This is the tab-separated file list
        request.dir_response->error_message = "";
        
        // Signal completion using the promise
        if (request.completion_promise) {
          request.completion_promise->set_value(true);
        }
        
        // RCLCPP_INFO(this->get_logger(), "Directory service request completed with %zu characters", 
        //             data.length());
      } else {
        RCLCPP_WARN(this->get_logger(), "Received SDIR response but pending request is not get_dir type");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Received SDIR response but no pending request");
    }
  }

  void handleSdlineResponse(const std::string& data) {
    // RCLCPP_INFO(this->get_logger(), "Received SDLINE response: %s", data.c_str());
    
    std::lock_guard<std::mutex> lock(service_mutex_);
    
    // Check if we have a pending file dump request
    if (!pending_service_requests_.empty()) {
      auto& request = pending_service_requests_.front();
      
      if (request.service_type == "get_file") {
        // Update last activity time to reset timeout
        request.last_activity_time = this->get_clock()->now();
        
        // Accumulate line content (data should already have \r removed by Teensy)
        if (!request.accumulated_content.empty()) {
          request.accumulated_content += "\n";
        }
        request.accumulated_content += data;
        
        // RCLCPP_INFO(this->get_logger(), "Accumulated %zu characters so far", 
        //             request.accumulated_content.length());
      } else {
        RCLCPP_WARN(this->get_logger(), "Received SDLINE response but pending request is not get_file type");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Received SDLINE response but no pending request");
    }
  }

  void handleSdeofResponse(const std::string& data) {
    // RCLCPP_INFO(this->get_logger(), "Received SDEOF response, file dump complete");
    (void)data; // Unused parameter
    
    std::lock_guard<std::mutex> lock(service_mutex_);
    
    // Check if we have a pending file dump request
    if (!pending_service_requests_.empty()) {
      auto request = pending_service_requests_.front();
      
      if (request.service_type == "get_file") {
        pending_service_requests_.pop();
        
        // Set the response data with accumulated content
        request.file_response->success = true;
        request.file_response->file_contents = request.accumulated_content;
        request.file_response->error_message = "";
        
        // Signal completion using the promise
        if (request.completion_promise) {
          request.completion_promise->set_value(true);
        }
        
        // RCLCPP_INFO(this->get_logger(), "File service request completed with %zu characters", 
        //             request.accumulated_content.length());
      } else {
        RCLCPP_WARN(this->get_logger(), "Received SDEOF response but pending request is not get_file type");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Received SDEOF response but no pending request");
    }
  }

  void handleSdGetDirRequest(
      const std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Request> request,
      std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Response> response) {
    
    // RCLCPP_INFO(this->get_logger(), "=== SERVICE HANDLER CALLED ===");
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
        "get_dir",
        response,
        nullptr,
        this->get_clock()->now(),
        this->get_clock()->now(),
        completion_promise,
        ""
      });
    }
    
    // Queue the serial message
    {
      std::lock_guard<std::mutex> lock(outgoing_queue_mutex_);
      outgoing_message_queue_.push("SDDIR:\n");
      // RCLCPP_INFO(this->get_logger(), "SDDIR message queued, queue size now: %zu", 
      //             outgoing_message_queue_.size());
    }
    
    // RCLCPP_INFO(this->get_logger(), "SDDIR request queued, waiting for response...");
    
    // Wait for completion with timeout
    auto status = completion_future.wait_for(std::chrono::seconds(10));
    
    if (status == std::future_status::timeout) {
      // Handle timeout
      std::lock_guard<std::mutex> lock(service_mutex_);
      
      // Remove the request from the queue if still there
      std::queue<PendingServiceRequest> temp_queue;
      while (!pending_service_requests_.empty()) {
        auto req = pending_service_requests_.front();
        if (req.dir_response != response) {
          temp_queue.push(req);
        }
        pending_service_requests_.pop();
      }
      pending_service_requests_ = temp_queue;
      
      response->success = false;
      response->directory_listing = "";
      response->error_message = "Timeout waiting for response from Teensy sensor";
      
      // RCLCPP_WARN(this->get_logger(), "SDIR service request timed out");
    } else {
      // RCLCPP_INFO(this->get_logger(), "SDIR service request completed successfully");
    }
  }

  void handleSdGetFileRequest(
      const std::shared_ptr<sigyn_interfaces::srv::TeensySdGetFile::Request> request,
      std::shared_ptr<sigyn_interfaces::srv::TeensySdGetFile::Response> response) {
    
    // RCLCPP_INFO(this->get_logger(), "=== FILE DUMP SERVICE HANDLER CALLED ===");
    // RCLCPP_INFO(this->get_logger(), "Requested filename: %s", request->filename.c_str());
    
    if (serial_fd_ < 0) {
      response->success = false;
      response->error_message = "Serial connection not available";
      response->file_contents = "";
      return;
    }

    if (request->filename.empty()) {
      response->success = false;
      response->error_message = "Filename cannot be empty";
      response->file_contents = "";
      return;
    }

    // Create a promise for async completion
    auto completion_promise = std::make_shared<std::promise<bool>>();
    auto completion_future = completion_promise->get_future();

    // Store the service request for async completion
    {
      std::lock_guard<std::mutex> lock(service_mutex_);
      pending_service_requests_.push({
        "sdfile_request",
        "get_file",
        nullptr,
        response,
        this->get_clock()->now(),
        this->get_clock()->now(),
        completion_promise,
        ""
      });
    }
    
    // Queue the serial message
    {
      std::lock_guard<std::mutex> lock(outgoing_queue_mutex_);
      std::string message = "SDFILE:" + request->filename + "\n";
      outgoing_message_queue_.push(message);
      // RCLCPP_INFO(this->get_logger(), "SDFILE message queued: %s", message.c_str());
    }
    
    // RCLCPP_INFO(this->get_logger(), "SDFILE request queued, waiting for response...");
    
    // Wait for completion with timeout (longer timeout for file dumps)
    auto status = completion_future.wait_for(std::chrono::seconds(120));
    
    if (status == std::future_status::timeout) {
      // Handle timeout
      std::lock_guard<std::mutex> lock(service_mutex_);
      
      // Remove the request from the queue if still there
      std::queue<PendingServiceRequest> temp_queue;
      while (!pending_service_requests_.empty()) {
        auto req = pending_service_requests_.front();
        if (req.file_response != response) {
          temp_queue.push(req);
        }
        pending_service_requests_.pop();
      }
      pending_service_requests_ = temp_queue;
      
      response->success = false;
      response->file_contents = "";
      response->error_message = "Timeout waiting for response from Teensy sensor";
      
      // RCLCPP_WARN(this->get_logger(), "SDFILE service request timed out");
    } else {
      // RCLCPP_INFO(this->get_logger(), "SDFILE service request completed successfully");
    }
  }

  // ROS2 components
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr roboclaw_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr teensy_diagnostics_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
  
  // IMU publishers for BNO055 sensors
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu0_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu1_pub_;
  
  rclcpp::Service<sigyn_interfaces::srv::TeensySdGetDir>::SharedPtr sd_getdir_service_;
  rclcpp::Service<sigyn_interfaces::srv::TeensySdGetFile>::SharedPtr sd_getfile_service_;
  
  // Single timer for all serial communication
  rclcpp::TimerBase::SharedPtr serial_timer_;

  // Serial communication for first Teensy (motor control)
  int serial_fd_;
  std::string serial_buffer_;
  
  // Serial communication for second Teensy (IMU sensors)
  int serial_fd2_;
  std::string serial_buffer2_;
  
  // Thread-safe message queue
  std::queue<std::string> outgoing_message_queue_;
  std::mutex outgoing_queue_mutex_;
  
  // Service request handling
  std::queue<PendingServiceRequest> pending_service_requests_;
  std::mutex service_mutex_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<SigynToSensor>();
  
  // Use MultiThreadedExecutor to allow service handlers to run on separate threads
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  RCLCPP_INFO(node->get_logger(), "Starting multi-threaded executor...");
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
