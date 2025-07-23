/**
 * @file message_parser.h
 * @brief Efficient message parsing for TeensyV2 communication protocol
 * 
 * Provides high-performance parsing of structured messages from TeensyV2
 * embedded system with validation, error recovery, and ROS2 integration.
 * 
 * Message Format: TYPE:key1=val1,key2=val2,...
 * 
 * Supported message types:
 * - BATT: Battery status and measurements
 * - PERF: Performance metrics and timing
 * - SAFETY: Safety status and E-stop conditions
 * - ESTOP: E-stop notifications
 * - DIAG: Diagnostic and error messages
 * - CONFIG: Configuration responses
 * 
 * @author Sigyn Robotics
 * @date 2025
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace sigyn_to_sensor_v2 {

/**
 * @brief Message type enumeration for efficient parsing dispatch.
 */
enum class MessageType {
  BATTERY,      ///< Battery status and measurements
  PERFORMANCE,  ///< Performance metrics and timing
  SAFETY,       ///< Safety status and conditions
  IMU,          ///< IMU sensor data (orientation, gyro, accel)
  TEMPERATURE,  ///< Temperature sensor data (individual TEMP and array TEMPERATURE)
  VL53L0X,      ///< VL53L0X time-of-flight sensor data
  ROBOCLAW,     ///< RoboClaw motor controller data
  ODOM,         ///< Odometry data (position, orientation, velocity)
  ESTOP,        ///< E-stop notifications
  DIAGNOSTIC,   ///< Diagnostic and error messages
  CONFIG,       ///< Configuration responses
  INIT,         ///< Initialization messages
  CRITICAL,     ///< Critical system messages
  UNKNOWN       ///< Unrecognized message type
};

/**
 * @brief Parsed key-value pairs from message content.
 */
using MessageData = std::unordered_map<std::string, std::string>;

/**
 * @brief Callback function type for message handling.
 */
using MessageCallback = std::function<void(const MessageData&, rclcpp::Time timestamp)>;

/**
 * @brief Battery data structure parsed from BATT messages.
 */
struct BatteryData {
  int id = 0;                           ///< Battery/sensor ID
  double voltage = 0.0;                 ///< Voltage in volts
  double current = 0.0;                 ///< Current in amperes (+ discharge, - charge)
  double power = 0.0;                   ///< Power in watts
  double percentage = 0.0;              ///< Charge percentage (0.0-1.0)
  std::string state = "UNKNOWN";        ///< Battery state string
  std::string sensors = "";             ///< Available sensor types
  std::string location = "";            ///< Battery location/description
  bool valid = false;                   ///< Data validity flag
};

/**
 * @brief Performance data structure parsed from PERF messages.
 */
struct PerformanceData {
  double loop_frequency = 0.0;          ///< Current loop frequency (Hz)
  double execution_time = 0.0;          ///< Execution time (ms)
  int violation_count = 0;              ///< Number of performance violations
  int module_count = 0;                 ///< Number of active modules
  double avg_frequency = 0.0;           ///< Average frequency over time
  double max_execution_time = 0.0;      ///< Maximum execution time observed
  bool valid = false;                   ///< Data validity flag
};

/**
 * @brief IMU data structure parsed from IMU messages.
 */
struct IMUData {
  int sensor_id = 0;                    ///< Sensor ID (0 or 1 for dual sensors)
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;  ///< Quaternion orientation
  double gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0; ///< Angular velocity (rad/s)
  double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0; ///< Linear acceleration (m/s²)
  uint8_t calibration_status = 0;       ///< Calibration status byte
  uint8_t system_status = 0;            ///< System status byte
  uint8_t system_error = 0;             ///< System error byte
  uint64_t timestamp = 0;               ///< Sensor timestamp (ms)
  bool valid = false;                   ///< Data validity flag
};

/**
 * @brief Safety data structure parsed from SAFETY messages.
 */
struct SafetyData {
  std::string state = "UNKNOWN";        ///< Safety state (NORMAL, WARNING, ESTOP, etc.)
  bool hardware_estop = false;          ///< Hardware E-stop status
  bool inter_board_safety = false;     ///< Inter-board safety signal status
  bool active_conditions = false;      ///< Any active E-stop conditions
  std::vector<std::string> sources;    ///< List of active E-stop sources
  bool valid = false;                   ///< Data validity flag
};

/**
 * @brief E-stop event data structure parsed from ESTOP messages.
 */
struct EstopData {
  bool active = false;                  ///< E-stop activation state
  std::string source = "";              ///< E-stop source identifier
  std::string reason = "";              ///< Human-readable reason
  double trigger_value = 0.0;           ///< Value that triggered E-stop (if applicable)
  bool manual_reset_required = false;  ///< Requires manual reset
  uint64_t timestamp = 0;               ///< Activation timestamp (milliseconds)
  bool valid = false;                   ///< Data validity flag
};

/**
 * @brief Diagnostic data structure parsed from DIAG messages.
 */
struct DiagnosticData {
  std::string level = "INFO";           ///< Diagnostic level (INFO, WARN, ERROR)
  std::string module = "";              ///< Source module name
  std::string message = "";             ///< Diagnostic message
  std::string details = "";             ///< Additional details
  uint64_t timestamp = 0;               ///< Message timestamp
  bool valid = false;                   ///< Data validity flag
};

/**
 * @brief Temperature sensor data structure parsed from TEMP and TEMPERATURE messages.
 */
struct TemperatureData {
  int sensor_id = 0;                    ///< Sensor ID/index
  double temperature_c = 0.0;           ///< Temperature in Celsius
  std::string status = "UNKNOWN";       ///< Sensor status (OK, ERROR, etc.)
  int total_sensors = 0;                ///< Total number of sensors (from array message)
  int active_sensors = 0;               ///< Number of active sensors
  std::vector<double> temperatures;     ///< Array of all temperatures (NaN for invalid)
  double reading_rate_hz = 0.0;         ///< System reading rate
  uint64_t total_readings = 0;          ///< Total readings since startup
  uint64_t total_errors = 0;            ///< Total errors since startup
  bool valid = false;                   ///< Data validity flag
};

/**
 * @brief VL53L0X time-of-flight sensor data structure parsed from VL53L0X messages.
 */
struct VL53L0XData {
  int total_sensors = 0;                ///< Total number of sensors
  int active_sensors = 0;               ///< Number of active sensors
  uint16_t min_distance_mm = 65535;     ///< Minimum distance reading (mm)
  uint16_t max_distance_mm = 0;         ///< Maximum distance reading (mm)
  bool obstacles_detected = false;      ///< Any obstacles detected
  std::vector<uint16_t> distances_mm;   ///< Array of distances (0 for invalid)
  double measurement_rate_hz = 0.0;     ///< System measurement rate
  uint64_t total_measurements = 0;      ///< Total measurements since startup
  uint64_t total_errors = 0;            ///< Total errors since startup
  bool valid = false;                   ///< Data validity flag
};

/**
 * @brief High-performance message parser for TeensyV2 communication.
 * 
 * Efficiently parses structured messages from the TeensyV2 embedded system
 * and converts them to appropriate ROS2 message types. Provides validation,
 * error recovery, and statistical analysis of communication quality.
 * 
 * Features:
 * - Zero-copy parsing where possible
 * - Comprehensive input validation
 * - Automatic error recovery
 * - Communication statistics
 * - Configurable callback system
 * 
 * Usage:
 * @code
 * MessageParser parser;
 * 
 * // Register callbacks for specific message types
 * parser.RegisterCallback(MessageType::BATTERY, 
 *   [](const MessageData& data, rclcpp::Time timestamp) {
 *     // Handle battery data
 *   });
 * 
 * // Parse incoming message
 * std::string message = "BATT:id=0,v=39.8,c=1.2,state=NORMAL";
 * parser.ParseMessage(message);
 * @endcode
 */
class MessageParser {
public:
  /**
   * @brief Constructor with optional logger.
   * 
   * @param[in] logger ROS2 logger for error reporting
   */
  explicit MessageParser(rclcpp::Logger logger = rclcpp::get_logger("message_parser"));

  /**
   * @brief Destructor.
   */
  ~MessageParser() = default;

  /**
   * @brief Register callback for specific message type.
   * 
   * @param[in] type Message type to handle
   * @param[in] callback Function to call when message is received
   */
  void RegisterCallback(MessageType type, MessageCallback callback);

  /**
   * @brief Parse a complete message string.
   * 
   * @param[in] message Complete message string from TeensyV2
   * @param[in] timestamp Optional timestamp (uses current time if not provided)
   * @return true if message was parsed successfully
   */
  bool ParseMessage(const std::string& message, 
                    rclcpp::Time timestamp = rclcpp::Time(0));

  /**
   * @brief Parse battery data from message.
   * 
   * @param[in] data Parsed key-value data
   * @return BatteryData structure with parsed values
   */
  BatteryData ParseBatteryData(const MessageData& data) const;

  /**
   * @brief Parse performance data from message.
   * 
   * @param[in] data Parsed key-value data  
   * @return PerformanceData structure with parsed values
   */
  PerformanceData ParsePerformanceData(const MessageData& data) const;

  /**
   * @brief Parse IMU data from message.
   * 
   * @param[in] data Parsed key-value data
   * @return IMUData structure with parsed values
   */
  IMUData ParseIMUData(const MessageData& data) const;

  /**
   * @brief Parse safety data from message.
   * 
   * @param[in] data Parsed key-value data
   * @return SafetyData structure with parsed values
   */
  SafetyData ParseSafetyData(const MessageData& data) const;

  /**
   * @brief Parse E-stop data from message.
   * 
   * @param[in] data Parsed key-value data
   * @return EstopData structure with parsed values
   */
  EstopData ParseEstopData(const MessageData& data) const;

  /**
   * @brief Parse diagnostic data from message.
   * 
   * @param[in] data Parsed key-value data
   * @return DiagnosticData structure with parsed values
   */
  DiagnosticData ParseDiagnosticData(const MessageData& data) const;

  /**
   * @brief Parse temperature data from TEMP/TEMPERATURE message.
   * 
   * @param[in] data Parsed key-value or JSON data
   * @param[in] is_array_format True if TEMPERATURE format, false if TEMP format
   * @return TemperatureData structure with parsed values
   */
  TemperatureData ParseTemperatureData(const MessageData& data, bool is_array_format = false) const;

  /**
   * @brief Parse VL53L0X data from VL53L0X message.
   * 
   * @param[in] data Parsed JSON data
   * @return VL53L0XData structure with parsed values
   */
  VL53L0XData ParseVL53L0XData(const MessageData& data) const;

  /**
   * @brief Convert battery data to ROS2 BatteryState message.
   * 
   * @param[in] data Parsed battery data
   * @param[in] timestamp Message timestamp
   * @return ROS2 BatteryState message
   */
  sensor_msgs::msg::BatteryState ToBatteryStateMsg(const BatteryData& data,
                                                   rclcpp::Time timestamp) const;

  /**
   * @brief Convert IMU data to ROS2 Imu message.
   * 
   * @param[in] data Parsed IMU data
   * @param[in] timestamp Message timestamp
   * @return ROS2 Imu message
   */
  sensor_msgs::msg::Imu ToImuMsg(const IMUData& data,
                                 rclcpp::Time timestamp) const;

  /**
   * @brief Convert diagnostic data to ROS2 DiagnosticArray message.
   * 
   * @param[in] data Parsed diagnostic data
   * @param[in] timestamp Message timestamp
   * @return ROS2 DiagnosticArray message
   */
  diagnostic_msgs::msg::DiagnosticArray ToDiagnosticArrayMsg(const DiagnosticData& data,
                                                             rclcpp::Time timestamp) const;

  /**
   * @brief Get message parsing statistics.
   * 
   * @return Diagnostic array with parsing statistics
   */
  diagnostic_msgs::msg::DiagnosticArray GetParsingStatistics() const;

  /**
   * @brief Reset parsing statistics.
   */
  void ResetStatistics();

private:
  /**
   * @brief Parse message type from message string.
   * 
   * @param[in] message Complete message string
   * @return Parsed message type
   */
  MessageType ParseMessageType(const std::string& message) const;

  /**
   * @brief Parse key-value pairs from message content.
   * 
   * @param[in] content Message content after type prefix
   * @return Map of key-value pairs
   */
  MessageData ParseKeyValuePairs(const std::string& content) const;

  /**
   * @brief Parse JSON content from PERF messages.
   * 
   * @param[in] content JSON string content
   * @return Map of key-value pairs extracted from JSON
   */
  MessageData ParseJsonContent(const std::string& content) const;

  /**
   * @brief Validate message format and content.
   * 
   * @param[in] message Complete message string
   * @return true if message format is valid
   */
  bool ValidateMessage(const std::string& message) const;

  /**
   * @brief Convert string to message type enum.
   * 
   * @param[in] type_str String representation of message type
   * @return MessageType enumeration
   */
  MessageType StringToMessageType(const std::string& type_str) const;

  /**
   * @brief Convert message type enum to string.
   * 
   * @param[in] type MessageType enumeration
   * @return String representation
   */
  std::string MessageTypeToString(MessageType type) const;

  /**
   * @brief Safe conversion from string to double.
   * 
   * @param[in] str String to convert
   * @param[in] default_value Default value if conversion fails
   * @return Converted double value
   */
  double SafeStringToDouble(const std::string& str, double default_value = 0.0) const;

  /**
   * @brief Safe conversion from string to integer.
   * 
   * @param[in] str String to convert
   * @param[in] default_value Default value if conversion fails
   * @return Converted integer value
   */
  int SafeStringToInt(const std::string& str, int default_value = 0) const;

  /**
   * @brief Safe conversion from string to boolean.
   * 
   * @param[in] str String to convert (true/false, 1/0, yes/no)
   * @param[in] default_value Default value if conversion fails
   * @return Converted boolean value
   */
  bool SafeStringToBool(const std::string& str, bool default_value = false) const;

  /**
   * @brief Parse comma-separated list into vector.
   * 
   * @param[in] str Comma-separated string
   * @return Vector of individual strings
   */
  std::vector<std::string> ParseCommaSeparatedList(const std::string& str) const;

  // Logger for error reporting
  rclcpp::Logger logger_;

  // Message type callbacks
  std::unordered_map<MessageType, MessageCallback> callbacks_;

  // Statistics tracking
  mutable uint64_t total_messages_received_;
  mutable uint64_t total_messages_parsed_;
  mutable uint64_t total_parsing_errors_;
  mutable uint64_t total_validation_errors_;
  mutable std::unordered_map<MessageType, uint64_t> messages_by_type_;
  mutable std::unordered_map<MessageType, uint64_t> errors_by_type_;
};

}  // namespace sigyn_to_sensor_v2
