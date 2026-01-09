/**
 * @file performance_monitor.h
 * @brief Performance monitoring ROS2 node for TeensyV2 system
 *
 * Receives and processes performance monitoring data from TeensyV2 boards,
 * publishes performance metrics and diagnostics.
 *
 * @author GitHub Copilot
 * @date 2025
 */

#ifndef SIGYN_TO_SENSOR_V2_PERFORMANCE_MONITOR_H_
#define SIGYN_TO_SENSOR_V2_PERFORMANCE_MONITOR_H_

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

namespace sigyn_to_sensor_v2 {

/**
 * @brief ROS2 node for performance monitoring from TeensyV2 boards
 *
 * Processes performance data received from multiple TeensyV2 boards and publishes
 * performance metrics and diagnostics for system health monitoring.
 */
  class PerformanceMonitor: public rclcpp::Node {
public:
  /**
   * @brief Constructor
   */
    PerformanceMonitor();

  /**
   * @brief Destructor
   */
    ~PerformanceMonitor() = default;

  /**
   * @brief Process performance data from TeensyV2 board
   * @param board_id Board ID
   * @param loop_frequency Current loop frequency (Hz)
   * @param avg_execution_time Average execution time (us)
   * @param max_execution_time Maximum execution time (us)
   * @param memory_usage Memory usage percentage
   * @param violations_count Number of performance violations
   */
    void ProcessPerformanceData(
      uint8_t board_id, float loop_frequency,
      uint32_t avg_execution_time, uint32_t max_execution_time,
      uint8_t memory_usage, uint32_t violations_count);

private:
  // Publishers
    rclcpp::Publisher < diagnostic_msgs::msg::DiagnosticArray > ::SharedPtr diagnostics_pub_;
    rclcpp::Publisher < std_msgs::msg::Float32 > ::SharedPtr frequency_pub_;
    rclcpp::Publisher < std_msgs::msg::Int32 > ::SharedPtr execution_time_pub_;
    rclcpp::Publisher < std_msgs::msg::Int32 > ::SharedPtr memory_usage_pub_;

  // Parameters
    double frequency_warn_threshold_;   ///< Minimum frequency warning threshold
    double frequency_error_threshold_;  ///< Minimum frequency error threshold
    uint32_t execution_time_warn_threshold_; ///< Maximum execution time warning (us)
    uint32_t execution_time_error_threshold_; ///< Maximum execution time error (us)
    uint8_t memory_warn_threshold_;     ///< Memory usage warning threshold (%)
    uint8_t memory_error_threshold_;    ///< Memory usage error threshold (%)

  // Timer for periodic diagnostics
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  // Performance state tracking
    struct PerformanceState
    {
      float loop_frequency = 0.0f;
      uint32_t avg_execution_time = 0;
      uint32_t max_execution_time = 0;
      uint8_t memory_usage = 0;
      uint32_t violations_count = 0;
      uint32_t last_violations_count = 0; // For delta calculation
      rclcpp::Time last_update;
      bool data_valid = false;
    };

    std::map < uint8_t, PerformanceState > performance_states_; ///< Per-board performance states

  /**
   * @brief Initialize ROS2 parameters
   */
    void InitializeParameters();

  /**
   * @brief Parameter change callback
   * @param event Parameter event
   */
    void ParameterCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

  /**
   * @brief Publish performance diagnostics periodically
   */
    void PublishDiagnostics();

  /**
   * @brief Check if performance values are within acceptable ranges
   * @param state Performance state to check
   * @return Diagnostic level (OK, WARN, ERROR)
   */
    uint8_t CheckPerformanceHealth(const PerformanceState & state);

  /**
   * @brief Get performance severity based on thresholds
   * @param state Performance state
   * @return Severity message string
   */
    std::string GetPerformanceSeverityMessage(const PerformanceState & state);
  };

}  // namespace sigyn_to_sensor_v2

#endif  // SIGYN_TO_SENSOR_V2_PERFORMANCE_MONITOR_H_
