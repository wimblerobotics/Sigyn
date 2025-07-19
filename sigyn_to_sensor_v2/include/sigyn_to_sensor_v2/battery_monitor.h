/**
 * @file battery_monitor.h
 * @brief Battery monitoring ROS2 node for TeensyV2 system
 *
 * Receives and processes battery monitoring data from TeensyV2 boards,
 * publishes battery status and diagnostics.
 *
 * @author GitHub Copilot
 * @date 2025
 */

#ifndef SIGYN_TO_SENSOR_V2_BATTERY_MONITOR_H_
#define SIGYN_TO_SENSOR_V2_BATTERY_MONITOR_H_


#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <map>
#include <cstdint>

namespace sigyn_to_sensor_v2 {

// ROS2 node for battery monitoring from TeensyV2 boards.
// Processes battery data received from multiple TeensyV2 boards and publishes
// standardized battery status messages and diagnostics.
class BatteryMonitor : public rclcpp::Node {
 public:
  BatteryMonitor();
  ~BatteryMonitor() override = default;

  // Processes battery data from TeensyV2 board.
  // @param board_id Board ID
  // @param voltage Battery voltage (V)
  // @param current Battery current (A)
  // @param temperature Battery temperature (C)
  // @param charge_percentage Estimated charge percentage
  void ProcessBatteryData(uint8_t board_id, float voltage, float current,
                         float temperature, uint8_t charge_percentage);

 private:
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  // Parameters
  double voltage_warn_threshold_;       // Voltage warning threshold
  double voltage_error_threshold_;      // Voltage error threshold
  double current_warn_threshold_;       // Current warning threshold
  double temperature_warn_threshold_;   // Temperature warning threshold
  double temperature_error_threshold_;  // Temperature error threshold

  // Timer for periodic diagnostics
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  // Battery state tracking
  struct BatteryState {
    float voltage = 0.0f;
    float current = 0.0f;
    float temperature = 0.0f;
    uint8_t charge_percentage = 0;
    rclcpp::Time last_update;
    bool data_valid = false;
  };

  // Per-board battery states
  std::map<uint8_t, BatteryState> battery_states_;

  // Initializes ROS2 parameters.
  void InitializeParameters();

  // Parameter change callback.
  // @param event Parameter event
  void ParameterCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

  // Publishes battery diagnostics periodically.
  void PublishDiagnostics();

  // Checks if battery values are within safe ranges.
  // @param state Battery state to check
  // @return Diagnostic level (OK, WARN, ERROR)
  uint8_t CheckBatteryHealth(const BatteryState& state);

  // Converts charge percentage to battery power state.
  // @param charge_percentage Charge percentage (0-100)
  // @return Battery power state enum
  uint8_t GetPowerState(uint8_t charge_percentage);
};

}  // namespace sigyn_to_sensor_v2

#endif  // SIGYN_TO_SENSOR_V2_BATTERY_MONITOR_H_
