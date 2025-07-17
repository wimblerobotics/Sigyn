/**
 * @file battery_monitor.h
 * @brief Comprehensive battery monitoring system for TeensyV2
 * 
 * Provides real-time monitoring of battery voltage, current, power, and state
 * with support for both INA226 current sensors and analog voltage monitoring.
 * Integrates with the safety system for critical battery conditions.
 * 
 * Features:
 * - Multi-sensor support (INA226, analog voltage dividers)
 * - Real-time power consumption tracking
 * - Battery state estimation (charging, discharging, critical)
 * - Safety integration with automatic E-stop on critical conditions
 * - Efficient messaging for ROS2 integration
 * 
 * Safety Thresholds:
 * - Critical low voltage: 32.0V (triggers E-stop)
 * - Warning low voltage: 34.0V (triggers warning)
 * - High current threshold: 15.0A (triggers E-stop)
 * - Temperature monitoring (if available)
 * 
 * @author Sigyn Robotics
 * @date 2025
 */

#pragma once

#include "../../common/core/module.h"
#include "../../common/core/serial_manager.h"
#include <Wire.h>

namespace sigyn_teensy {

/**
 * @brief Battery state enumeration for status reporting.
 */
enum class BatteryState {
  UNKNOWN,     ///< State not yet determined
  CHARGING,    ///< Battery is charging
  DISCHARGING, ///< Battery is discharging
  CRITICAL,    ///< Battery at critical level (emergency stop)
  WARNING,     ///< Battery at warning level
  NORMAL       ///< Battery operating normally
};

/**
 * @brief Configuration structure for battery monitoring parameters.
 */
struct BatteryConfig {
  // Voltage thresholds (V)
  float critical_low_voltage = 32.0f;    ///< Triggers E-stop
  float warning_low_voltage = 34.0f;     ///< Triggers warning
  float nominal_voltage = 36.0f;         ///< Nominal operating voltage
  float max_voltage = 42.0f;             ///< Maximum safe voltage
  
  // Current thresholds (A)
  float critical_high_current = 15.0f;   ///< Triggers E-stop
  float warning_high_current = 12.0f;    ///< Triggers warning
  
  // Power thresholds (W)
  float max_power = 500.0f;              ///< Maximum safe power draw
  
  // Timing parameters (ms)
  uint32_t update_period_ms = 100;       ///< Sensor update frequency
  uint32_t report_period_ms = 1000;      ///< Status report frequency
  
  // Sensor configuration
  bool enable_ina226 = true;             ///< Enable INA226 current sensor
  bool enable_analog_voltage = true;     ///< Enable analog voltage monitoring
  uint8_t ina226_address = 0x40;         ///< I2C address for INA226
  uint8_t analog_pin = A0;               ///< Analog pin for voltage monitoring
  float voltage_divider_ratio = 11.0f;   ///< Voltage divider ratio for analog
};

/**
 * @brief Comprehensive battery monitoring module.
 * 
 * Monitors battery voltage, current, and power using multiple sensor types.
 * Provides safety monitoring with automatic E-stop on critical conditions.
 * Integrates with the TeensyV2 module system for real-time operation.
 * 
 * The module supports:
 * - INA226 high-precision current/voltage sensors
 * - Analog voltage monitoring with configurable divider ratios
 * - Real-time power consumption tracking
 * - Battery state estimation and reporting
 * - Safety threshold monitoring with automatic E-stop
 * 
 * Usage:
 * @code
 * // Get singleton instance (automatically registers with module system)
 * BatteryMonitor& battery = BatteryMonitor::GetInstance();
 * 
 * // Configure if needed (uses defaults if not called)
 * BatteryConfig config;
 * config.critical_low_voltage = 30.0f;
 * battery.Configure(config);
 * 
 * // Module system handles setup() and loop() automatically
 * @endcode
 */
class BatteryMonitor : public Module {
 public:
  /**
   * @brief Get the singleton instance of BatteryMonitor.
   * 
   * @return Reference to the singleton BatteryMonitor instance
   */
  static BatteryMonitor& GetInstance();

  /**
   * @brief Configure battery monitoring parameters.
   * 
   * @param[in] config Configuration structure with monitoring parameters
   */
  void Configure(const BatteryConfig& config);

  /**
   * @brief Get current battery voltage (V).
   * 
   * @return Battery voltage in volts, or NaN if sensor unavailable
   */
  float GetVoltage() const { return voltage_; }

  /**
   * @brief Get current battery current (A).
   * 
   * Positive values indicate discharging, negative values indicate charging.
   * 
   * @return Battery current in amperes, or NaN if sensor unavailable
   */
  float GetCurrent() const { return current_; }

  /**
   * @brief Get current power consumption (W).
   * 
   * @return Power consumption in watts, or NaN if sensors unavailable
   */
  float GetPower() const { return power_; }

  /**
   * @brief Get current battery state.
   * 
   * @return Current battery state enumeration
   */
  BatteryState GetState() const { return state_; }

  /**
   * @brief Get battery charge percentage estimate.
   * 
   * @return Estimated charge percentage (0.0-1.0), or NaN if unavailable
   */
  float GetChargePercentage() const { return charge_percentage_; }

  /**
   * @brief Check if battery monitoring sensors are functioning.
   * 
   * @return true if at least one sensor is providing valid data
   */
  bool IsSensorHealthy() const { return sensor_healthy_; }

 protected:
  // Module interface implementation
  void setup() override;
  void loop() override;
  const char* name() override { return "BatteryMonitor"; }
  bool IsUnsafe() override;
  void ProcessMessage(const String& message) override;

 private:
  /**
   * @brief Private constructor for singleton pattern.
   */
  BatteryMonitor();

  /**
   * @brief Initialize INA226 current sensor.
   * 
   * @return true if initialization successful
   */
  bool InitializeINA226();

  /**
   * @brief Read voltage from INA226 sensor.
   * 
   * @return Voltage in volts, or NaN if read failed
   */
  float ReadINA226Voltage();

  /**
   * @brief Read current from INA226 sensor.
   * 
   * @return Current in amperes, or NaN if read failed
   */
  float ReadINA226Current();

  /**
   * @brief Read voltage from analog pin.
   * 
   * @return Voltage in volts, or NaN if read failed
   */
  float ReadAnalogVoltage();

  /**
   * @brief Update battery state based on current measurements.
   */
  void UpdateBatteryState();

  /**
   * @brief Send battery status message via SerialManager.
   */
  void SendStatusMessage();

  /**
   * @brief Check safety thresholds and trigger warnings/E-stops.
   */
  void CheckSafetyThresholds();

  /**
   * @brief Estimate battery charge percentage based on voltage.
   * 
   * @param[in] voltage Current battery voltage
   * @return Estimated charge percentage (0.0-1.0)
   */
  float EstimateChargePercentage(float voltage) const;

  // Configuration
  BatteryConfig config_;

  // Sensor readings
  float voltage_;              ///< Current voltage (V)
  float current_;              ///< Current current (A)
  float power_;                ///< Current power (W)
  float charge_percentage_;    ///< Estimated charge (0.0-1.0)
  BatteryState state_;         ///< Current battery state

  // Sensor status
  bool ina226_available_;      ///< INA226 sensor available and functioning
  bool analog_available_;      ///< Analog voltage sensor available
  bool sensor_healthy_;        ///< At least one sensor working

  // Timing control
  uint32_t last_sensor_update_;    ///< Last sensor read timestamp
  uint32_t last_status_report_;    ///< Last status message timestamp

  // Safety tracking
  bool voltage_critical_;      ///< Critical voltage condition active
  bool current_critical_;      ///< Critical current condition active
  bool power_critical_;        ///< Critical power condition active
  uint32_t safety_violation_start_; ///< Timestamp of first safety violation

  // Moving average for noise reduction
  static constexpr size_t kAverageWindow = 5;
  float voltage_history_[kAverageWindow];
  float current_history_[kAverageWindow];
  size_t history_index_;
  size_t history_count_;

  // Serial communication
  SerialManager* serial_manager_;
};

}  // namespace sigyn_teensy
