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
 * @author Wimble Robotics
 * @date 2025
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "../common/core/module.h"  // Ensure Module base class is included
#include "INA226.h"
#include "serial_manager.h"

namespace sigyn_teensy {
/**
 * @brief Configuration parameters for battery monitoring system.
 * 
 * This structure contains all configurable parameters for the battery monitoring
 * system, including safety thresholds, sensor configuration, and reporting intervals.
 * Values are carefully chosen based on LiFePO4 battery characteristics and robot
 * safety requirements.
 * 
 * **Safety Threshold Design Philosophy:**
 * Battery safety thresholds are set with multiple layers of protection:
 * 1. Warning thresholds: Early notification to allow graceful shutdown
 * 2. Critical thresholds: Immediate safety response to prevent damage
 * 3. Hysteresis: Different thresholds for alarm and recovery to prevent oscillation
 * 
 * **LiFePO4 Battery Characteristics (12S configuration):**
 * - Nominal voltage: 38.4V (3.2V per cell)
 * - Fully charged: 43.2V (3.6V per cell)
 * - Safe discharge minimum: 33.6V (2.8V per cell)
 * - Critical minimum: 31.2V (2.6V per cell)
 * 
 * **Current Monitoring:**
 * Current thresholds are based on motor specifications and power electronics
 * ratings. High current conditions may indicate motor stall, short circuits,
 * or other dangerous conditions requiring immediate intervention.
 * 
 * **Update Rates:**
 * - Fast monitoring (100ms): Critical for safety, allows rapid response
 * - Slower reporting (1s): Reduces communication overhead for non-critical data
 * 
 * **Sensor Redundancy:**
 * Both INA226 and analog voltage monitoring can be enabled simultaneously
 * to provide sensor redundancy and cross-validation of measurements.
 */
struct BatteryConfig {
  // Safety voltage thresholds (Volts)
  float critical_low_voltage = 32.0f;    ///< Emergency shutdown voltage - triggers immediate E-stop
  float warning_low_voltage = 34.0f;     ///< Low battery warning - initiates return-to-base procedure
  float critical_high_voltage = 45.0f;   ///< Overvoltage protection - prevents overcharging damage
  float warning_high_voltage = 44.0f;    ///< High voltage warning - charging system notification
  
  // Current monitoring thresholds (Amperes)
  float high_current_threshold = 15.0f;   ///< High current warning - indicates heavy load
  float critical_high_current = 20.0f;    ///< Critical current limit - triggers E-stop for safety
  float charging_current_max = 5.0f;      ///< Maximum safe charging current
  
  // Power and energy thresholds
  float max_power_watts = 500.0f;         ///< Maximum continuous power draw
  float energy_warning_wh = 50.0f;        ///< Remaining energy warning threshold
  
  // Timing and reporting configuration
  int update_period_ms = 100;             ///< Sensor reading interval (10Hz) - safety critical
  int report_period_ms = 1000;            ///< Status reporting interval (1Hz) - for logging/ROS2
  int calibration_samples = 100;          ///< Number of samples for sensor calibration
  
  // Sensor enable/disable flags
  bool enable_ina226 = true;              ///< Enable high-precision INA226 current sensor
  bool enable_analog_voltage = true;      ///< Enable analog voltage divider backup measurement
  bool enable_temperature_monitoring = false; ///< Enable battery temperature monitoring if available
  bool enable_cell_balancing = false;     ///< Enable individual cell monitoring if available
  
  // Hardware configuration
  int ina226_address = 0x40;              ///< I2C address for primary INA226 sensor
  int analog_voltage_pin = A0;            ///< ADC pin for analog voltage measurement
  int temperature_pin = A1;               ///< ADC pin for temperature sensor (if used)
  float voltage_divider_ratio = 11.0f;    ///< Voltage divider scaling factor for 10:1 divider with safety margin
  
  // Calibration and accuracy parameters
  float voltage_calibration_offset = 0.0f; ///< Voltage measurement offset correction
  float current_calibration_offset = 0.0f; ///< Current measurement offset correction
  float voltage_calibration_scale = 1.0f;  ///< Voltage measurement scale correction
  float current_calibration_scale = 1.0f;  ///< Current measurement scale correction
};

/**
 * @brief Battery state enumeration for status reporting.
 * 
 * Represents the current operational state of the battery system for
 * monitoring, diagnostics, and safety response coordination.
 */
enum class BatteryState {
  UNKNOWN,      ///< State not yet determined (initialization phase)
  CHARGING,     ///< Battery is actively charging (positive current flow)
  DISCHARGING,  ///< Battery is providing power (negative current flow)
  CRITICAL,     ///< Battery at critical level (triggers emergency stop)
  WARNING,      ///< Battery at warning level (triggers return-to-base)
  NORMAL        ///< Battery operating within normal parameters
};

class BatteryMonitor : public Module {
 public:
  // --- Public API ---
  /**
   * @brief Get singleton instance of BatteryMonitor.
   * @return Reference to singleton BatteryMonitor
   */
  static BatteryMonitor& getInstance();

  /**
   * @brief Get the current (in Amps) for the specified battery index.
   * @param idx Battery index (default: 0)
   * @return Current in Amps
   */
  float getCurrent(size_t idx = 0) const;

  /**
   * @brief Get the voltage (in Volts) for the specified battery index.
   * @param idx Battery index (default: 0)
   * @return Voltage in Volts
   */
  float getVoltage(size_t idx = 0) const;

  /**
   * @brief Return the name of this module.
   */
  const char* name() const override;

  // --- Legacy API (Stubs) ---
  /**
   * @brief Legacy API: Get battery state (stub).
   * @return Always returns 0 (not implemented)
   */
  int getState() const { return 0; }

  /**
   * @brief Legacy API: Check if sensor is healthy (stub).
   * @return Always returns true (not implemented)
   */
  bool isSensorHealthy() const { return true; }

 protected:
  /**
   * @brief Perform one-time initialization for the module.
   */
  void setup() override;

  /**
   * @brief Perform regular, cyclic work for the module.
   */
  void loop() override;

 private:
  /**
   * @brief Private constructor for singleton pattern.
   * Initializes all battery monitoring hardware and state.
   */
  BatteryMonitor();

  // Prevent copying
  BatteryMonitor(const BatteryMonitor&) = delete;
  BatteryMonitor& operator=(const BatteryMonitor&) = delete;

  // --- Static members ---

  /**
   * @brief Number of batteries supported (currently only LIPO).
   */
  static constexpr size_t kNumberOfBatteries = 3;

  /**
   * @brief Per-battery configuration (static/global).
   */
  static BatteryConfig g_battery_config_[kNumberOfBatteries];

  /**
   * @brief INA226 sensor instances for power supply monitoring.
   */
  static INA226 g_ina226_[kNumberOfBatteries];

  /**
   * @brief Device indexes for INA226 sensors.
   */
  static uint8_t gINA226_DeviceIndexes_[kNumberOfBatteries];

  /**
   * @brief Conversion factor for analog voltage readings (36V system).
   */
  static constexpr float k36VAnalogToVoltageConversion = 0.087393162f;

  /**
   * @brief Exponential averaging alpha for current readings.
   */
  static constexpr float kDefaultCurrentAlpha = 0.2f;

  /**
   * @brief Exponential averaging alpha for voltage readings.
   */
  static constexpr float kDefaultVoltageAlpha = 0.1f;

  /**
   * @brief I2C address for multiplexer.
   */
  static constexpr int I2C_MULTIPLEXER_ADDRESS = 0x70;

  /**
   * @brief Main battery analog pin.
   */
  static constexpr int MAIN_BATTERY_PIN = 24;

  /**
   * @brief Main battery status report interval (ms).
   */
  static constexpr int MAIN_BATTERY_REPORT_INTERVAL_MS = 1000;

  /**
   * @brief I2C multiplexer enable pin.
   */
  static constexpr int kI2CMultiplexorEnablePin = 8;

  // --- Data members ---

  /**
   * @brief Per-instance battery configuration.
   */
  BatteryConfig config_;

  /**
   * @brief True if I2C multiplexer is available.
   */
  bool multiplexer_available_;

  /**
   * @brief True if setup() has completed successfully.
   */
  bool setup_completed_;

  /**
   * @brief Battery state for each battery.
   */
  BatteryState state_[kNumberOfBatteries];

  /**
   * @brief Exponential moving average of current readings.
   */
  float current_ema_[kNumberOfBatteries];

  /**
   * @brief Exponential moving average of voltage readings.
   */
  float voltage_ema_[kNumberOfBatteries];

  /**
   * @brief Pointer to INA226 sensor objects for each battery.
   */
  INA226* ina226_[kNumberOfBatteries];

  /**
   * @brief Total number of readings taken for each battery.
   */
  size_t total_readings_[kNumberOfBatteries];

  // --- Member functions ---

  /**
   * @brief Estimate battery charge percentage from voltage.
   * @param voltage Battery voltage
   * @return Estimated charge percentage (0-100)
   */
  float estimateChargePercentage(float voltage) const;

  /**
   * @brief Send battery status message for specified battery index.
   * @param idx Battery index
   */
  void sendStatusMessage(size_t idx);

  /**
   * @brief Convert BatteryState enum to string representation.
   * @param state Battery state enum value
   * @return String representation of battery state
   */
  const char* batteryStateToString(BatteryState state) const;

  /**
   * @brief Update battery state for specified battery index.
   * @param idx Battery index
   */
  void updateBatteryState(size_t idx);

  /**
   * @brief Update exponential average for a value.
   * @param current_avg Current average value
   * @param new_value New value to include
   * @param alpha Averaging factor
   * @return Updated average
   */
  float updateExponentialAverage(float current_avg, float new_value,
                                 float alpha);

  /**
   * @brief Select sensor for specified battery index (multiplexer).
   * @param battery_idx Battery index
   */
  void selectSensor(size_t battery_idx) const;

  /**
   * @brief Test if I2C multiplexer is available and working.
   * @return True if multiplexer is available
   */
  bool testI2cMultiplexer();
};

}  // namespace sigyn_teensy
