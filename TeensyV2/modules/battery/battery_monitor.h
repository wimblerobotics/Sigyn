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
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <Wire.h>
#include "../common/core/module.h"  // Ensure Module base class is included
#include "INA226.h"
#include "serial_manager.h"

namespace sigyn_teensy {
// Minimal BatteryConfig struct for compatibility
struct BatteryConfig {
  float critical_low_voltage = 32.0f;
  float warning_low_voltage = 34.0f;
  float high_current_threshold = 15.0f;
  float critical_high_current = 15.0f;  // 15A triggers E-stop
  int update_period_ms = 100;           // 10Hz monitoring
  int report_period_ms = 1000;          // 1Hz status reports
  bool enable_ina226 = true;            // Enable INA226 sensor
  bool enable_analog_voltage = true;    // Enable analog backup
  int ina226_address = 0x40;            // I2C address for INA226
  int analog_pin = 0;                   // Analog voltage input (A0)
  float voltage_divider_ratio = 11.0f;  // 10:1 + safety margin
  BatteryConfig(float critical_low_voltage = 32.0f,
                float warning_low_voltage = 34.0f,
                float high_current_threshold = 15.0f,
                float critical_high_current = 15.0f,
                int update_period_ms = 100,
                int report_period_ms = 1000,
                bool enable_ina226 = true,
                bool enable_analog_voltage = true,
                int ina226_address = 0x40,
                int analog_pin = 0,
                float voltage_divider_ratio = 11.0f)
      : critical_low_voltage(critical_low_voltage),
        warning_low_voltage(warning_low_voltage),
        high_current_threshold(high_current_threshold),
        critical_high_current(critical_high_current),
        update_period_ms(update_period_ms),
        report_period_ms(report_period_ms),
        enable_ina226(enable_ina226),
        enable_analog_voltage(enable_analog_voltage),
        ina226_address(ina226_address),
        analog_pin(analog_pin),
        voltage_divider_ratio(voltage_divider_ratio) {}
};

/**
 * @brief Battery state enumeration for status reporting.
 */
enum class BatteryState {
  UNKNOWN,      ///< State not yet determined
  CHARGING,     ///< Battery is charging
  DISCHARGING,  ///< Battery is discharging
  CRITICAL,     ///< Battery at critical level (emergency stop)
  WARNING,      ///< Battery at warning level
  NORMAL        ///< Battery operating normally
};

class BatteryMonitor : public Module {
public:
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
   * @brief Main loop for battery monitoring. Call regularly from main loop.
   */
  void loop() override;

  /**
   * @brief Get the module name string.
   * @return Module name
   */
  const char* name() const override;

  /**
   * @brief Setup routine for battery monitoring hardware and state.
   */
  void setup() override;

  /**
   * @brief Legacy API: Get current (Amps) for specified battery index.
   */
  float GetCurrent(size_t idx = 0) const { return getCurrent(idx); }

  /**
   * @brief Legacy API: Get voltage (Volts) for specified battery index.
   */
  float GetVoltage(size_t idx = 0) const { return getVoltage(idx); }

  /**
   * @brief Legacy API: Get battery state (stub).
   * @return Always returns 0 (not implemented)
   */
  int GetState() const { return 0; }

  /**
   * @brief Legacy API: Check if sensor is healthy (stub).
   * @return Always returns true (not implemented)
   */
  bool IsSensorHealthy() const { return true; }

  /**
   * @brief Get singleton instance of BatteryMonitor.
   * @return Reference to singleton BatteryMonitor
   */
  static BatteryMonitor& GetInstance();

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
  static constexpr size_t kNumberOfBatteries = 1;

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
  float EstimateChargePercentage(float voltage) const;

  /**
   * @brief Send battery status message for specified battery index.
   * @param idx Battery index
   */
  void SendStatusMessage(size_t idx);

  /**
   * @brief Update battery state for specified battery index.
   * @param idx Battery index
   */
  void UpdateBatteryState(size_t idx);

  /**
   * @brief Update exponential average for a value.
   * @param current_avg Current average value
   * @param new_value New value to include
   * @param alpha Averaging factor
   * @return Updated average
   */
  float updateExponentialAverage(float current_avg, float new_value, float alpha);

  /**
   * @brief Select sensor for specified battery index (multiplexer).
   * @param battery_idx Battery index
   */
  void selectSensor(size_t battery_idx) const;

  /**
   * @brief Test if I2C multiplexer is available and working.
   * @return True if multiplexer is available
   */
  bool testI2CMultiplexer();
};

}  // namespace sigyn_teensy
