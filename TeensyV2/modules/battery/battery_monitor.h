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

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <stddef.h>

#include "INA226.h"
#include "serial_manager.h"
#include "../common/core/module.h" // Ensure Module base class is included

namespace sigyn_teensy {
// Minimal BatteryConfig struct for compatibility
struct BatteryConfig {
  float critical_low_voltage = 32.0f;
  float warning_low_voltage = 34.0f;
  float high_current_threshold = 15.0f;
  float critical_high_current = 15.0f; // 15A triggers E-stop
  int update_period_ms = 100;           // 10Hz monitoring
  int report_period_ms = 1000;          // 1Hz status reports
  bool enable_ina226 = true;            // Enable INA226 sensor
  bool enable_analog_voltage = true;    // Enable analog backup
  int ina226_address = 0x40;        // I2C address for INA226
  int analog_pin = 0;                   // Analog voltage input (A0)
  float voltage_divider_ratio = 11.0f;  // 10:1 + safety margin
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
  BatteryMonitor();
  void setup() override;
  void loop() override;
  const char* name() const override;
  float getVoltage(size_t idx = 0) const;
  float getCurrent(size_t idx = 0) const;
  float updateExponentialAverage(float current_avg, float new_value, float alpha);
  void selectSensor(size_t battery_idx) const;
  bool testI2CMultiplexer();

  // Legacy API compatibility wrappers
  static BatteryMonitor& GetInstance();
  void Configure(const BatteryConfig& config);
  float GetVoltage(size_t idx = 0) const { return getVoltage(idx); }
  float GetCurrent(size_t idx = 0) const { return getCurrent(idx); }
  int GetState() const { return 0; } // Stub: implement state logic if needed
  bool IsSensorHealthy() const { return true; } // Stub: implement health check if needed


 private:
  float EstimateChargePercentage(float voltage) const;
  void SendStatusMessage(size_t idx);
  void UpdateBatteryState(size_t idx);

  static constexpr size_t kNumberOfBatteries = 1;  // Only LIPO for now
  static constexpr float k36VAnalogToVoltageConversion = 0.087393162f;
  static constexpr float kDefaultVoltageAlpha = 0.1f;
  static constexpr float kDefaultCurrentAlpha = 0.2f;
  static constexpr int MAIN_BATTERY_PIN = 24;
  static constexpr int kI2CMultiplexorEnablePin = 8;
  static constexpr int MAIN_BATTERY_REPORT_INTERVAL_MS = 1000;  // 1Hz reporting
  bool setup_completed_;
  bool multiplexer_available_;
  INA226* ina226_[kNumberOfBatteries];
  float voltage_ema_[kNumberOfBatteries];
  float current_ema_[kNumberOfBatteries];
  BatteryState state_[kNumberOfBatteries];
  size_t total_readings_[kNumberOfBatteries];
  BatteryConfig config_;
  static uint8_t gINA226_DeviceIndexes_[kNumberOfBatteries];
};

}  // namespace sigyn_teensy
