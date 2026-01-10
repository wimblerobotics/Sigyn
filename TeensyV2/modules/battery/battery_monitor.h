// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file battery_monitor.h
 * @brief Comprehensive battery monitoring system for TeensyV2
 *
 * Provides real-time monitoring of battery voltage, current, power, and state
 * with support for both INA226 voltage/current sensors and analog voltage monitoring.
 * Integrates with the safety system for critical battery conditions.
 *
 * Features:
 * - Multi-sensor support (INA226, analog voltage dividers)
 * - Real-time power consumption tracking
 * - Battery state estimation (charging, discharging, critical)
 * - Safety integration with automatic E-stop on critical conditions
 * - Efficient messaging for ROS2 integration
 * - DI support for testing
 *
 * Safety Thresholds:
 * - Critical low voltage: 32.0V (triggers E-stop)
 * - Warning low voltage: 34.0V (triggers warning)
 * - High current threshold: 15.0A (triggers E-stop)
 *
 * @author Wimble Robotics
 * @date 2025
 */

#pragma once

#include <Arduino.h>
#ifndef UNIT_TEST
#include <Wire.h>
#endif
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "../../common/core/module.h"
#include "../../common/core/serial_manager.h"
#include "interfaces/i_power_sensor.h"

namespace sigyn_teensy {

  /**
   * @brief Configuration parameters for battery monitoring system.
   */
  struct BatteryConfig {
    // Safety voltage thresholds (Volts)
    float critical_low_voltage = 32.0f;    ///< Emergency shutdown voltage - triggers immediate E-stop
    float warning_low_voltage = 34.0f;     ///< Low battery warning - initiates return-to-base procedure
    float voltage_multiplier = 1.0f;       ///< Voltage scaling factor
    
    // Current monitoring thresholds (Amperes)
    float critical_high_current = 20.0f;   ///< Critical current limit - triggers E-stop

    // Timing and reporting configuration
    int update_period_ms = 100;

    // Hardware configuration
    int ina226_address = 0x40;
    int analog_voltage_pin = A0;
    int temperature_pin = A1;
    float voltage_divider_ratio = 11.0f;

    float voltage_calibration_offset = 0.0f;
    float current_calibration_offset = 0.0f;
    float voltage_calibration_scale = 1.0f;
    float current_calibration_scale = 1.0f;
    char battery_name[32];
  };

  /**
   * @brief Battery state enumeration for status reporting.
   */
  enum class BatteryState {
    UNKNOWN,      ///< State not yet determined (initialization phase)
    CHARGING,     ///< Battery is actively charging
    DISCHARGING,  ///< Battery is providing power
    CRITICAL,     ///< Battery at critical level (triggers emergency stop)
    WARNING,      ///< Battery at warning level (triggers return-to-base)
    NORMAL,       ///< Battery operating within normal parameters
    DEGRADED      ///< Sensor failure or communication loss
  };

  class BatteryMonitor : public Module {
  public:
    // --- Public API ---
    static BatteryMonitor& getInstance();

    /**
     * @brief Inject a power sensor implementation.
     * Must be called before setup() if mocking is desired.
     * @param index Battery index (0-4)
     * @param sensor Pointer to sensor implementation
     */
    void registerSensor(size_t index, IPowerSensor* sensor);

    float getCurrent(size_t idx = 0) const;
    float getVoltage(size_t idx = 0) const;
    BatteryState getBatteryState(size_t idx = 0) const { return state_[idx]; }

    const char* name() const override;

    /**
     * @brief Check for safety violations.
     * Returns true if any battery is in CRITICAL state.
     */
    bool isUnsafe() override;

    // --- Legacy API ---
    int getState() const { return 0; }
    bool isSensorHealthy() const { return true; }

#ifdef UNIT_TEST
    // Public wrappers for testing
    void testSetup() { setup(); }
    void testLoop() { loop(); }
    
    // Reset internal state for test isolation
    void testReset() {
      for (size_t i = 0; i < kNumberOfBatteries; i++) {
        voltage_ema_[i] = 0.0f;
        current_ema_[i] = 0.0f;
        total_readings_[i] = 0;
        state_[i] = BatteryState::UNKNOWN;
      }
    }
#endif

  protected:
    void setup() override;
    void loop() override;

  private:
    BatteryMonitor();
    BatteryMonitor(const BatteryMonitor&) = delete;
    BatteryMonitor& operator=(const BatteryMonitor&) = delete;

    static constexpr size_t kNumberOfBatteries = 5;
    static BatteryConfig g_battery_config_[kNumberOfBatteries];
    static uint8_t gINA226_DeviceIndexes_[kNumberOfBatteries];

    static constexpr float kDefaultCurrentAlpha = 0.5f;  // Converges in ~3 readings
    static constexpr float kDefaultVoltageAlpha = 0.4f;  // Converges in ~4 readings
    static constexpr int I2C_MULTIPLEXER_ADDRESS = 0x70;
    static constexpr int MAIN_BATTERY_REPORT_INTERVAL_MS = 1000;
    static constexpr int kI2CMultiplexorEnablePin = 8;

    BatteryConfig config_;
    bool multiplexer_available_;
    bool setup_completed_;

    BatteryState state_[kNumberOfBatteries];
    float current_ema_[kNumberOfBatteries];
    float voltage_ema_[kNumberOfBatteries];

    // Polymorphic sensors
    IPowerSensor* sensors_[kNumberOfBatteries];
    bool sensor_owned_[kNumberOfBatteries]; // True if we created the sensor and should delete it

    size_t total_readings_[kNumberOfBatteries];

    float estimateChargePercentage(float voltage) const;
    void sendStatusMessage(size_t idx);
    const char* batteryStateToString(BatteryState state) const;
    void updateBatteryState(size_t idx);
    float updateExponentialAverage(float current_avg, float new_value, float alpha);
    void selectSensor(size_t battery_idx) const;
    bool testI2cMultiplexer();
  };

}  // namespace sigyn_teensy
