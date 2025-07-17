#pragma once

#include <stdint.h>

#include "INA226.h"  // For INA226 battery monitoring.
#include "config.h"
#include "module.h"

/**
 * @brief Battery monitoring module for Teensy board 2.
 * 
 * This module provides comprehensive monitoring for two types of power systems:
 * 1. LIPO Battery: 36V nominal (10-cell), 30Ah capacity with analog voltage monitoring
 * 2. Power Supplies: 24V, 12V, 5V, 3.3V rails derived from main battery via INA226 sensors
 * 
 * Key features:
 * - Dual monitoring system: analog for LIPO, INA226 for power supplies
 * - Exponential averaging for voltage and current with outlier detection
 * - Per-battery safety monitoring with configurable alarm thresholds
 * - Motor overcurrent protection (critical for 24V supply)
 * - Scalable architecture ready for additional power supplies
 * - I2C multiplexer support for multiple INA226 sensors
 * 
 * Safety Features:
 * - Overvoltage detection with configurable consecutive reading threshold
 * - Overcurrent protection with time-based thresholds
 * - Per-battery alarm states with global unsafe condition aggregation
 * - Motor stall detection via prolonged overcurrent on 24V supply
 * 
 * @note This implementation is specific to Teensy board 2 hardware configuration.
 */
class BatteryModule : public Module {
 public:
  /**
   * @brief Enumeration of supported battery/power supply types.
   * 
   * Defines the different power systems monitored by this module.
   * Used as indices for battery-specific data arrays and configuration.
   * 
   * @note Additional power supply types can be added before kNumberOfBatteries.
   */
  typedef enum {
    k36vBattery,           ///< 36V LIPO battery (10-cell, 30Ah) - monitored via analog pin
    // Future power supplies will be added here:
    // k24vSupply,         ///< 24V power supply (motor power) - monitored via INA226
    // k12vSupply,         ///< 12V power supply - monitored via INA226  
    // k5vSupply,          ///< 5V power supply - monitored via INA226
    // k3v3Supply,         ///< 3.3V power supply - monitored via INA226
    kNumberOfBatteries,    ///< Total number of monitored power systems
  } BatteryType;

  /**
   * @brief Battery/power supply monitoring configuration.
   * 
   * Contains all configuration parameters for a specific power system including
   * voltage/current thresholds, averaging parameters, and safety limits.
   */
  struct BatteryConfig {
    float nominal_voltage;              ///< Expected nominal voltage (V)
    float max_charge_voltage;           ///< Maximum safe voltage when charging (V) 
    float min_discharge_voltage;        ///< Minimum safe voltage when discharging (V)
    float overvoltage_multiplier;       ///< Multiplier for overvoltage detection (e.g., 1.25)
    float max_current;                  ///< Maximum safe current (A)
    float overcurrent_time_threshold;   ///< Time threshold for overcurrent alarm (seconds)
    uint8_t consecutive_alarm_count;    ///< Number of consecutive readings for alarm trigger
    float voltage_alpha;                ///< Exponential averaging factor for voltage (0.0-1.0)
    float current_alpha;                ///< Exponential averaging factor for current (0.0-1.0)
    bool is_lipo_battery;               ///< True if LIPO battery (analog), false if power supply (INA226)
    bool monitor_motor_current;         ///< True if this supply powers motors (critical monitoring)
  };

  /**
   * @brief Per-battery alarm state tracking.
   * 
   * Maintains alarm conditions and consecutive reading counts for each
   * monitored power system.
   */
  struct BatteryAlarmState {
    bool overvoltage_alarm;             ///< True if overvoltage condition detected
    bool overcurrent_alarm;             ///< True if overcurrent condition detected
    bool is_unsafe;                     ///< True if battery is in unsafe state (non-resettable)
    uint8_t consecutive_overvoltage;    ///< Count of consecutive overvoltage readings
    uint8_t consecutive_overcurrent;    ///< Count of consecutive overcurrent readings
    unsigned long overcurrent_start_time; ///< Timestamp when overcurrent started (ms)
  };

  /**
   * @brief Gets the singleton instance of BatteryModule.
   * 
   * @return Reference to the singleton BatteryModule instance.
   */
  static BatteryModule& singleton();

  /**
   * @brief Gets the exponentially averaged current reading for a specific battery.
   * 
   * Returns the exponentially averaged current from recent readings.
   * For LIPO battery, returns 0.0f (current not monitored via analog).
   * 
   * @param battery The battery type to read current from.
   * @return Averaged current in amperes, or 0.0f if not available.
   */
  float getCurrent(BatteryType battery) const;

  /**
   * @brief Gets the exponentially averaged voltage reading for a specific battery.
   * 
   * Returns the exponentially averaged voltage from recent readings.
   * Uses analog reading for LIPO battery, INA226 for power supplies.
   * 
   * @param battery The battery type to read voltage from.
   * @return Averaged voltage in volts, or 0.0f if battery index is invalid.
   */
  float getVoltage(BatteryType battery) const;

  /**
   * @brief Checks if a specific battery is in an unsafe state.
   * 
   * Returns the per-battery unsafe state which is set when alarm conditions
   * are met and is not automatically resettable.
   * 
   * @param battery The battery type to check.
   * @return true if the specified battery is in an unsafe state.
   */
  bool isUnsafe(BatteryType battery) const;

  /**
   * @brief Gets the alarm state for a specific battery.
   * 
   * Returns the complete alarm state structure containing all alarm flags
   * and consecutive reading counts for the specified battery.
   * 
   * @param battery The battery type to get alarm state for.
   * @return Reference to the battery's alarm state structure.
   */
  const BatteryAlarmState& getAlarmState(BatteryType battery) const;

 protected:
  /**
   * @brief Main processing loop for battery monitoring.
   * 
   * Periodically reads voltage/current from all batteries, performs exponential
   * averaging, checks for outliers and alarm conditions, and sends status reports.
   * Called by the base Module class at regular intervals.
   */
  void loop() override;

  /**
   * @brief Gets the module name for identification.
   * 
   * @return Module name string "MBAT".
   */
  const char* name() override { return "MBAT"; }

  /**
   * @brief Initializes the battery monitoring hardware.
   * 
   * Sets up analog pins, I2C communication, multiplexer, and INA226 sensors.
   * Initializes battery configurations and alarm states.
   */
  void setup() override;

 private:
  /**
   * @brief Private constructor for singleton pattern.
   * 
   * Initializes battery configurations, alarm states, and diagnostic messaging.
   */
  BatteryModule();

  /**
   * @brief Initializes battery/power supply configurations.
   * 
   * Sets up monitoring parameters for each power system including voltage thresholds,
   * current limits, averaging factors, and safety parameters.
   */
  void initializeBatteryConfigs();

  /**
   * @brief Calculates battery state of charge percentage from voltage.
   * 
   * Uses linear interpolation between minimum and maximum battery voltages
   * to estimate the state of charge percentage. Only applicable to LIPO battery.
   * 
   * @param battery The battery type (should be LIPO).
   * @param voltage The battery voltage in volts.
   * @return Estimated state of charge as percentage (0.0-100.0).
   */
  float calculatePercentage(BatteryType battery, float voltage);

  /**
   * @brief Checks if any battery system is in an unsafe state.
   * 
   * Aggregates unsafe conditions from all monitored batteries and power supplies.
   * This is the global safety check used by the base Module class.
   * 
   * @return true if any battery/power supply is in an unsafe state.
   */
  bool isUnsafe() override;

  /**
   * @brief Processes safety checks for a specific battery.
   * 
   * Evaluates voltage and current readings against safety thresholds,
   * updates alarm states, and sets unsafe flags when conditions are met.
   * 
   * @param battery The battery type to check.
   * @param voltage Current voltage reading.
   * @param current Current reading (0.0 for LIPO battery).
   */
  void processSafetyChecks(BatteryType battery, float voltage, float current);

  /**
   * @brief Resets resettable safety warning flags.
   * 
   * Clears alarm flags that can be reset, but preserves unsafe states
   * which require manual intervention or system restart.
   */
  void resetSafetyFlags() override;

  /**
   * @brief Selects a specific battery sensor via I2C multiplexer.
   * 
   * Configures the I2C multiplexer to route communication to the INA226
   * sensor associated with the specified power supply.
   * 
   * @param battery The battery type to select for communication.
   */
  void selectSensor(BatteryType battery) const;

  /**
   * @brief Sends formatted battery status message.
   * 
   * Constructs and transmits a diagnostic message containing voltage,
   * percentage, current, and alarm status for the specified battery.
   * 
   * @param battery The battery type to report.
   */
  void sendBatteryState(BatteryType battery);

  /**
   * @brief Tests I2C multiplexer connectivity.
   * 
   * Attempts communication with the I2C multiplexer to verify it's present
   * and responsive before attempting to configure power supply sensors.
   * 
   * @return true if multiplexer responds successfully.
   */
  bool testI2CMultiplexer();

  /**
   * @brief Updates exponential average with new reading.
   * 
   * Applies exponential moving average: new_avg = alpha * new_value + (1-alpha) * old_avg
   * 
   * @param current_average Current averaged value.
   * @param new_reading New sensor reading.
   * @param alpha Averaging factor (0.0-1.0, higher = more responsive).
   * @return Updated averaged value.
   */
  float updateExponentialAverage(float current_average, float new_reading, float alpha);

  // Static configuration constants
  static const uint8_t kNumberReadingsToAverage_ = 10;                      ///< Legacy: kept for compatibility
  static const uint8_t INA226_ADDRESS = 0x40;                              ///< Default I2C address for INA226 sensors
  static const uint8_t kI2CMultiplexorEnablePin = 8;                       ///< GPIO pin to enable I2C multiplexer power
  static constexpr float k36VAnalogToVoltageConversion = 0.087393162f;      ///< Conversion factor for 36V battery analog reading
  
  // Default exponential averaging factors
  static constexpr float kDefaultVoltageAlpha = 0.1f;                      ///< Default voltage averaging factor (10% new, 90% old)
  static constexpr float kDefaultCurrentAlpha = 0.2f;                      ///< Default current averaging factor (20% new, 80% old)

  // Per-battery configuration and state arrays
  static BatteryConfig g_battery_configs_[kNumberOfBatteries];             ///< Configuration parameters per battery
  static BatteryAlarmState g_alarm_states_[kNumberOfBatteries];            ///< Alarm state tracking per battery
  static float g_voltage_averages_[kNumberOfBatteries];                    ///< Exponentially averaged voltage per battery
  static float g_current_averages_[kNumberOfBatteries];                    ///< Exponentially averaged current per battery

  // Hardware interface arrays
  static INA226 g_ina226_[kNumberOfBatteries];                            ///< INA226 sensor instances for power supply monitoring
  static uint8_t gINA226_DeviceIndexes_[kNumberOfBatteries];              ///< I2C multiplexer channel mapping per battery

  // Legacy arrays (kept for compatibility during transition)
  static float g_averages_[kNumberOfBatteries][kNumberReadingsToAverage_];  ///< Legacy circular buffer (to be removed)
  static size_t g_next_average_index_[kNumberOfBatteries];                  ///< Legacy circular buffer indices (to be removed)

  // Singleton management
  static BatteryModule* g_instance_;  ///< Singleton instance pointer

  // Instance state variables
  bool multiplexer_available_;                                  ///< Flag indicating I2C multiplexer availability
  bool setup_completed_;                                        ///< Flag indicating successful hardware initialization
  unsigned long total_battery_readings_[kNumberOfBatteries];   ///< Total readings taken per battery
};
