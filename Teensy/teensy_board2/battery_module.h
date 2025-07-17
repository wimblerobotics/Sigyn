#pragma once

#include <stdint.h>

#include "INA226.h"  // For INA226 battery monitoring.
#include "config.h"
#include "module.h"

/**
 * @brief Battery monitoring module for Teensy board 2.
 * 
 * This module provides comprehensive battery monitoring capabilities using both
 * analog voltage readings and INA226 current/voltage sensors. It supports
 * multiple batteries with independent monitoring, averaging, and safety checks.
 * 
 * Key features:
 * - Multi-battery support with per-battery averaging
 * - Hybrid monitoring: analog pins for main battery, INA226 for additional batteries
 * - I2C multiplexer support for multiple INA226 sensors
 * - Safety monitoring with configurable thresholds
 * - Periodic reporting and diagnostic messaging
 * 
 * @note This implementation is specific to Teensy board 2 hardware configuration.
 */
class BatteryModule : public Module {
 public:
  /**
   * @brief Enumeration of supported battery types.
   * 
   * Defines the different battery configurations supported by this module.
   * Used as indices for battery-specific data arrays.
   */
  typedef enum {
    k36vBattery,           ///< 36V battery configuration
    kNumberOfBatteries,    ///< Total number of batteries (used for array sizing)
  } BatteryType;

  /**
   * @brief Gets the singleton instance of BatteryModule.
   * 
   * @return Reference to the singleton BatteryModule instance.
   */
  static BatteryModule& singleton();

  /**
   * @brief Gets the current reading for a specific battery.
   * 
   * Reads current from the INA226 sensor associated with the specified battery.
   * Automatically selects the correct sensor via I2C multiplexer.
   * 
   * @param battery The battery type to read current from.
   * @return Current in amperes, or 0.0f if battery index is invalid.
   */
  float getCurrent(BatteryType battery) const;

  /**
   * @brief Gets the averaged voltage reading for a specific battery.
   * 
   * Returns the averaged voltage from the circular buffer of recent readings
   * for the specified battery. Uses different reading methods per battery type.
   * 
   * @param battery The battery type to read voltage from.
   * @return Averaged voltage in volts, or 0.0f if battery index is invalid.
   */
  float getVoltage(BatteryType battery) const;

 protected:
  /**
   * @brief Main processing loop for battery monitoring.
   * 
   * Periodically reads voltage/current from all batteries and sends status reports.
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
   * Configures all necessary hardware for battery monitoring operations.
   */
  void setup() override;

 private:
  /**
   * @brief Private constructor for singleton pattern.
   * 
   * Initializes all battery averaging arrays, indices, and diagnostic messaging.
   */
  BatteryModule();

  /**
   * @brief Calculates battery percentage from voltage.
   * 
   * Uses linear interpolation between minimum and maximum battery voltages
   * to estimate the state of charge percentage.
   * 
   * @param voltage The battery voltage in volts.
   * @return Estimated battery percentage (0-100).
   */
  float calculatePercentage(float voltage);

  /**
   * @brief Checks if any battery is in an unsafe state.
   * 
   * Evaluates all batteries against critical voltage thresholds to determine
   * if emergency action is required.
   * 
   * @return true if any battery is below critical voltage threshold.
   */
  bool isUnsafe() override;

  /**
   * @brief Resets safety warning flags.
   * 
   * Clears any safety-related warning states. Called when conditions improve.
   */
  void resetSafetyFlags() override;

  /**
   * @brief Selects a specific battery sensor via I2C multiplexer.
   * 
   * Configures the I2C multiplexer to route communication to the INA226
   * sensor associated with the specified battery.
   * 
   * @param battery The battery type to select for communication.
   */
  void selectSensor(BatteryType battery) const;

  /**
   * @brief Sends formatted battery status message.
   * 
   * Constructs and transmits a diagnostic message containing voltage,
   * percentage, and current readings for the specified battery.
   * 
   * @param device_index The battery index to report (0-based).
   */
  void sendBatteryState(uint8_t device_index);

  /**
   * @brief Tests I2C multiplexer connectivity.
   * 
   * Attempts communication with the I2C multiplexer to verify it's present
   * and responsive before attempting to configure battery sensors.
   * 
   * @return true if multiplexer responds successfully.
   */
  bool testI2CMultiplexer();

  // Static configuration constants
  static const uint8_t kNumberReadingsToAverage_ = 10;           ///< Number of readings to average per battery
  static const uint8_t INA226_ADDRESS = 0x40;                   ///< Default I2C address for INA226 sensors
  static const uint8_t kI2CMultiplexorEnablePin = 8;            ///< GPIO pin to enable I2C multiplexer power
  static constexpr float k36VAnalogToVoltageConversion = 0.087393162f;  ///< Conversion factor for 36V battery analog reading

  // Per-battery data arrays
  static float g_averages_[kNumberOfBatteries][kNumberReadingsToAverage_];  ///< Circular buffer of voltage readings per battery
  static size_t g_next_average_index_[kNumberOfBatteries];                  ///< Current write index for each battery's circular buffer

  // Hardware interface arrays
  static INA226 g_ina226_[kNumberOfBatteries];                            ///< INA226 sensor instances for battery monitoring
  static uint8_t gINA226_DeviceIndexes_[kNumberOfBatteries];              ///< I2C multiplexer channel mapping for each battery

  // Hardware configuration
  static BatteryModule* g_instance_;  ///< Singleton instance pointer

  // Instance state variables
  bool multiplexer_available_;                                  ///< Flag indicating I2C multiplexer availability
  bool setup_completed_;                                        ///< Flag indicating successful hardware initialization
  unsigned long total_battery_readings_[kNumberOfBatteries];   ///< Total readings taken per battery for averaging calculation
};
