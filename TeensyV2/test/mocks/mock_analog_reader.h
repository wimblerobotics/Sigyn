// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file mock_analog_reader.h
 * @brief Mock analog reader for unit testing
 *
 * Provides controllable analog readings for testing temperature monitor
 * and other analog sensor code without hardware dependencies.
 *
 * @author Wimble Robotics
 * @date 2026
 */

#pragma once

#include <map>
#include <cstdint>
#include "modules/sensors/interfaces/i_analog_reader.h"

namespace sigyn_teensy {

/**
 * @brief Mock analog reader for testing
 *
 * Allows tests to inject specific ADC values for each pin.
 * Can simulate sensor readings, failures, noise, etc.
 */
class MockAnalogReader : public IAnalogReader {
public:
  MockAnalogReader() = default;
  ~MockAnalogReader() override = default;

  /**
   * @brief Set the ADC value for a specific pin
   * @param pin Analog pin number
   * @param value ADC value to return (0-4095 for 12-bit)
   */
  void setAnalogValue(uint8_t pin, uint16_t value) {
    pin_values_[pin] = value;
  }

  /**
   * @brief Set temperature for a pin (automatically converts to ADC value)
   * @param pin Analog pin number
   * @param temp_celsius Temperature in Celsius
   *
   * Converts temperature to equivalent TMP36 ADC reading.
   * TMP36: 500mV at 0°C, 10mV/°C
   * Must match temperature_monitor.cpp conversion exactly:
   * voltage_mv = (raw_value * 3.3 * 1000.0) / 4096
   * temp_c = (voltage_mv - 500.0) / 10.0
   */
  void setTemperature(uint8_t pin, float temp_celsius) {
    // TMP36: Voltage = (temp * 10mV) + 500mV
    float voltage_mv = (temp_celsius * 10.0f) + 500.0f;
    
    // Convert to ADC counts (12-bit, 3.3V reference)
    // Reverse of: voltage_mv = (raw_value * 3.3 * 1000.0) / 4096
    // raw_value = (voltage_mv * 4096) / (3.3 * 1000.0)
    uint16_t adc_value = static_cast<uint16_t>((voltage_mv * 4096.0f) / (3.3f * 1000.0f));
    
    pin_values_[pin] = adc_value;
  }

  /**
   * @brief Clear all pin values (return 0 for unset pins)
   */
  void reset() {
    pin_values_.clear();
    resolution_bits_ = 12;
  }

  /**
   * @brief Get the resolution setting
   */
  uint8_t getResolution() const {
    return resolution_bits_;
  }

  // IAnalogReader interface implementation
  uint16_t readAnalog(uint8_t pin) override {
    if (pin_values_.find(pin) != pin_values_.end()) {
      return pin_values_[pin];
    }
    return 0;  // Unset pins return 0
  }

  void setAnalogResolution(uint8_t bits) override {
    resolution_bits_ = bits;
  }

private:
  std::map<uint8_t, uint16_t> pin_values_;
  uint8_t resolution_bits_ = 12;
};

} // namespace sigyn_teensy
