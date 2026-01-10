// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file Arduino.cpp (Mock)
 * @brief Implementation of mock Arduino API for unit testing
 */

#include "Arduino.h"

namespace arduino_mock {
  uint32_t current_millis = 0;
  uint64_t current_micros = 0;
  std::map<int, int> digital_pins;
  std::map<int, int> analog_values;
  std::map<int, int> pin_modes;
  
  void setMillis(uint32_t ms) {
    current_millis = ms;
    current_micros = static_cast<uint64_t>(ms) * 1000ULL;
  }
  
  void setMicros(uint64_t us) {
    current_micros = us;
    current_millis = static_cast<uint32_t>(us / 1000ULL);
  }
  
  void setAnalogValue(int pin, int value) {
    analog_values[pin] = value;
  }
  
  void setDigitalValue(int pin, int value) {
    digital_pins[pin] = value;
  }
  
  void reset() {
    current_millis = 0;
    current_micros = 0;
    digital_pins.clear();
    analog_values.clear();
    pin_modes.clear();
  }
}

// Global mock hardware instances
HardwareSerial Serial;
HardwareSerial Serial1;
HardwareSerial Serial2;

TwoWire Wire;
TwoWire Wire1;
TwoWire Wire2;
