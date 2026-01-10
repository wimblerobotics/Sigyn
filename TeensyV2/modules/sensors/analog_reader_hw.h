// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file analog_reader_hw.h
 * @brief Hardware implementation of analog reader interface
 *
 * Provides real hardware ADC reading using Arduino/Teensy SDK.
 * This is the production implementation used on actual hardware.
 *
 * @author Wimble Robotics
 * @date 2026
 */

#pragma once

#ifdef UNIT_TEST
#include "Arduino.h"
#else
#include <Arduino.h>
#endif

#include "interfaces/i_analog_reader.h"

namespace sigyn_teensy {

/**
 * @brief Hardware implementation using real Teensy ADC
 *
 * This implementation calls the actual Arduino ADC functions.
 * Used in production firmware running on Teensy 4.1.
 */
class AnalogReaderHW : public IAnalogReader {
public:
  AnalogReaderHW() = default;
  ~AnalogReaderHW() override = default;

  uint16_t readAnalog(uint8_t pin) override {
    return analogRead(pin);
  }

  void setAnalogResolution(uint8_t bits) override {
    analogReadResolution(bits);
  }
};

} // namespace sigyn_teensy
