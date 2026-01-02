// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file i_analog_reader.h
 * @brief Interface for analog sensor reading abstraction
 *
 * Provides a testable interface for reading analog values from hardware.
 * This abstraction allows dependency injection for unit testing while
 * maintaining the real hardware implementation for production use.
 *
 * Design Philosophy:
 * - Interface at the hardware boundary (ADC reads)
 * - Preserves all business logic (conversion, filtering, validation)
 * - No heap allocation in production implementations
 * - Simple, focused interface
 *
 * @author Wimble Robotics
 * @date 2026
 */

#pragma once

#include <cstdint>

namespace sigyn_teensy {

/**
 * @brief Pure virtual interface for analog sensor reading
 *
 * This interface abstracts the hardware-specific analog reading operations,
 * enabling dependency injection for testing. Production code uses real ADC
 * hardware, test code can inject controlled values.
 */
class IAnalogReader {
public:
  virtual ~IAnalogReader() = default;

  /**
   * @brief Read raw analog value from specified pin
   *
   * @param pin Analog pin number to read
   * @return Raw ADC value (0-4095 for 12-bit resolution)
   *
   * Production: Calls Arduino analogRead()
   * Testing: Returns injected test value
   */
  virtual uint16_t readAnalog(uint8_t pin) = 0;

  /**
   * @brief Set analog read resolution
   *
   * @param bits Resolution in bits (typically 10-12 for Teensy)
   *
   * Production: Calls Arduino analogReadResolution()
   * Testing: No-op or stores for validation
   */
  virtual void setAnalogResolution(uint8_t bits) = 0;
};

} // namespace sigyn_teensy
