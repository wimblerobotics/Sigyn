// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file Arduino.h (Mock)
 * @brief Mock Arduino API for PC-based unit testing
 *
 * Provides minimal Arduino API surface needed to compile and test
 * TeensyV2 code on a PC without actual hardware. This is NOT a full
 * Arduino simulator - it's just enough to make the code compile and
 * allow unit tests to control time and I/O.
 *
 * Key Features:
 * - Controllable time (millis, micros, delay)
 * - Stubbed digital I/O
 * - Stubbed analog I/O with injectable values
 * - Basic String class (will use std::string under the hood)
 *
 * Usage:
 * This header is only included when UNIT_TEST is defined.
 * Production builds use the real Arduino.h from Teensy SDK.
 */

#pragma once

#include <cstdint>
#include <cmath>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <string>
#include <map>

// Arduino basic types
typedef bool boolean;
typedef uint8_t byte;

// Pin modes
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

// Digital pin states
#define LOW 0
#define HIGH 1

// Analog reference modes
#define DEFAULT 0
#define INTERNAL 1
#define EXTERNAL 2

// Print base specifiers (Arduino compatibility)
#ifndef DEC
#define DEC 10
#endif
#ifndef HEX
#define HEX 16
#endif
#ifndef OCT
#define OCT 8
#endif
#ifndef BIN
#define BIN 2
#endif

// Math constants (if not already defined)
#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#ifndef HALF_PI
#define HALF_PI 1.5707963267948966192313216916398
#endif

#ifndef TWO_PI
#define TWO_PI 6.283185307179586476925286766559
#endif

// Analog resolution (Teensy 4.1 specific)
static constexpr int ADC_RESOLUTION = 12;

// Analog pin definitions (Teensy 4.1)
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 21
#define A8 22
#define A9 23

namespace arduino_mock {
  // Global mock state
  extern uint32_t current_millis;
  extern uint64_t current_micros;
  extern std::map<int, int> digital_pins;
  extern std::map<int, int> analog_values;
  extern std::map<int, int> pin_modes;
  
  // Control functions for tests
  void setMillis(uint32_t ms);
  void setMicros(uint64_t us);
  void setAnalogValue(int pin, int value);
  void setDigitalValue(int pin, int value);
  void reset();
}

// Arduino timing functions
inline uint32_t millis() {
  return arduino_mock::current_millis;
}

inline uint64_t micros() {
  return arduino_mock::current_micros;
}

inline void delay(uint32_t ms) {
  arduino_mock::current_millis += ms;
  arduino_mock::current_micros += (ms * 1000ULL);
}

inline void delayMicroseconds(uint32_t us) {
  arduino_mock::current_micros += us;
  if (us >= 1000) {
    arduino_mock::current_millis += (us / 1000);
  }
}

// Arduino digital I/O
inline void pinMode(int pin, int mode) {
  arduino_mock::pin_modes[pin] = mode;
}

inline void digitalWrite(int pin, int value) {
  arduino_mock::digital_pins[pin] = value;
}

inline int digitalRead(int pin) {
  if (arduino_mock::digital_pins.find(pin) != arduino_mock::digital_pins.end()) {
    return arduino_mock::digital_pins[pin];
  }
  return LOW;
}

// Arduino analog I/O
inline int analogRead(int pin) {
  if (arduino_mock::analog_values.find(pin) != arduino_mock::analog_values.end()) {
    return arduino_mock::analog_values[pin];
  }
  return 0;
}

inline void analogWrite(int pin, int value) {
  arduino_mock::analog_values[pin] = value;
}

inline void analogReference(int mode) {
  (void)mode;  // Ignored in mock
}

inline void analogReadResolution(int bits) {
  (void)bits;  // Ignored in mock
}

inline void analogWriteResolution(int bits) {
  (void)bits;  // Ignored in mock
}

// Math functions (Arduino-specific wrappers)
inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline long constrain(long x, long a, long b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

// Import std::isnan into global scope (Arduino compatibility)
using std::isnan;

// Arduino String class (minimal implementation using std::string)
class String {
public:
  String() : data_() {}
  String(const char* str) : data_(str ? str : "") {}
  String(const std::string& str) : data_(str) {}
  String(const String&) = default;
  String(String&&) noexcept = default;
  String(int val) : data_(std::to_string(val)) {}
  String(unsigned int val) : data_(std::to_string(val)) {}
  String(long val) : data_(std::to_string(val)) {}
  String(unsigned long val) : data_(std::to_string(val)) {}
  String(int32_t val, int base) { data_ = formatInteger(static_cast<unsigned long>(static_cast<uint32_t>(val)), base); }
  String(uint32_t val, int base) { data_ = formatInteger(static_cast<unsigned long>(val), base); }
  String(long val, int base) { data_ = formatInteger(static_cast<unsigned long>(val), base); }
  String(unsigned long val, int base) { data_ = formatInteger(val, base); }
  String(float val, int decimals = 2) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%.*f", decimals, val);
    data_ = buffer;
  }
  String(double val, int decimals = 2) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%.*f", decimals, val);
    data_ = buffer;
  }
  
  const char* c_str() const { return data_.c_str(); }
  size_t length() const { return data_.length(); }
  
  String& operator=(const String& other) {
    data_ = other.data_;
    return *this;
  }

  String& operator=(String&& other) noexcept {
    data_ = std::move(other.data_);
    return *this;
  }
  
  String& operator=(const char* str) {
    data_ = str ? str : "";
    return *this;
  }
  
  String operator+(const String& other) const {
    return String(data_ + other.data_);
  }
  
  String operator+(const char* str) const {
    return String(data_ + (str ? str : ""));
  }
  
  // Allow: const char* + String
  friend String operator+(const char* lhs, const String& rhs) {
    std::string result = lhs ? lhs : "";
    result += rhs.data_;
    return String(result);
  }
  
  String& operator+=(const String& other) {
    data_ += other.data_;
    return *this;
  }
  
  String& operator+=(const char* str) {
    if (str) data_ += str;
    return *this;
  }
  
  bool operator==(const String& other) const {
    return data_ == other.data_;
  }
  
  bool operator!=(const String& other) const {
    return data_ != other.data_;
  }
  
  // String search/manipulation methods
  int indexOf(char c) const {
    size_t pos = data_.find(c);
    return (pos == std::string::npos) ? -1 : static_cast<int>(pos);
  }

  int indexOf(char c, int fromIndex) const {
    if (fromIndex < 0) fromIndex = 0;
    size_t pos = data_.find(c, static_cast<size_t>(fromIndex));
    return (pos == std::string::npos) ? -1 : static_cast<int>(pos);
  }
  
  int indexOf(const char* str) const {
    if (!str) return -1;
    size_t pos = data_.find(str);
    return (pos == std::string::npos) ? -1 : static_cast<int>(pos);
  }

  int indexOf(const char* str, int fromIndex) const {
    if (!str) return -1;
    if (fromIndex < 0) fromIndex = 0;
    size_t pos = data_.find(str, static_cast<size_t>(fromIndex));
    return (pos == std::string::npos) ? -1 : static_cast<int>(pos);
  }
  
  String substring(int start) const {
    if (start < 0 || static_cast<size_t>(start) >= data_.length()) {
      return String();
    }
    return String(data_.substr(start));
  }
  
  String substring(int start, int end) const {
    if (start < 0 || static_cast<size_t>(start) >= data_.length()) {
      return String();
    }
    if (end <= start) return String();
    return String(data_.substr(start, end - start));
  }

  float toFloat() const {
    // Arduino behavior: returns 0.0 if no valid conversion.
    char* endptr = nullptr;
    const char* s = data_.c_str();
    float v = std::strtof(s, &endptr);
    if (endptr == s) return 0.0f;
    return v;
  }
  
private:
  static std::string formatInteger(unsigned long val, int base) {
    if (base == DEC) return std::to_string(val);
    if (base == HEX) {
      std::ostringstream oss;
      oss << std::uppercase << std::hex << val;
      return oss.str();
    }
    if (base == OCT) {
      std::ostringstream oss;
      oss << std::oct << val;
      return oss.str();
    }
    if (base == BIN) {
      std::string out;
      do {
        out.insert(out.begin(), (val & 1UL) ? '1' : '0');
        val >>= 1U;
      } while (val != 0);
      return out;
    }
    // Generic base (2..36)
    if (base < 2) base = 2;
    if (base > 36) base = 36;
    const char* digits = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    std::string out;
    do {
      out.insert(out.begin(), digits[val % static_cast<unsigned long>(base)]);
      val /= static_cast<unsigned long>(base);
    } while (val != 0);
    return out;
  }

  std::string data_;
};

// Serial classes (minimal stub)
class HardwareSerial {
public:
  void begin(unsigned long baud) { (void)baud; }
  void end() {}
  operator bool() const { return true; }
  size_t print(const char* str) {
    return str ? std::strlen(str) : 0;
  }
  size_t write(uint8_t c) { (void)c; return 1; }
  size_t write(const uint8_t* buffer, size_t size) { (void)buffer; (void)size; return size; }
  int available() { return 0; }
  int read() { return -1; }
  void flush() {}
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;

// Wire (I2C) stub
class TwoWire {
public:
  void begin() {}
  void begin(uint8_t addr) { (void)addr; }
  void setClock(uint32_t freq) { (void)freq; }
  void beginTransmission(uint8_t addr) { (void)addr; }
  uint8_t endTransmission(bool stop = true) { (void)stop; return 0; }
  size_t write(uint8_t data) { (void)data; return 1; }
  uint8_t requestFrom(uint8_t addr, uint8_t quantity, bool stop = true) {
    (void)addr; (void)quantity; (void)stop; return 0;
  }
  int available() { return 0; }
  int read() { return -1; }
};

extern TwoWire Wire;
extern TwoWire Wire1;
extern TwoWire Wire2;
