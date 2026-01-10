// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

#include "vl53l0x_driver.h"

#include <algorithm>

namespace {

  constexpr uint8_t kDefaultAddress = 0x29;
  constexpr uint8_t kInterruptReadyMask = 0x07;
  constexpr uint32_t kDefaultTimeoutMs = 200U;

  inline uint32_t nowMicros() {
    return micros();
  }

  inline float clamp01(float value) {
    if (value < 0.0f) {
      return 0.0f;
    }
    if (value > 1.0f) {
      return 1.0f;
    }
    return value;
  }

  VL53L0XDriver::RangeStatus mapRangeStatus(uint8_t raw_status) {
    const uint8_t code = raw_status & 0x1F;
    switch (code) {
    case 0:
      return VL53L0XDriver::RANGE_VALID;
    case 1:
      return VL53L0XDriver::STATUS_SIGMA_FAIL;
    case 2:
      return VL53L0XDriver::STATUS_SIGNAL_FAIL;
    case 3:
      return VL53L0XDriver::STATUS_MIN_RANGE_FAIL;
    case 4:
      return VL53L0XDriver::STATUS_PHASE_FAIL;
    case 5:
    case 6:
    case 7:
    case 8:
      return VL53L0XDriver::STATUS_HARDWARE_FAIL;
    default:
      return VL53L0XDriver::STATUS_NONE;
    }
  }

  void accumulateStatus(VL53L0XDriver::Counters& counters, VL53L0XDriver::RangeStatus status) {
    switch (status) {
    case VL53L0XDriver::RANGE_VALID:
      counters.ok++;
      break;
    case VL53L0XDriver::STATUS_SIGNAL_FAIL:
      counters.signal_fail++;
      break;
    case VL53L0XDriver::STATUS_SIGMA_FAIL:
      counters.sigma_fail++;
      break;
    case VL53L0XDriver::STATUS_MIN_RANGE_FAIL:
      counters.min_range_fail++;
      break;
    case VL53L0XDriver::STATUS_PHASE_FAIL:
      counters.phase_fail++;
      break;
    case VL53L0XDriver::STATUS_HARDWARE_FAIL:
      counters.hw_fail++;
      break;
    case VL53L0XDriver::STATUS_NONE:
    default:
      counters.none++;
      break;
    }
  }

  uint8_t decodeRawStatus(uint8_t status_register) {
    // ST API shifts RESULT_RANGE_STATUS (0x14) right by 3 before masking.
    // Bits [5:3] contain the range status in the hardware register.
    return static_cast<uint8_t>((status_register >> 3) & 0x1F);
  }

  bool statusAllowsMeasurement(VL53L0XDriver::RangeStatus status) {
    switch (status) {
    case VL53L0XDriver::RANGE_VALID:
    case VL53L0XDriver::STATUS_SIGMA_FAIL:
    case VL53L0XDriver::STATUS_SIGNAL_FAIL:
    case VL53L0XDriver::STATUS_MIN_RANGE_FAIL:
    case VL53L0XDriver::STATUS_PHASE_FAIL:
      return true;
    default:
      return false;
    }
  }

  const char* describeStatus(VL53L0XDriver::RangeStatus status) {
    switch (status) {
    case VL53L0XDriver::RANGE_VALID: return nullptr;
    case VL53L0XDriver::STATUS_SIGMA_FAIL: return "sigma_fail";
    case VL53L0XDriver::STATUS_SIGNAL_FAIL: return "signal_fail";
    case VL53L0XDriver::STATUS_MIN_RANGE_FAIL: return "min_range_fail";
    case VL53L0XDriver::STATUS_PHASE_FAIL: return "phase_fail";
    case VL53L0XDriver::STATUS_HARDWARE_FAIL: return "hardware_fail";
    case VL53L0XDriver::STATUS_NONE: return "status_none";
    default: return "status_unknown";
    }
  }

}  // namespace

VL53L0XDriver::VL53L0XDriver()
  : measurement_timing_budget_us_(0),
  alpha_(1.0f),
  iir_mm_(-1),
  last_error_(nullptr),
  last_error_detail_(0U) {
  sensor_.setAddress(kDefaultAddress);
}

void VL53L0XDriver::setBus(TwoWire* w) {
  bus_ = w;
  sensor_.setBus(bus_);
}

void VL53L0XDriver::setAddress(uint8_t a) {
  address_ = a;
  sensor_.setAddress(address_);
}

uint8_t VL53L0XDriver::address() const {
  return address_;
}

bool VL53L0XDriver::initAndConfigure(uint32_t timingBudget_us, float signalRateLimit_Mcps) {
  if (!bus_) {
    last_error_ = "no_bus";
    last_error_detail_ = 0U;
    return false;
  }

  sensor_.setBus(bus_);
  sensor_.setAddress(address_);

  if (!sensor_.init(true)) {
    last_error_ = "init_failed";
    last_error_detail_ = 0U;
    return false;
  }

  if (!sensor_.setSignalRateLimit(signalRateLimit_Mcps)) {
    last_error_ = "signal_rate_limit";
    last_error_detail_ = static_cast<uint32_t>(signalRateLimit_Mcps * 1000.0f);
    return false;
  }

  if (!sensor_.setMeasurementTimingBudget(timingBudget_us)) {
    last_error_ = "timing_budget";
    last_error_detail_ = timingBudget_us;
    return false;
  }

  measurement_timing_budget_us_ = timingBudget_us;

  const uint32_t timeout_ms = std::max<uint32_t>(timingBudget_us / 1000U + 5U, kDefaultTimeoutMs);
  sensor_.setTimeout(static_cast<uint16_t>(timeout_ms));

  resetCounters();
  alpha_ = clamp01(alpha_);
  iir_mm_ = -1;
  last_error_ = nullptr;
  last_error_detail_ = 0U;
  return true;
}

bool VL53L0XDriver::startContinuous(uint16_t period_ms) {
  sensor_.startContinuous(period_ms);
  return true;
}

void VL53L0XDriver::stop() {
  sensor_.stopContinuous();
}

bool VL53L0XDriver::readIfReady(Reading& out) {
  out.valid = false;
  out.mm = 0U;
  out.status = STATUS_NONE;
  out.raw_status = 0U;
  out.timestamp_us = nowMicros();

  // CRITICAL: Check if data is ready WITHOUT blocking.
  // The Arduino library's readRangeContinuousMillimeters() BLOCKS waiting for data,
  // which breaks the non-blocking loop() architecture. We must check the interrupt
  // status first and only proceed if data is available.
  const uint8_t interrupt_status = sensor_.readReg(RESULT_INTERRUPT_STATUS);
  if ((interrupt_status & kInterruptReadyMask) == 0U) {
    // No new measurement available yet - return immediately without blocking
    return false;
  }

  // Data is ready - read the range value directly from the sensor register
  // This is what the Arduino library does internally (RESULT_RANGE_STATUS + 10)
  const uint16_t distance_mm = sensor_.readReg16Bit(RESULT_RANGE_STATUS + 10);

  // Read the status register to get error information
  const uint8_t status_register = sensor_.readReg(RESULT_RANGE_STATUS);
  const uint8_t raw_status = decodeRawStatus(status_register);

  // Clear the interrupt so the sensor can start the next measurement
  sensor_.writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  // Decode the status
  RangeStatus status = mapRangeStatus(raw_status);

  // Recognize common VL53L0X out-of-range sentinel values
  const bool is_out_of_range = (distance_mm == 8191U) || (distance_mm == 8190U);
  bool measurement_valid = true;

  if (is_out_of_range) {
    // Out-of-range: sensor worked but target is beyond max range
    // Treat as valid but degraded measurement
    status = RANGE_VALID;
    measurement_valid = true;
  }
  else if (distance_mm == 0xFFFFU) {
    // Explicit error sentinel
    status = STATUS_HARDWARE_FAIL;
    measurement_valid = false;
  }

  counters_.total_reads++;
  accumulateStatus(counters_, status);

  out.mm = distance_mm;
  out.status = status;
  out.raw_status = raw_status;
  out.timestamp_us = nowMicros();
  out.valid = measurement_valid;

  if (!measurement_valid) {
    last_error_ = describeStatus(status);
    last_error_detail_ = distance_mm;
    return true; // measurement was read but not valid
  }

  last_error_ = nullptr;
  last_error_detail_ = 0U;
  return true;
}

bool VL53L0XDriver::testSingleMeasurement(Reading& out, uint32_t timeout_ms) {
  const uint16_t previous_timeout = sensor_.getTimeout();
  sensor_.setTimeout(static_cast<uint16_t>(timeout_ms));

  out.valid = false;
  out.mm = 0U;
  out.status = STATUS_NONE;
  out.raw_status = 0U;
  out.timestamp_us = nowMicros();

  const uint16_t distance = sensor_.readRangeSingleMillimeters();
  const bool timeout = sensor_.timeoutOccurred();
  const uint8_t status_register = sensor_.readReg(RESULT_RANGE_STATUS);
  const uint8_t raw_status = decodeRawStatus(status_register);
  RangeStatus status = mapRangeStatus(raw_status);

  counters_.total_reads++;

  if (timeout) {
    counters_.timeouts++;
    last_error_ = "timeout";
    last_error_detail_ = timeout_ms;
    sensor_.setTimeout(previous_timeout);
    return false;
  }

  const bool measurement_valid = (distance != 0xFFFFU) && statusAllowsMeasurement(status);

  accumulateStatus(counters_, status);

  out.mm = distance;
  out.status = status;
  out.raw_status = raw_status;
  out.timestamp_us = nowMicros();
  out.valid = measurement_valid;

  sensor_.setTimeout(previous_timeout);

  if (!measurement_valid) {
    last_error_ = describeStatus(status);
    last_error_detail_ = raw_status;
    return false;
  }

  last_error_ = nullptr;
  last_error_detail_ = 0U;
  return true;
}

void VL53L0XDriver::setIIRAlpha(float alpha) {
  alpha_ = clamp01(alpha);
  if (alpha_ >= 0.999f) {
    alpha_ = 1.0f;
    iir_mm_ = -1;
  }
}

const VL53L0XDriver::Counters& VL53L0XDriver::counters() const {
  return counters_;
}

void VL53L0XDriver::resetCounters() {
  counters_ = Counters{};
}

uint8_t VL53L0XDriver::readRegister(uint8_t reg) {
  return sensor_.readReg(reg);
}

void VL53L0XDriver::writeRegister(uint8_t reg, uint8_t value) {
  sensor_.writeReg(reg, value);
}

const char* VL53L0XDriver::lastError() const {
  return last_error_;
}

uint32_t VL53L0XDriver::lastErrorDetail() const {
  return last_error_detail_;
}

uint8_t VL53L0XDriver::decodeVcselPeriod(uint8_t reg_val) {
  return static_cast<uint8_t>(((reg_val)+1U) << 1U);
}

uint32_t VL53L0XDriver::calcMacroPeriod(uint8_t vcsel_period_pclks) {
  return (((uint32_t)2304 * static_cast<uint32_t>(vcsel_period_pclks) * 1655U) + 500U) / 1000U;
}
