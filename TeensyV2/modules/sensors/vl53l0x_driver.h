#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

class VL53L0XDriver {
public:
  enum RangeStatus {
    RANGE_VALID = 0,         // Same as STATUS_GOOD
    STATUS_GOOD = 0,
    STATUS_SIGMA_FAIL = 1,
    STATUS_SIGNAL_FAIL = 2,
    STATUS_MIN_RANGE_FAIL = 3,
    STATUS_PHASE_FAIL = 4,
    HW_FAIL = 5,             // Same as STATUS_HARDWARE_FAIL
    STATUS_HARDWARE_FAIL = 5,
    STATUS_NONE = 255
  };

  struct Reading {
    uint16_t mm = 0;
    RangeStatus status = RANGE_VALID;
    uint8_t raw_status = 0;
    uint32_t timestamp_us = 0;
    bool valid = false;
  };

  struct Counters {
    uint32_t total_reads = 0;
    uint32_t ok = 0;
    uint32_t signal_fail = 0;
    uint32_t sigma_fail = 0;
    uint32_t min_range_fail = 0;
    uint32_t phase_fail = 0;
    uint32_t hw_fail = 0;
    uint32_t none = 0;
    uint32_t timeouts = 0;
  };

  VL53L0XDriver();
  void setBus(TwoWire* w);
  void setAddress(uint8_t a);
  uint8_t address() const;

  bool initAndConfigure(uint32_t timingBudget_us, float signalRateLimit_Mcps);
  bool startContinuous(uint16_t period_ms = 0);
  void stop();
  bool readIfReady(Reading& out);
  bool testSingleMeasurement(Reading& out, uint32_t timeout_ms);
  void setIIRAlpha(float alpha);
  const Counters& counters() const;
  void resetCounters();
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);
  const char* lastError() const;
  uint32_t lastErrorDetail() const;

  static uint8_t decodeVcselPeriod(uint8_t reg_val);
  static uint32_t calcMacroPeriod(uint8_t vcsel_period_pclks);

  enum Reg : uint8_t {
    SYSRANGE_START = 0x00,
    SYSTEM_SEQUENCE_CONFIG = 0x01,
    SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,
    GPIO_HV_MUX_ACTIVE_HIGH = 0x84,
    SYSTEM_INTERRUPT_CLEAR = 0x0B,
    RESULT_INTERRUPT_STATUS = 0x13,
    RESULT_RANGE_STATUS = 0x14,
    I2C_SLAVE_DEVICE_ADDRESS = 0x8A,
    MSRC_CONFIG_CONTROL = 0x60,
    PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51,
    PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56,
    PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57,
    FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71,
    FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47,
    FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48,
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,
    GLOBAL_CONFIG_VCSEL_WIDTH = 0x32,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
    GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
    DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
    VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,
    IDENTIFICATION_MODEL_ID = 0xC0,
    OSC_CALIBRATE_VAL = 0xF8,
    SYSTEM_INTERMEASUREMENT_PERIOD = 0x04,
    MSRC_CONFIG_TIMEOUT_MACROP = 0x46
  };

private:
  TwoWire* bus_ = nullptr;
  uint8_t address_ = 0x29;
  uint32_t measurement_timing_budget_us_ = 0;
  float alpha_ = 1.0f;
  int32_t iir_mm_ = -1;
  Counters counters_;
  const char* last_error_ = nullptr;
  uint32_t last_error_detail_ = 0;
  VL53L0X sensor_;
};
