// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

#pragma once

#include <cstdint>

namespace sigyn_teensy {

/**
 * Minimal RoboClaw interface for host-based unit testing.
 *
 * This intentionally includes only the subset of RoboClaw APIs that are
 * exercised by `roboclaw_monitor.cpp`.
 */
class IRoboClaw {
 public:
  virtual ~IRoboClaw() = default;

  virtual void begin(long speed) = 0;

  virtual bool ResetEncoders(uint8_t address) = 0;
  virtual uint32_t ReadError(uint8_t address, bool* valid = nullptr) = 0;

  virtual bool SetM1MaxCurrent(uint8_t address, uint32_t max) = 0;
  virtual bool SetM2MaxCurrent(uint8_t address, uint32_t max) = 0;
  virtual bool ReadM1MaxCurrent(uint8_t address, uint32_t& max) = 0;
  virtual bool ReadM2MaxCurrent(uint8_t address, uint32_t& max) = 0;

  virtual bool SetM1VelocityPID(uint8_t address, float kp, float ki, float kd, uint32_t qpps) = 0;
  virtual bool SetM2VelocityPID(uint8_t address, float kp, float ki, float kd, uint32_t qpps) = 0;

  virtual uint32_t ReadEncM1(uint8_t address, uint8_t* status = nullptr, bool* valid = nullptr) = 0;
  virtual uint32_t ReadEncM2(uint8_t address, uint8_t* status = nullptr, bool* valid = nullptr) = 0;
  virtual uint32_t ReadSpeedM1(uint8_t address, uint8_t* status = nullptr, bool* valid = nullptr) = 0;
  virtual uint32_t ReadSpeedM2(uint8_t address, uint8_t* status = nullptr, bool* valid = nullptr) = 0;

  virtual bool ReadCurrents(uint8_t address, int16_t& current1, int16_t& current2) = 0;
  virtual uint16_t ReadMainBatteryVoltage(uint8_t address, bool* valid = nullptr) = 0;
  virtual uint16_t ReadLogicBatteryVoltage(uint8_t address, bool* valid = nullptr) = 0;

  virtual bool ReadTemp(uint8_t address, uint16_t& temp) = 0;
  virtual bool ReadVersion(uint8_t address, char* version) = 0;

  virtual bool SpeedAccelDistanceM1M2(
      uint8_t address,
      uint32_t accel,
      int32_t speed1,
      uint32_t distance1,
      int32_t speed2,
      uint32_t distance2,
      uint8_t flag = 0) = 0;
};

}  // namespace sigyn_teensy
