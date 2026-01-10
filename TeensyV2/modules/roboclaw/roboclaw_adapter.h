// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

#pragma once

#include "interfaces/i_roboclaw.h"

#ifndef UNIT_TEST
#include "RoboClaw.h"

namespace sigyn_teensy {

/**
 * Adapter that exposes the concrete RoboClaw driver through the minimal
 * `IRoboClaw` interface used by `RoboClawMonitor`.
 */
class RoboClawAdapter final : public IRoboClaw {
 public:
  explicit RoboClawAdapter(RoboClaw& roboclaw) : roboclaw_(roboclaw) {}

  void begin(long speed) override { roboclaw_.begin(speed); }

  bool ResetEncoders(uint8_t address) override { return roboclaw_.ResetEncoders(address); }
  uint32_t ReadError(uint8_t address, bool* valid = nullptr) override { return roboclaw_.ReadError(address, valid); }

  bool SetM1MaxCurrent(uint8_t address, uint32_t max) override { return roboclaw_.SetM1MaxCurrent(address, max); }
  bool SetM2MaxCurrent(uint8_t address, uint32_t max) override { return roboclaw_.SetM2MaxCurrent(address, max); }
  bool ReadM1MaxCurrent(uint8_t address, uint32_t& max) override { return roboclaw_.ReadM1MaxCurrent(address, max); }
  bool ReadM2MaxCurrent(uint8_t address, uint32_t& max) override { return roboclaw_.ReadM2MaxCurrent(address, max); }

  bool SetM1VelocityPID(uint8_t address, float kp, float ki, float kd, uint32_t qpps) override {
    return roboclaw_.SetM1VelocityPID(address, kp, ki, kd, qpps);
  }
  bool SetM2VelocityPID(uint8_t address, float kp, float ki, float kd, uint32_t qpps) override {
    return roboclaw_.SetM2VelocityPID(address, kp, ki, kd, qpps);
  }

  uint32_t ReadEncM1(uint8_t address, uint8_t* status = nullptr, bool* valid = nullptr) override {
    return roboclaw_.ReadEncM1(address, status, valid);
  }
  uint32_t ReadEncM2(uint8_t address, uint8_t* status = nullptr, bool* valid = nullptr) override {
    return roboclaw_.ReadEncM2(address, status, valid);
  }
  uint32_t ReadSpeedM1(uint8_t address, uint8_t* status = nullptr, bool* valid = nullptr) override {
    return roboclaw_.ReadSpeedM1(address, status, valid);
  }
  uint32_t ReadSpeedM2(uint8_t address, uint8_t* status = nullptr, bool* valid = nullptr) override {
    return roboclaw_.ReadSpeedM2(address, status, valid);
  }

  bool ReadCurrents(uint8_t address, int16_t& current1, int16_t& current2) override {
    return roboclaw_.ReadCurrents(address, current1, current2);
  }
  uint16_t ReadMainBatteryVoltage(uint8_t address, bool* valid = nullptr) override {
    return roboclaw_.ReadMainBatteryVoltage(address, valid);
  }
  uint16_t ReadLogicBatteryVoltage(uint8_t address, bool* valid = nullptr) override {
    return roboclaw_.ReadLogicBatteryVoltage(address, valid);
  }

  bool ReadTemp(uint8_t address, uint16_t& temp) override { return roboclaw_.ReadTemp(address, temp); }
  bool ReadVersion(uint8_t address, char* version) override { return roboclaw_.ReadVersion(address, version); }

  bool SpeedAccelM1M2(
      uint8_t address,
      uint32_t accel,
      int32_t speed1,
      int32_t speed2) override {
    return roboclaw_.SpeedAccelM1M2(address, accel, speed1, speed2);
  }

  bool SpeedAccelDistanceM1M2(
      uint8_t address,
      uint32_t accel,
      int32_t speed1,
      uint32_t distance1,
      int32_t speed2,
      uint32_t distance2,
      uint8_t flag = 0) override {
    return roboclaw_.SpeedAccelDistanceM1M2(address, accel, speed1, distance1, speed2, distance2, flag);
  }

 private:
  RoboClaw& roboclaw_;
};

}  // namespace sigyn_teensy

#endif  // UNIT_TEST
