// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file mock_roboclaw.h
 * @brief Minimal, controllable RoboClaw mock for unit testing RoboClawMonitor
 */

#pragma once

#include <cstdint>
#include <cstring>

#include "modules/roboclaw/interfaces/i_roboclaw.h"

namespace sigyn_teensy {

class MockRoboClaw final : public IRoboClaw {
 public:
  struct State {
    // Call counters
    uint32_t begin_calls = 0;
    uint32_t reset_encoders_calls = 0;
    uint32_t read_error_calls = 0;
    uint32_t set_m1_max_current_calls = 0;
    uint32_t set_m2_max_current_calls = 0;
    uint32_t read_m1_max_current_calls = 0;
    uint32_t read_m2_max_current_calls = 0;
    uint32_t set_m1_pid_calls = 0;
    uint32_t set_m2_pid_calls = 0;
    uint32_t read_enc_m1_calls = 0;
    uint32_t read_enc_m2_calls = 0;
    uint32_t read_speed_m1_calls = 0;
    uint32_t read_speed_m2_calls = 0;
    uint32_t read_currents_calls = 0;
    uint32_t read_main_voltage_calls = 0;
    uint32_t read_logic_voltage_calls = 0;
    uint32_t read_temp_calls = 0;
    uint32_t read_version_calls = 0;
    uint32_t speed_accel_distance_calls = 0;

    // Configurable outputs / behaviors
    bool reset_encoders_ok = true;

    uint32_t read_error_value = 0;
    bool read_error_valid = true;

    bool set_m1_max_current_ok = true;
    bool set_m2_max_current_ok = true;

    uint32_t m1_max_current = 0;
    uint32_t m2_max_current = 0;
    bool read_m1_max_current_ok = true;
    bool read_m2_max_current_ok = true;

    bool set_m1_pid_ok = true;
    bool set_m2_pid_ok = true;

    uint32_t enc_m1 = 0;
    uint32_t enc_m2 = 0;
    uint8_t enc_status_m1 = 0;
    uint8_t enc_status_m2 = 0;
    bool enc_valid_m1 = true;
    bool enc_valid_m2 = true;

    uint32_t speed_m1 = 0;
    uint32_t speed_m2 = 0;
    uint8_t speed_status_m1 = 0;
    uint8_t speed_status_m2 = 0;
    bool speed_valid_m1 = true;
    bool speed_valid_m2 = true;

    bool read_currents_ok = true;
    int16_t current1 = 0;
    int16_t current2 = 0;

    uint16_t main_voltage = 0;
    bool main_voltage_valid = true;

    uint16_t logic_voltage = 0;
    bool logic_voltage_valid = true;

    bool read_temp_ok = true;
    uint16_t temp = 0;

    bool read_version_ok = true;
    char version[64] = "Roboclaw Mock";

    bool speed_accel_distance_ok = true;

    // Last command captured
    struct LastSpeedAccelDistance {
      bool was_called = false;
      uint8_t address = 0;
      uint32_t accel = 0;
      int32_t speed1 = 0;
      uint32_t distance1 = 0;
      int32_t speed2 = 0;
      uint32_t distance2 = 0;
      uint8_t flag = 0;
    } last_cmd;
  } state;

  void reset() { state = State{}; }

  // IRoboClaw
  void begin(long) override { state.begin_calls++; }

  bool ResetEncoders(uint8_t) override {
    state.reset_encoders_calls++;
    return state.reset_encoders_ok;
  }

  uint32_t ReadError(uint8_t, bool* valid = nullptr) override {
    state.read_error_calls++;
    if (valid) *valid = state.read_error_valid;
    return state.read_error_value;
  }

  bool SetM1MaxCurrent(uint8_t, uint32_t) override {
    state.set_m1_max_current_calls++;
    return state.set_m1_max_current_ok;
  }

  bool SetM2MaxCurrent(uint8_t, uint32_t) override {
    state.set_m2_max_current_calls++;
    return state.set_m2_max_current_ok;
  }

  bool ReadM1MaxCurrent(uint8_t, uint32_t& max) override {
    state.read_m1_max_current_calls++;
    max = state.m1_max_current;
    return state.read_m1_max_current_ok;
  }

  bool ReadM2MaxCurrent(uint8_t, uint32_t& max) override {
    state.read_m2_max_current_calls++;
    max = state.m2_max_current;
    return state.read_m2_max_current_ok;
  }

  bool SetM1VelocityPID(uint8_t, float, float, float, uint32_t) override {
    state.set_m1_pid_calls++;
    return state.set_m1_pid_ok;
  }

  bool SetM2VelocityPID(uint8_t, float, float, float, uint32_t) override {
    state.set_m2_pid_calls++;
    return state.set_m2_pid_ok;
  }

  uint32_t ReadEncM1(uint8_t, uint8_t* status = nullptr, bool* valid = nullptr) override {
    state.read_enc_m1_calls++;
    if (status) *status = state.enc_status_m1;
    if (valid) *valid = state.enc_valid_m1;
    return state.enc_m1;
  }

  uint32_t ReadEncM2(uint8_t, uint8_t* status = nullptr, bool* valid = nullptr) override {
    state.read_enc_m2_calls++;
    if (status) *status = state.enc_status_m2;
    if (valid) *valid = state.enc_valid_m2;
    return state.enc_m2;
  }

  uint32_t ReadSpeedM1(uint8_t, uint8_t* status = nullptr, bool* valid = nullptr) override {
    state.read_speed_m1_calls++;
    if (status) *status = state.speed_status_m1;
    if (valid) *valid = state.speed_valid_m1;
    return state.speed_m1;
  }

  uint32_t ReadSpeedM2(uint8_t, uint8_t* status = nullptr, bool* valid = nullptr) override {
    state.read_speed_m2_calls++;
    if (status) *status = state.speed_status_m2;
    if (valid) *valid = state.speed_valid_m2;
    return state.speed_m2;
  }

  bool ReadCurrents(uint8_t, int16_t& current1, int16_t& current2) override {
    state.read_currents_calls++;
    current1 = state.current1;
    current2 = state.current2;
    return state.read_currents_ok;
  }

  uint16_t ReadMainBatteryVoltage(uint8_t, bool* valid = nullptr) override {
    state.read_main_voltage_calls++;
    if (valid) *valid = state.main_voltage_valid;
    return state.main_voltage;
  }

  uint16_t ReadLogicBatteryVoltage(uint8_t, bool* valid = nullptr) override {
    state.read_logic_voltage_calls++;
    if (valid) *valid = state.logic_voltage_valid;
    return state.logic_voltage;
  }

  bool ReadTemp(uint8_t, uint16_t& temp) override {
    state.read_temp_calls++;
    temp = state.temp;
    return state.read_temp_ok;
  }

  bool ReadVersion(uint8_t, char* version) override {
    state.read_version_calls++;
    if (!version) return false;
    if (!state.read_version_ok) return false;
    std::strncpy(version, state.version, 63);
    version[63] = '\0';
    return true;
  }

  bool SpeedAccelDistanceM1M2(
      uint8_t address,
      uint32_t accel,
      int32_t speed1,
      uint32_t distance1,
      int32_t speed2,
      uint32_t distance2,
      uint8_t flag = 0) override {
    state.speed_accel_distance_calls++;
    state.last_cmd.was_called = true;
    state.last_cmd.address = address;
    state.last_cmd.accel = accel;
    state.last_cmd.speed1 = speed1;
    state.last_cmd.distance1 = distance1;
    state.last_cmd.speed2 = speed2;
    state.last_cmd.distance2 = distance2;
    state.last_cmd.flag = flag;
    return state.speed_accel_distance_ok;
  }
};

}  // namespace sigyn_teensy
