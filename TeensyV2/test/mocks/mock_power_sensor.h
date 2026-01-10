// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file mock_power_sensor.h
 * @brief Mock implementation of IPowerSensor for unit testing
 */

#pragma once

#include "modules/battery/interfaces/i_power_sensor.h"
#include <cstdint>

namespace sigyn_teensy {

/**
 * @brief Mock power sensor for testing battery monitoring logic
 * 
 * Allows tests to simulate various sensor states:
 * - Connected/disconnected sensors
 * - Various voltage and current readings
 * - Sensor initialization failures
 */
class MockPowerSensor : public IPowerSensor {
public:
    MockPowerSensor() 
        : initialized_(false)
        , connected_(false)
        , voltage_(0.0f)
        , current_(0.0f)
        , power_(0.0f)
        , init_should_fail_(false)
        , select_count_(0) {}

    // IPowerSensor interface
    bool init() override {
        if (init_should_fail_) {
            return false;
        }
        initialized_ = true;
        connected_ = true;
        return true;
    }

    bool isConnected() override {
        return connected_;
    }

    float readBusVoltage() override {
        return voltage_;
    }

    float readCurrent() override {
        return current_;
    }

    float readPower() override {
        return power_;
    }

    void select() override {
        select_count_++;
    }

    // Test control methods
    void setVoltage(float v) { voltage_ = v; }
    void setCurrent(float c) { current_ = c; }
    void setPower(float p) { power_ = p; }
    void setConnected(bool connected) { connected_ = connected; }
    void setInitShouldFail(bool should_fail) { init_should_fail_ = should_fail; }
    
    bool isInitialized() const { return initialized_; }
    int getSelectCount() const { return select_count_; }
    
    // Reset for test isolation
    void reset() {
        initialized_ = false;
        connected_ = false;
        voltage_ = 0.0f;
        current_ = 0.0f;
        power_ = 0.0f;
        init_should_fail_ = false;
        select_count_ = 0;
    }

private:
    bool initialized_;
    bool connected_;
    float voltage_;
    float current_;
    float power_;
    bool init_should_fail_;
    int select_count_;
};

} // namespace sigyn_teensy
