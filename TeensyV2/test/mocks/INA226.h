// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file INA226.h (Mock)
 * @brief Mock INA226 library for PC-based unit testing
 * 
 * Stub implementation - not used in tests since we inject MockPowerSensor.
 * This just allows the ina226_sensor.h to compile.
 */

#pragma once

#include <cstdint>

// INA226 conversion time constants (from real library)
#define INA226_1100_us 4

// INA226 averaging constants (from real library)
#define INA226_16_SAMPLES 3

// Mock INA226 class
class INA226 {
public:
    INA226(uint8_t address) {}
    
    bool begin() { return false; }
    bool isConnected() { return false; }
    
    float getBusVoltage() { return 0.0f; }
    float getCurrent() { return 0.0f; }
    float getPower() { return 0.0f; }
    
    void setAverage(uint8_t avg) {}
    void setBusVoltageConversionTime(uint8_t cvt) {}
    void setShuntVoltageConversionTime(uint8_t cvt) {}
    int setMaxCurrentShunt(float maxCurrent, float shunt, bool normalize = true) { return 0; }
};
