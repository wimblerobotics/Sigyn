#pragma once

#include <Arduino.h>
#ifndef UNIT_TEST
#include <Wire.h>
#endif
#include "../interfaces/i_power_sensor.h"
#include <INA226.h>

namespace sigyn_teensy {

/**
 * @brief Implementation of IPowerSensor for the INA226 hardware sensor.
 * Supports I2C multiplexing for multiple sensors on the same bus.
 */
class INA226Sensor : public IPowerSensor {
public:
    /**
     * @brief Constructor
     * @param address I2C address of the INA226 (usually 0x40)
     * @param mux_channel Multiplexer channel (0-7), or -1 if no multiplexer
     * @param mux_address Multiplexer I2C address
     */
    INA226Sensor(uint8_t address, int8_t mux_channel = -1, uint8_t mux_address = 0x70)
        : ina226_(address), mux_channel_(mux_channel), mux_address_(mux_address) {}

    bool init() override {
        select();
        bool success = ina226_.begin();
        if (success) {
            ina226_.setAverage(INA226_16_SAMPLES);
            ina226_.setBusVoltageConversionTime(INA226_1100_us);
            ina226_.setShuntVoltageConversionTime(INA226_1100_us);
            // Mode setting might be default or different API
            // ina226_.setMode(INA226_MODE_SHUNT_BUS_CONT); // Check if this exists
            
            // Re-check calibrate API: 
            // int setMaxCurrentShunt(float maxCurrent = 20.0, float shunt = 0.002, bool normalize = true);
            // int configure(float shunt = 0.1, float current_LSB_mA = 0.1, float current_zero_offset_mA = 0, uint16_t bus_V_scaling_e4 = 10000);
            
            // Using setMaxCurrentShunt as equivalent to calibrate(shunt, maxCurrent)
            ina226_.setMaxCurrentShunt(100.0, 0.002);
        }
        return success;
    }

    bool isConnected() override {
        select();
        return ina226_.isConnected();
    }

    float readBusVoltage() override {
        select();
        return ina226_.getBusVoltage();
    }

    float readCurrent() override {
        select();
        return ina226_.getCurrent();
    }

    float readPower() override {
        select();
        return ina226_.getPower();
    }

    void select() override {
        if (mux_channel_ >= 0) {
            Wire.beginTransmission(mux_address_);
            Wire.write(1 << mux_channel_);
            Wire.endTransmission();
            // Small delay to allow mux to switch
            delayMicroseconds(10); 
        }
    }

private:
    INA226 ina226_;
    int8_t mux_channel_;
    uint8_t mux_address_;
};

} // namespace sigyn_teensy
