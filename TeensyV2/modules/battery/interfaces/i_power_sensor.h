#pragma once

#include <stdint.h>

namespace sigyn_teensy {

/**
 * @brief Interface for power sensors (monitoring voltage and current).
 * Provides abstraction for hardware sensors like INA226 or ADC-based monitoring.
 */
class IPowerSensor {
public:
    virtual ~IPowerSensor() = default;

    /**
     * @brief Initialize the sensor hardware.
     * @return true if initialization was successful.
     */
    virtual bool init() = 0;

    /**
     * @brief Check if the sensor is connected and communicating.
     * @return true if sensor is responsive.
     */
    virtual bool isConnected() = 0;

    /**
     * @brief Read the bus voltage in Volts.
     * @return Voltage in Volts.
     */
    virtual float readBusVoltage() = 0;

    /**
     * @brief Read the current in Amperes.
     * @return Current in Amps.
     */
    virtual float readCurrent() = 0;

    /**
     * @brief Read the power in Watts.
     * @return Power in Watts.
     */
    virtual float readPower() = 0;

    /**
     * @brief Prepare sensor for reading (e.g. set multiplexer).
     * Optional for sensors that don't need channel selection.
     */
    virtual void select() {}
};

} // namespace sigyn_teensy
