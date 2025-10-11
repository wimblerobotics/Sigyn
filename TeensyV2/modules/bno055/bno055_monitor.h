/**
 * @file bno055_monitor.h
 * @brief Comprehensive IMU monitoring system using dual BNO055 sensors
 *
 * This file implements the BNO055Monitor module that provides real-time
 * monitoring of orientation, angular velocity, and linear acceleration using
 * two BNO055 9-DOF IMU sensors for the TeensyV2 system. The implementation
 * supports redundant sensor configurations with automatic sensor detection
 * and graceful degradation.
 *
 * Key Implementation Features:
 *
 * **Dual-Sensor Support:**
 * - Automatic detection and configuration of BNO055 sensors
 * - Graceful degradation when sensors are unavailable
 * - Support for two simultaneous IMU sensors on separate I2C multiplexer channels
 * - Configurable I2C multiplexer support for sensor selection
 *
 * **Real-Time Monitoring:**
 * - High-frequency data acquisition (up to 100Hz) with minimal latency
 * - Quaternion, Euler angles, gyroscope, and linear acceleration data
 * - Exponential moving average (EMA) filtering for stable readings
 * - Low-latency data acquisition suitable for navigation and control
 *
 * **Safety Integration:**
 * - Automatic detection of sensor failures or invalid readings
 * - Integration with global safety system via isUnsafe() interface
 * - Sensor health monitoring and error reporting
 * - Configurable safety thresholds with appropriate hysteresis
 *
 * **Data Formats:**
 * - Native BNO055 coordinate system with optional ROS coordinate conversion
 * - Quaternion output in both w,x,y,z and x,y,z,w formats
 * - Angular velocity in rad/s and linear acceleration in m/s²
 * - Euler angles in degrees for human-readable orientation
 *
 * **Performance Characteristics:**
 * - Sensor reading time: <2ms for both sensors combined
 * - Memory footprint: ~256 bytes per sensor configuration
 * - No dynamic memory allocation during operation
 * - Deterministic execution time for real-time safety
 *
 * **Error Handling:**
 * - Robust I2C communication with automatic retry logic
 * - Sensor disconnection detection and graceful recovery
 * - Invalid reading detection and filtering
 * - Comprehensive error reporting via SerialManager
 *
 * The implementation follows the TeensyV2 architectural principles of
 * modularity, safety, and real-time performance while providing the
 * detailed orientation information needed for autonomous robot navigation.
 *
 * @author Sigyn Robotics
 * @date 2025
 * @version 2.0
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

#include "../../common/core/module.h"

namespace sigyn_teensy {

  /**
   * @brief BNO055 sensor operating modes
   */
  enum class BNO055Mode {
    CONFIG = 0x00,    ///< Configuration mode for setup
    NDOF = 0x0C       ///< 9-DOF absolute orientation mode
  };

  /**
   * @brief BNO055 power modes
   */
  enum class BNO055PowerMode {
    NORMAL = 0x00,    ///< Normal power mode
    LOW_POWER = 0x01, ///< Low power mode
    SUSPEND = 0x02    ///< Suspend mode
  };

  /**
   * @brief IMU sensor state enumeration
   */
  enum class IMUState {
    UNKNOWN,          ///< Sensor state unknown
    INITIALIZING,     ///< Sensor initializing
    CALIBRATING,      ///< Sensor calibrating
    NORMAL,           ///< Normal operation
    WARNING,          ///< Warning conditions detected
    CRITICAL,         ///< Critical error condition
    FAILED            ///< Sensor failed
  };

  /**
   * @brief Comprehensive IMU data structure
   */
  struct IMUData {
    // Quaternion (w, x, y, z format)
    float qw = 0.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;

    // Angular velocity (rad/s)
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;

    // Linear acceleration (m/s²)
    float ax = 0.0f, ay = 0.0f, az = 0.0f;

    // Euler angles (degrees)
    float euler_h = 0.0f, euler_r = 0.0f, euler_p = 0.0f;

    // Status and timing
    uint8_t calibration_status = 0;       ///< Calibration status byte
    uint8_t system_status = 0;            ///< System status byte
    uint8_t system_error = 0;             ///< System error byte
    uint32_t timestamp_ms = 0;            ///< Reading timestamp
    bool valid = false;                   ///< Data validity flag
  };

  /**
   * @brief State for non-blocking sensor reading state machine
   */
  enum class ReadState {
    IDLE,
    READ_QUATERNION,
    READ_GYROSCOPE,
    READ_ACCELERATION,
    READ_EULER,
    READ_STATUS,
    COMPLETE
  };

  /**
   * @brief BNO055 sensor configuration structure
   */
  struct BNO055Config {
    uint8_t i2c_address = 0x28;           ///< I2C address (typically 0x28 or 0x29)
    uint8_t mux_channel = 0;              ///< I2C multiplexer channel
    uint32_t read_interval_ms = 20;       ///< Reading interval (50Hz default)
    uint32_t timeout_ms = 100;            ///< Communication timeout
    bool enable_calibration = true;       ///< Enable automatic calibration
    bool enable_temp_compensation = false; ///< Enable temperature compensation
    float gyro_scale = 1.0f;              ///< Gyroscope scaling factor
    float accel_scale = 1.0f;             ///< Accelerometer scaling factor
  };

  /**
   * @brief High-performance dual BNO055 IMU monitoring system.
   *
   * Efficiently monitors two BNO055 9-DOF IMU sensors and converts data to
   * appropriate formats for navigation and control systems. Provides validation,
   * error recovery, and statistical analysis of sensor performance.
   *
   * Features:
   * - Dual sensor redundancy for critical applications
   * - Real-time orientation and motion data
   * - Automatic sensor calibration and health monitoring
   * - Configurable data rates and filtering
   * - ROS-compatible coordinate frame conversion
   *
   * Usage:
   * @code
   * BNO055Monitor& monitor = BNO055Monitor::getInstance();
   *
   * // Get data from primary sensor
   * IMUData data;
   * if (monitor.getIMUData(0, data)) {
   *   // Use orientation data
   * }
   * @endcode
   */
  class BNO055Monitor : public Module {
  public:
    // Constants
    static constexpr uint8_t kMaxSensors = 2;           ///< Maximum supported sensors
    static constexpr uint8_t kBNO055Address = 0x28;     ///< Default BNO055 I2C address (can be 0x28 or 0x29)
    static constexpr uint8_t kBNO055ChipID = 0xA0;      ///< Expected chip ID
    static constexpr uint8_t kI2CMultiplexerAddress = 0x70; ///< I2C multiplexer address
    static constexpr uint32_t kDefaultUpdateInterval = 20; ///< Default update interval (50Hz)
    static constexpr uint32_t kDefaultReportInterval = 100; ///< Default report interval (10Hz)
    // New: per-sensor publish interval targeting 20 Hz per sensor
    static constexpr uint32_t kPerSensorPublishIntervalMs = 20; ///< Per-sensor publish interval (20Hz)
    // Number of consecutive failures before escalating a sensor to CRITICAL
    static constexpr uint8_t kCriticalFailThreshold = 3; ///< Escalate after this many consecutive read failures

    // Watchdog thresholds
    static constexpr uint32_t kStaleWarnMs = 500;   ///< Age to warn about stale IMU publish
    static constexpr uint32_t kStaleResetMs = 2000;  ///< Age to attempt watchdog recovery

    // BNO055 Register addresses
    static constexpr uint8_t kRegChipID = 0x00;         ///< Chip ID register
    static constexpr uint8_t kRegPageID = 0x07;         ///< Page ID register
    static constexpr uint8_t kRegOprMode = 0x3D;        ///< Operation mode register
    static constexpr uint8_t kRegPwrMode = 0x3E;        ///< Power mode register
    static constexpr uint8_t kRegSysTrigger = 0x3F;     ///< System trigger register
    static constexpr uint8_t kRegSysStatus = 0x39;      ///< System status register
    static constexpr uint8_t kRegSysErr = 0x3A;         ///< System error register
    static constexpr uint8_t kRegCalibStat = 0x35;      ///< Calibration status register
    static constexpr uint8_t kRegUnitSel = 0x3B;        ///< Unit selection register (accel/gyro/euler/temp units)
    static constexpr uint8_t kRegAxisMapConfig = 0x41;  ///< Axis mapping configuration register
    static constexpr uint8_t kRegAxisMapSign = 0x42;    ///< Axis mapping sign register

    // Data registers
    static constexpr uint8_t kRegQuaternionW = 0x20;    ///< Quaternion W LSB
    static constexpr uint8_t kRegGyroX = 0x14;          ///< Gyroscope X LSB
    static constexpr uint8_t kRegLinearAccelX = 0x28;   ///< Linear acceleration X LSB
    static constexpr uint8_t kRegEulerH = 0x1A;         ///< Euler heading LSB

    // Public Methods
    static BNO055Monitor& getInstance();
    /**
     * @brief Returns the name of the module.
     * @return const char* The name of the module.
     */
    const char* name() const override;

    /**
     * @brief Set up the BNO055 monitor and initialize sensors.
     */
    void setup() override;

    /**
     * @brief Main loop for processing sensor data.
     */
    void loop() override;

    /**
     * @brief Check if the module is in an unsafe state.
     *
     * @return True if the module is in an unsafe state, false otherwise.
     */
    bool isUnsafe() override;

    /**
     * @brief Reset safety flags and re-initialize failed sensors.
     */
    void resetSafetyFlags() override;

    /**
     * @brief Get the latest IMU data for a specific sensor.
     *
     * @param[in] sensor_id Sensor identifier (0 or 1)
     * @param[out] data IMU data structure to populate
     * @return true if data is valid and retrieved successfully
     */
    bool getIMUData(uint8_t sensor_id, IMUData& data) const;

    /**
     * @brief Get IMU data in ROS coordinate frame.
     *
     * @param[in] sensor_id Sensor identifier (0 or 1)
     * @param[out] qx Quaternion x component
     * @param[out] qy Quaternion y component
     * @param[out] qz Quaternion z component
     * @param[out] qw Quaternion w component
     * @param[out] gx Angular velocity x (rad/s)
     * @param[out] gy Angular velocity y (rad/s)
     * @param[out] gz Angular velocity z (rad/s)
     * @param[out] ax Linear acceleration x (m/s²)
     * @param[out] ay Linear acceleration y (m/s²)
     * @param[out] az Linear acceleration z (m/s²)
     * @return true if data is valid and retrieved successfully
     */
    bool getIMUDataROS(uint8_t sensor_id,
      float& qx, float& qy, float& qz, float& qw,
      float& gx, float& gy, float& gz,
      float& ax, float& ay, float& az) const;

    /**
     * @brief Get the current state of a sensor.
     *
     * @param[in] sensor_id Sensor identifier (0 or 1)
     * @return Current sensor state
     */
    IMUState getSensorState(uint8_t sensor_id) const;

    /**
     * @brief Check if sensor is calibrated.
     *
     * @param[in] sensor_id Sensor identifier (0 or 1)
     * @return true if sensor is fully calibrated
     */
    bool isSensorCalibrated(uint8_t sensor_id) const;

    /**
     * @brief Update the configuration for a specific sensor.
     *
     * @param[in] sensor_id Sensor identifier (0 or 1)
     * @param[in] config New configuration
     * @return true if configuration updated successfully
     */
    bool updateSensorConfig(uint8_t sensor_id, const BNO055Config& config);

  private:
    /**
     * @brief Private constructor for singleton pattern.
     */
    BNO055Monitor();

    /**
     * @brief Initialize a single BNO055 sensor.
     *
     * @param[in] sensor_id Sensor identifier (0 or 1)
     * @return true if sensor initialized successfully
     */
    bool initializeSensor(uint8_t sensor_id);

    /**
     * @brief Prime sensor with initial readings to stabilize fusion engine.
     *
     * @param sensor_id The ID of the sensor to prime (0 or 1).
     * @return True if priming was successful, false otherwise.
     */
    bool primeSensorReadings(uint8_t sensor_id);

    /**
     * @brief Select the I2C multiplexer channel for a sensor.
     *
     * @param[in] mux_channel Multiplexer channel to select
     */
    void selectSensor(uint8_t mux_channel) const;
    // Helper to force a multiplexer reselect (used by watchdog)
    void reselectSensor_(uint8_t mux_channel) const;

    /**
     * @brief Test if the I2C multiplexer is available.
     *
     * @return true if multiplexer is available
     */
    bool testI2CMultiplexer() const;

    /**
     * @brief Read data from a BNO055 register.
     *
     * @param[in] reg Register address
     * @param[out] data Buffer to store read data
     * @param[in] length Number of bytes to read
     * @return true if read successful
     */
    bool readRegister(uint8_t reg, uint8_t* data, uint8_t length) const;

    /**
     * @brief Write a value to a BNO055 register.
     *
     * @param[in] reg Register address
     * @param[in] value Value to write
     * @return true if write successful
     */
    bool writeRegister(uint8_t reg, uint8_t value) const;

    /**
     * @brief Check the chip ID of the BNO055 sensor.
     *
     * @return true if chip ID matches expected value
     */
    bool checkChipID() const;

    /**
     * @brief Read quaternion data from the sensor.
     *
     * @param[out] w Quaternion w component
     * @param[out] x Quaternion x component
     * @param[out] y Quaternion y component
     * @param[out] z Quaternion z component
     * @return true if read successful
     */
    bool readQuaternion(float& w, float& x, float& y, float& z) const;

    /**
     * @brief Read gyroscope data from the sensor.
     *
     * @param[out] x Angular velocity x (rad/s)
     * @param[out] y Angular velocity y (rad/s)
     * @param[out] z Angular velocity z (rad/s)
     * @return true if read successful
     */
    bool readGyroscope(float& x, float& y, float& z) const;

    /**
     * @brief Read linear acceleration data from the sensor.
     *
     * @param[out] x Linear acceleration x (m/s²)
     * @param[out] y Linear acceleration y (m/s²)
     * @param[out] z Linear acceleration z (m/s²)
     * @return true if read successful
     */
    bool readLinearAcceleration(float& x, float& y, float& z) const;

    /**
     * @brief Read Euler angles from the sensor.
     *
     * @param[out] heading Heading angle (degrees)
     * @param[out] roll Roll angle (degrees)
     * @param[out] pitch Pitch angle (degrees)
     * @return true if read successful
     */
    bool readEulerAngles(float& heading, float& roll, float& pitch) const;

    /**
     * @brief Read status registers from the sensor.
     *
     * @param[out] sys_status System status
     * @param[out] sys_error System error
     * @param[out] calib_status Calibration status
     * @return true if read successful
     */
    bool readStatus(uint8_t& sys_status, uint8_t& sys_error, uint8_t& calib_status) const;

    /**
     * @brief Send a status message for a sensor via SerialManager.
     *
     * @param[in] sensor_id Sensor identifier (0 or 1)
     */
    void sendStatusMessage(uint8_t sensor_id);

    /**
     * @brief Convert BNO055 quaternion to ROS coordinate frame.
     *
     * @param[in] bno_w BNO055 quaternion w
     * @param[in] bno_x BNO055 quaternion x
     * @param[in] bno_y BNO055 quaternion y
     * @param[in] bno_z BNO055 quaternion z
     * @param[out] ros_x ROS quaternion x
     * @param[out] ros_y ROS quaternion y
     * @param[out] ros_z ROS quaternion z
     * @param[out] ros_w ROS quaternion w
     */
    static void convertQuaternionToROS(float bno_w, float bno_x, float bno_y, float bno_z,
      float& ros_x, float& ros_y, float& ros_z, float& ros_w);

    // Member variables
    BNO055Config sensor_configs_[kMaxSensors];  ///< Sensor configurations
    IMUData sensor_data_[kMaxSensors];          ///< Latest sensor data
    IMUState sensor_states_[kMaxSensors];       ///< Current sensor states
    ReadState current_read_state_[kMaxSensors]; ///< Current read state for state machine
    uint32_t last_update_time_[kMaxSensors];    ///< Last successful update time
    uint32_t next_sensor_start_time_[kMaxSensors]; ///< Next scheduled read time
    uint32_t last_report_time_;                 ///< Last report time for throttling
    uint8_t active_sensor_count_;               ///< Number of active sensors
    bool multiplexer_available_;                ///< Flag for I2C multiplexer availability
    bool setup_completed_;                      ///< Flag to indicate setup is done
    bool safety_violation_;                     ///< Flag for safety violations

    // Scheduling helpers for staggered reads
    uint8_t current_sensor_index_ = 0;          ///< Index for staggering sensor reads
    uint32_t sensor_stagger_interval_ = kDefaultReportInterval / kMaxSensors; ///< Stagger interval for reads

    // IMU watchdog and diagnostics
    uint32_t last_publish_time_ms_[kMaxSensors] = { 0, 0 };    ///< Last time we published IMU for each sensor
    uint32_t last_stale_report_ms_[kMaxSensors] = { 0, 0 };    ///< Throttle stale warnings per sensor
    uint32_t last_watchdog_reset_ms_[kMaxSensors] = { 0, 0 };  ///< Throttle resets per sensor
    uint32_t imu_reset_count_[kMaxSensors] = { 0, 0 };         ///< Number of watchdog recoveries per sensor
    mutable uint32_t i2c_error_count_ = 0;                   ///< Count of I2C errors (mutable for const methods)
    uint32_t mux_reselect_count_ = 0;                        ///< Times we forced mux reselection
    uint32_t last_heartbeat_ms_ = 0;                         ///< Last time we emitted heartbeat

    // Scale factors for sensor data conversion
    static constexpr float kScaleQuaternion = 16384.0f; // 1 / (2^14)
    static constexpr float kScaleAccel = 100.0f;      // 100 LSB/m/s²
    static constexpr float kScaleGyro = 16.0f;        // 16 LSB/°/s
    static constexpr float kScaleEuler = 16.0f;       // 16 LSB/°
    static constexpr float kRadPerDeg = 0.01745329251994329576923690768489f; // PI / 180

    /**
     * @brief Performance statistics for each sensor.
     */
    struct PerformanceStats {
      float min = 0.0f;
      float max = 0.0f;
      float avg = 0.0f;
      float last = 0.0f;
      uint32_t count = 0;
    };

    PerformanceStats performance_stats_[kMaxSensors];

    /**
     * @brief Validate gyroscope reads during priming.
     *
     * Ensures that gyroscope reads are successful before completing priming.
     * Logs failures and retries if necessary.
     *
     * @param sensor_id The ID of the sensor to validate (0 or 1).
     * @return True if gyroscope reads are successful, false otherwise.
     */
    bool validateGyroscopeReadsDuringPriming(uint8_t sensor_id);

    /**
     * @brief Initialize min value in performance stats on first successful read.
     *
     * Ensures that the min value is set correctly during the first valid sensor read.
     *
     * @param sensor_id The ID of the sensor to update (0 or 1).
     * @param value The value to initialize the min field with.
     */
    void initializeMinValueIfUnset(uint8_t sensor_id, float value);

    // Watchdog helpers
    void checkWatchdog_();
    void recoverSensor_(uint8_t sensor_id);

    // Retry helpers (declared here; definitions in .cpp)
    bool readGyroWithRetry_(uint8_t sensor_id, float& x, float& y, float& z);
    bool readAccelWithRetry_(uint8_t sensor_id, float& x, float& y, float& z);

    // Failure counters for adaptive degradation logic
    uint8_t consecutive_fail_quat_[kMaxSensors] = { 0, 0 };
    uint8_t consecutive_fail_gyro_[kMaxSensors] = { 0, 0 };
    uint8_t consecutive_fail_accel_[kMaxSensors] = { 0, 0 };
  };

} // namespace sigyn_teensy
