#pragma once

#include <Wire.h>
#include <stdint.h>

#include <vector>

#include "Arduino.h"
#include "module.h"

// BNO055 Register addresses
#define BNO055_CHIP_ID 0x00
#define BNO055_PAGE_ID 0x07
#define BNO055_OPR_MODE 0x3D
#define BNO055_PWR_MODE 0x3E
#define BNO055_SYS_TRIGGER 0x3F
#define BNO055_SYS_STATUS 0x39
#define BNO055_SYS_ERR 0x3A
#define BNO055_CALIB_STAT 0x35

// Operation modes
#define OPERATION_MODE_CONFIG 0x00
#define OPERATION_MODE_NDOF 0x0C

// Data registers
#define BNO055_QUATERNION_DATA_W_LSB 0x20
#define BNO055_GYRO_DATA_X_LSB 0x14
#define BNO055_LINEAR_ACCEL_DATA_X_LSB 0x28
#define BNO055_EULER_H_LSB 0x1A

// Expected chip ID
#define BNO055_ID 0xA0

// Structure to hold IMU data
struct IMUData {
  float qw, qx, qy, qz;             // Quaternion
  float gx, gy, gz;                 // Gyroscope (rad/s)
  float ax, ay, az;                 // Linear acceleration (m/s²)
  float euler_h, euler_r, euler_p;  // Euler angles (degrees)
  uint32_t read_time_tenths_ms;     // Time to read data in 0.1ms units
  bool valid;                       // Data validity
};

struct BNO055Chip {
  uint8_t address;
  uint8_t mux_channel;
  bool initialized;
  IMUData last_data;
  uint32_t last_read_time;
  uint32_t read_interval_ms;  // How often to read this chip
};

class BNO055Module : public Module {
 public:
  // Constants
  static const uint8_t MAX_SENSORS = 2;       // Maximum number of sensors supported
  static const uint8_t ENABLED_SENSORS = 2;   // Number of sensors currently enabled (change this to enable more sensors)

  // Get singleton instance
  static BNO055Module &singleton();

  // Public interface to get IMU data
  bool getIMUData(uint8_t chip_index, IMUData &data);

  // Get IMU data in ROS coordinate frame and quaternion order
  bool getIMUDataROS(uint8_t chip_index, float &qx, float &qy, float &qz, float &qw, float &gyro_x,
                     float &gyro_y, float &gyro_z, float &accel_x, float &accel_y, float &accel_z);

  // Handle incoming commands from PC/ROS2
  void handleCommand(const String &command);

  // Safety interface (inherited from Module)
  bool isUnsafe() override;

  // From Module.
  virtual const char *name() override { return "BNO055"; }

  void resetSafetyFlags() override;

  // Send IMU data via serial interface
  void sendIMUData();

 protected:
  // From Module.
  void loop() override;

  // From Module.
  void setup() override;

 private:
  // Constants
  static const uint8_t BNO055_DEFAULT_ADDRESS = 0x28;   // Default BNO055 I2C address

  // Private constructor
  BNO055Module();

  // BNO055 communication functions (alphabetical order)
  bool bno_check_id(uint8_t addr);
  bool bno_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
  bool bno_setup(uint8_t addr);
  bool bno_write(uint8_t addr, uint8_t reg, uint8_t val);

  // Convert BNO055 quaternion (w,x,y,z) to ROS format (x,y,z,w) with coordinate transform
  static void convertQuaternionToROS(float bno_w, float bno_x, float bno_y, float bno_z,
                                     float &ros_x, float &ros_y, float &ros_z, float &ros_w);

  // Initialization functions (alphabetical order)
  bool initializeSensor(uint8_t sensor_index);

  // Data reading functions (alphabetical order)
  bool read_euler(uint8_t addr, float *heading, float *roll, float *pitch);
  bool read_gyro(uint8_t addr, float *x, float *y, float *z);
  bool read_imu_data(uint8_t addr, IMUData *data);
  bool read_linear_accel(uint8_t addr, float *x, float *y, float *z);
  bool read_quaternion(uint8_t addr, float *w, float *x, float *y, float *z);

  void selectSensor(uint8_t sensor_index);

  bool testI2CMultiplexer();

  // Static data members
  static BNO055Module *g_instance_;

  // Instance data members
  uint32_t data_send_interval_ms_;  // How often to send data (default 100ms = 10Hz)
  uint32_t last_data_send_time_;    // Last time data was sent via serial
  bool multiplexer_available_;      // Multiplexer availability
  std::vector<BNO055Chip> sensors_;
  bool setup_completed_;            // Setup status
};
