#include "bno055_module.h"

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "serial_manager.h"

const float RAD_PER_DEG = 0.0174533f;  // π/180

BNO055Module::BNO055Module() : Module() {
  // Initialize timing for data transmission
  last_data_send_time_ = 0;
  data_send_interval_ms_ = 100;  // Send data every 100ms (10Hz)
  setup_completed_ = false;
  multiplexer_available_ = false;

  // Initialize sensor array
  sensors_.resize(ENABLED_SENSORS);
  for (uint8_t i = 0; i < ENABLED_SENSORS; i++) {
    sensors_[i].address = BNO055_DEFAULT_ADDRESS;  // Default address
    sensors_[i].mux_channel = i;
    sensors_[i].initialized = false;
    sensors_[i].last_read_time = 0;
    sensors_[i].read_interval_ms = 50;
  }

  SerialManager::singleton().SendDiagnosticMessage(
      String("[BNO055Module::BNO055Module] Configured for ") +
      String(ENABLED_SENSORS) + String(" sensors"));

  // delay(1000);  // Give sensors time to power up

  // // Initialize each sensor
  // for (auto &sensor : sensors_) {
  //   selectSensor(sensor.mux_channel);
  //   sensor.initialized = bno_setup(sensor.address);
  //   sensor.last_read_time = 0;
  //   sensor.last_data.valid = false;

  //   if (sensor.initialized) {
  //     Serial.printf("BNO055 at 0x%02X, mux %d initialized successfully\n",
  //     sensor.address,
  //                   sensor.mux_channel);
  //   } else {
  //     Serial.printf("BNO055 at 0x%02X, mux %d initialization failed\n",
  //     sensor.address,
  //                   sensor.mux_channel);
  //   }
  // }

  // Increase I2C speed after initialization
  // Wire.setClock(400000);
}

void BNO055Module::setup() {
  if (setup_completed_) {
    return;  // Already completed setup
  }

  SerialManager::singleton().SendDiagnosticMessage(
      "[BNO055Module::setup] Starting sensor initialization...");

  // Initialize I2C if not already done
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  Wire.begin();
  Wire.setClock(400000);  // Set I2C to 400kHz for faster communication

  // Test I2C multiplexer connectivity
  multiplexer_available_ = testI2CMultiplexer();
  if (!multiplexer_available_) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BNO055Module::setup] I2C multiplexer not available, cannot proceed.");
    return;
  }

  // Initialize each enabled sensor
  uint8_t initialized_count = 0;

  for (uint8_t i = 0; i < ENABLED_SENSORS; i++) {
    if (initializeSensor(i)) {
      initialized_count++;
      SerialManager::singleton().SendDiagnosticMessage(
          "[BNO055Module::setup] Sensor " + String(i) +
          " initialized successfully");
    } else {
      SerialManager::singleton().SendDiagnosticMessage(
          "[BNO055Module::setup] Sensor " + String(i) +
          " initialization failed");
    }

    // Add a small delay between sensor initializations
    delay(1);
  }

  SerialManager::singleton().SendDiagnosticMessage(
      "[BNO055Module::setup] " + String(initialized_count) + "/" +
      String(ENABLED_SENSORS) + " sensors initialized");

  setup_completed_ = true;
}

void BNO055Module::loop() {
  if (!setup_completed_) {
    return;  // Skip if setup not completed
  }

  uint32_t current_time = millis();

  // Read from each initialized sensor based on its interval
  for (uint8_t sensor_index = 0; sensor_index < ENABLED_SENSORS;
       sensor_index++) {
    selectSensor(sensor_index);
    if (sensors_[sensor_index].initialized &&
        (current_time - sensors_[sensor_index].last_read_time >=
         sensors_[sensor_index].read_interval_ms)) {
      // Read IMU data (this should be fast, < 2ms)
      read_imu_data(sensors_[sensor_index].address,
                    &sensors_[sensor_index].last_data);
      sensors_[sensor_index].last_read_time = current_time;
      // Serial.printf("Read data from BNO055 at 0x%02X\n",
      // sensors_[sensor_index].address); Serial.printf("Data: Q=%.4f %.4f %.4f
      // %.4f G=%.2f %.2f %.2f A=%.2f %.2f %.2f E=%.2f %.2f
      // %.2f\n",
      //     sensor.last_data.qw, sensor.last_data.qx, sensor.last_data.qy,
      //     sensor.last_data.qz, sensor.last_data.gx, sensor.last_data.gy,
      //     sensor.last_data.gz, sensor.last_data.ax, sensor.last_data.ay,
      //     sensor.last_data.az, sensor.last_data.euler_h,
      //     sensor.last_data.euler_r, sensor.last_data.euler_p);
    } else {
      // Skip reading if not initialized or not time yet
      if (!sensors_[sensor_index].initialized) {
        Serial.printf("BNO055 at 0x%02X, mux %d is not initialized\n",
                      sensors_[sensor_index].address,
                      sensors_[sensor_index].mux_channel);
        // delay(10'000); // Wait before retrying
      } else {
        // Serial.printf("Skipping read for BNO055 at 0x%02X, not time yet\n",
        // sensor.address); delay(1000); // Wait before retrying
      }
    }
  }

  // Send data periodically (non-blocking)
  if (current_time - last_data_send_time_ >= data_send_interval_ms_) {
    sendIMUData();
    last_data_send_time_ = current_time;
  }
}

bool BNO055Module::isUnsafe() {
  // Check if any sensor is reporting errors or invalid data
  for (const auto &sensor : sensors_) {
    if (sensor.initialized && !sensor.last_data.valid) {
      return true;  // Sensor failure detected
    }
  }
  return false;
}

void BNO055Module::resetSafetyFlags() {
  // Reset any safety flags related to the BNO055 sensors
  // Could implement sensor reset logic here if needed
}

void BNO055Module::selectSensor(uint8_t sensor_index) {
  // Select the appropriate channel on the I2C multiplexer
  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
  Wire.write(1 << sensor_index);
  Wire.endTransmission();

  // Small delay to allow multiplexer to switch
  delayMicroseconds(100);
}

bool BNO055Module::getIMUData(uint8_t sensor_index, IMUData &data) {
  if (sensor_index >= sensors_.size()) {
    return false;
  }

  if (!sensors_[sensor_index].initialized) {
    return false;
  }

  data = sensors_[sensor_index].last_data;
  return data.valid;
}

// BNO055 communication functions
bool BNO055Module::bno_write(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  uint8_t result = Wire.endTransmission();

  if (result != 0) {
    return false;
  }

  delayMicroseconds(150);  // BNO055 needs time between register writes
  return true;
}

bool BNO055Module::bno_read(uint8_t addr, uint8_t reg, uint8_t *buf,
                            uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  uint8_t result = Wire.endTransmission(false);  // Send restart

  if (result != 0) {
    return false;
  }

  delayMicroseconds(150);

  uint8_t received = Wire.requestFrom(addr, len);
  if (received != len) {
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    if (Wire.available()) {
      buf[i] = Wire.read();
    } else {
      return false;
    }
  }

  return true;
}

bool BNO055Module::bno_check_id(uint8_t addr) {
  uint8_t sensor_id;
  if (!bno_read(addr, BNO055_CHIP_ID, &sensor_id, 1)) {
    return false;
  }
  return sensor_id == BNO055_ID;
}

bool BNO055Module::initializeSensor(uint8_t sensor_index) {
  if (sensor_index >= ENABLED_SENSORS) {
    return false;
  }

  // Select the sensor through multiplexer
  selectSensor(sensor_index);
  if (!bno_setup(sensors_[sensor_index].address)) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BNO055Module::initializeSensor] Failed to initialize sensor " +
        String(sensor_index));
    return false;
  }
  sensors_[sensor_index].initialized = true;
  sensors_[sensor_index].last_read_time = 0;
  return true;
}

bool BNO055Module::bno_setup(uint8_t addr) {
  // Check if device is present and has correct ID
  if (!bno_check_id(addr)) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BNO055Module::bno_setup] BNO055 not found at address 0x" +
        String(addr, HEX));
    return false;
  }

  // Set to config mode first
  if (!bno_write(addr, BNO055_OPR_MODE, OPERATION_MODE_CONFIG)) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BNO055Module::bno_setup] Failed to set BNO055 to config mode at "
        "address 0x" +
        String(addr, HEX));
    return false;
  }
  delay(25);

  // Reset the system
  if (!bno_write(addr, BNO055_SYS_TRIGGER, 0x20)) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BNO055Module::bno_setup] Failed to reset BNO055 at address 0x" +
        String(addr, HEX));
    return false;
  }
  delay(700);

  // Check if device is still there after reset
  if (!bno_check_id(addr)) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BNO055Module::bno_setup] BNO055 not found after reset at address 0x" +
        String(addr, HEX));
    return false;
  }

  // Set normal power mode
  if (!bno_write(addr, BNO055_PWR_MODE, 0x00)) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BNO055Module::bno_setup] Failed to set normal power mode at address "
        "0x" +
        String(addr, HEX));
    return false;
  }
  delay(10);

  // Select page 0
  if (!bno_write(addr, BNO055_PAGE_ID, 0x00)) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BNO055Module::bno_setup] Failed to select page 0 at address 0x" +
        String(addr, HEX));
    return false;
  }
  delay(10);

  // Set NDOF fusion mode
  if (!bno_write(addr, BNO055_OPR_MODE, OPERATION_MODE_NDOF)) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BNO055Module::bno_setup] Failed to set NDOF mode at address 0x" +
        String(addr, HEX));
    return false;
  }
  delay(100);

  return true;
}

bool BNO055Module::read_quaternion(uint8_t addr, float *w, float *x, float *y,
                                   float *z) {
  uint8_t buf[8];
  if (!bno_read(addr, BNO055_QUATERNION_DATA_W_LSB, buf, 8)) return false;
  int16_t qw = (int16_t)(buf[1] << 8 | buf[0]);
  int16_t qx = (int16_t)(buf[3] << 8 | buf[2]);
  int16_t qy = (int16_t)(buf[5] << 8 | buf[4]);
  int16_t qz = (int16_t)(buf[7] << 8 | buf[6]);
  *w = qw / 16384.0f;
  *x = qx / 16384.0f;
  *y = qy / 16384.0f;
  *z = qz / 16384.0f;
  return true;
}

bool BNO055Module::read_gyro(uint8_t addr, float *x, float *y, float *z) {
  uint8_t buf[6];
  if (!bno_read(addr, BNO055_GYRO_DATA_X_LSB, buf, 6)) return false;
  int16_t gx = (int16_t)(buf[1] << 8 | buf[0]);
  int16_t gy = (int16_t)(buf[3] << 8 | buf[2]);
  int16_t gz = (int16_t)(buf[5] << 8 | buf[4]);
  *x = gx * 0.0625f * RAD_PER_DEG;
  *y = gy * 0.0625f * RAD_PER_DEG;
  *z = gz * 0.0625f * RAD_PER_DEG;
  return true;
}

bool BNO055Module::read_linear_accel(uint8_t addr, float *x, float *y,
                                     float *z) {
  uint8_t buf[6];
  if (!bno_read(addr, BNO055_LINEAR_ACCEL_DATA_X_LSB, buf, 6)) return false;
  int16_t ax = (int16_t)(buf[1] << 8 | buf[0]);
  int16_t ay = (int16_t)(buf[3] << 8 | buf[2]);
  int16_t az = (int16_t)(buf[5] << 8 | buf[4]);
  *x = ax / 100.0f;
  *y = ay / 100.0f;
  *z = az / 100.0f;
  return true;
}

bool BNO055Module::read_euler(uint8_t addr, float *heading, float *roll,
                              float *pitch) {
  uint8_t buf[6];
  if (!bno_read(addr, BNO055_EULER_H_LSB, buf, 6)) return false;
  int16_t h = (int16_t)(buf[1] << 8 | buf[0]);
  int16_t r = (int16_t)(buf[3] << 8 | buf[2]);
  int16_t p = (int16_t)(buf[5] << 8 | buf[4]);
  *heading = h / 16.0f;
  *roll = r / 16.0f;
  *pitch = p / 16.0f;
  return true;
}

bool BNO055Module::read_imu_data(uint8_t addr, IMUData *data) {
  uint32_t start_time = micros();

  data->valid =
      read_quaternion(addr, &data->qw, &data->qx, &data->qy, &data->qz) &&
      read_gyro(addr, &data->gx, &data->gy, &data->gz) &&
      read_linear_accel(addr, &data->ax, &data->ay, &data->az) &&
      read_euler(addr, &data->euler_h, &data->euler_r, &data->euler_p);

  uint32_t end_time = micros();
  data->read_time_tenths_ms = (end_time - start_time) / 100;

  return data->valid;
}

void BNO055Module::sendIMUData() {
  // Send data from all initialized sensors in ROS format
  for (uint8_t i = 0; i < sensors_.size(); i++) {
    if (sensors_[i].initialized && sensors_[i].last_data.valid) {
      const IMUData &data = sensors_[i].last_data;

      // Convert to ROS format
      float ros_qx, ros_qy, ros_qz, ros_qw;
      convertQuaternionToROS(data.qw, data.qx, data.qy, data.qz, ros_qx, ros_qy,
                             ros_qz, ros_qw);

      // Convert other data to ROS coordinate system
      float ros_gx = data.gy;   // BNO055 Y → ROS X
      float ros_gy = -data.gx;  // BNO055 X → ROS Y (flip sign)
      float ros_gz = data.gz;   // BNO055 Z → ROS Z

      float ros_ax = data.ay;   // BNO055 Y → ROS X
      float ros_ay = -data.ax;  // BNO055 X → ROS Y (flip sign)
      float ros_az = data.az;   // BNO055 Z → ROS Z

      // Format:
      // IMU_DATA:sensor_id,qx,qy,qz,qw,gx,gy,gz,ax,ay,az,h,r,p,read_time (ROS
      // format)
      char message[256];
      snprintf(message, sizeof(message),
               "IMU_DATA:%d,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%"
               ".2f,%.2f,%.2f,%d",
               (int)i, ros_qx, ros_qy, ros_qz,
               ros_qw,                  // ROS quaternion order: x,y,z,w
               ros_gx, ros_gy, ros_gz,  // ROS coordinate system
               ros_ax, ros_ay, ros_az,  // ROS coordinate system
               data.euler_h, data.euler_r,
               data.euler_p,  // Keep Euler as-is for now
               (int)data.read_time_tenths_ms);

      SerialManager::singleton().SendDiagnosticMessage(message);
    }
  }
}

void BNO055Module::handleCommand(const String &command) {
  // Handle commands like:
  // "IMU_GET_DATA" - request immediate data
  // "IMU_SET_RATE:sensor_id:rate_ms" - set read rate for specific sensor
  // "IMU_CALIBRATE:sensor_id" - trigger calibration for specific sensor

  if (command == "IMU_GET_DATA") {
    sendIMUData();
  } else if (command.startsWith("IMU_SET_RATE:")) {
    // Parse "IMU_SET_RATE:sensor_id:rate_ms"
    int firstColon = command.indexOf(':', 13);  // After "IMU_SET_RATE:"
    int secondColon = command.indexOf(':', firstColon + 1);

    if (firstColon > 0 && secondColon > 0) {
      int sensor_id = command.substring(13, firstColon).toInt();
      int rate_ms = command.substring(firstColon + 1, secondColon).toInt();

      if (sensor_id >= 0 && sensor_id < (int)sensors_.size() && rate_ms > 0 &&
          rate_ms <= 1000) {
        sensors_[sensor_id].read_interval_ms = rate_ms;
        SerialManager::singleton().SendDiagnosticMessage(
            "[BNO055Module::handleCommand] IMU_RATE_SET:" + String(sensor_id) +
            ":" + String(rate_ms));
      } else {
        SerialManager::singleton().SendDiagnosticMessage(
            "[BNO055Module::handleCommand] IMU_ERROR:Invalid parameters");
      }
    }
  } else if (command.startsWith("IMU_CALIBRATE:")) {
    // Parse "IMU_CALIBRATE:sensor_id"
    int sensor_id = command.substring(14).toInt();

    if (sensor_id >= 0 && sensor_id < (int)sensors_.size() &&
        sensors_[sensor_id].initialized) {
      // Read calibration status
      uint8_t calib_stat;
      if (bno_read(sensors_[sensor_id].address, BNO055_CALIB_STAT, &calib_stat,
                   1)) {
        SerialManager::singleton().SendDiagnosticMessage(
            "[BNO055Module::handleCommand] IMU_CALIB:" + String(sensor_id) +
            ":" + String(calib_stat, HEX));
      } else {
        SerialManager::singleton().SendDiagnosticMessage(
            "[BNO055Module::handleCommand] IMU_ERROR:Calibration read failed");
      }
    } else {
      SerialManager::singleton().SendDiagnosticMessage(
          "[BNO055Module::handleCommand] IMU_ERROR:Invalid sensor ID");
    }
  } else {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BNO055Module::handleCommand] IMU_ERROR:Unknown command");
  }
}

void BNO055Module::convertQuaternionToROS(float bno_w, float bno_x, float bno_y,
                                          float bno_z, float &ros_x,
                                          float &ros_y, float &ros_z,
                                          float &ros_w) {
  // Convert from BNO055 coordinate system to ROS body frame convention
  // BNO055: X=right, Y=forward, Z=up (component side up)
  // ROS: X=forward, Y=left, Z=up
  //
  // Coordinate transformation:
  // BNO055 X (right) → ROS Y (left) with sign flip
  // BNO055 Y (forward) → ROS X (forward)
  // BNO055 Z (up) → ROS Z (up)
  //
  // Quaternion order: BNO055 (w,x,y,z) → ROS (x,y,z,w)

  ros_x = bno_y;   // BNO055 Y → ROS X (forward)
  ros_y = -bno_x;  // BNO055 X → ROS Y (right→left, flip sign)
  ros_z = bno_z;   // BNO055 Z → ROS Z (up)
  ros_w = bno_w;   // Scalar component
}

bool BNO055Module::getIMUDataROS(uint8_t sensor_index, float &qx, float &qy,
                                 float &qz, float &qw, float &gyro_x,
                                 float &gyro_y, float &gyro_z, float &accel_x,
                                 float &accel_y, float &accel_z) {
  if (sensor_index >= sensors_.size()) {
    return false;
  }

  const IMUData &data = sensors_[sensor_index].last_data;
  if (!data.valid) {
    return false;
  }

  // Convert quaternion to ROS format and coordinate system
  convertQuaternionToROS(data.qw, data.qx, data.qy, data.qz, qx, qy, qz, qw);

  // Convert gyroscope data to ROS coordinate system (rad/s)
  gyro_x = data.gy;   // BNO055 Y → ROS X (forward)
  gyro_y = -data.gx;  // BNO055 X → ROS Y (right→left, flip sign)
  gyro_z = data.gz;   // BNO055 Z → ROS Z (up)

  // Convert linear acceleration to ROS coordinate system (m/s²)
  accel_x = data.ay;   // BNO055 Y → ROS X (forward)
  accel_y = -data.ax;  // BNO055 X → ROS Y (right→left, flip sign)
  accel_z = data.az;   // BNO055 Z → ROS Z (up)

  return true;
}

bool BNO055Module::testI2CMultiplexer() {
  // Test if the I2C multiplexer is present and responsive
  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BNO055Module::testI2CMultiplexer] I2C multiplexer found at address "
        "0x" +
        String(I2C_MULTIPLEXER_ADDRESS, HEX));
    return true;
  } else {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BNO055Module::testI2CMultiplexer] I2C multiplexer NOT found at "
        "address 0x" +
        String(I2C_MULTIPLEXER_ADDRESS, HEX) + " (error: " + String(error) +
        ")");
    return false;
  }
}

BNO055Module &BNO055Module::singleton() {
  if (!g_instance_) {
    g_instance_ = new BNO055Module();
  }
  return *g_instance_;
}

BNO055Module *BNO055Module::g_instance_ = nullptr;
