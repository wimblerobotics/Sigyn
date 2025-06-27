#include "bno055_module.h"
#include "serial_manager.h"
#include <Wire.h>
#include <Arduino.h>

const float RAD_PER_DEG = 0.0174533f; // π/180

BNO055Module::BNO055Module() : Module() {
    // Initialize timing for data transmission
    last_data_send_time_ = 0;
    data_send_interval_ms_ = 1000; // Send data every 100ms (10Hz)
    
    // Initialize I2C
    Wire.begin(); // This function is called in the main program setup.
    Wire.setClock(100000); // Start with 100kHz for better reliability
    
    // Setup chips
    // chips_.resize(2); // Support 2 BNO055 chips
    chips_.resize(2); // Support 1 BNO055 chips
    chips_[0].address = BNO055_ADDRESS_A;
    chips_[0].read_interval_ms = 10; // Read every 10ms
    chips_[1].address = BNO055_ADDRESS_B;
    chips_[1].read_interval_ms = 10; // Read every 10ms
    
    delay(1000); // Give sensors time to power up
    
    // Initialize each chip
    for (auto& chip : chips_) {
        chip.initialized = bno_setup(chip.address);
        chip.last_read_time = 0;
        chip.last_data.valid = false;
        
        if (chip.initialized) {
            Serial.printf("BNO055 at 0x%02X initialized successfully\n", chip.address);
        } else {
            Serial.printf("BNO055 at 0x%02X initialization failed\n", chip.address);
        }
    }
    
    // Increase I2C speed after initialization
    Wire.setClock(400000);
}

void BNO055Module::setup() {
    // Nothing needed here since initialization is done in constructor
}

void BNO055Module::loop() {
    uint32_t current_time = millis();
    
    // Read from each initialized chip based on its interval
    for (auto& chip : chips_) {
        if (chip.initialized && 
            (current_time - chip.last_read_time >= chip.read_interval_ms)) {
            
            // Read IMU data (this should be fast, < 2ms)
            read_imu_data(chip.address, &chip.last_data);
            chip.last_read_time = current_time;
            // Serial.printf("Read data from BNO055 at 0x%02X\n", chip.address);
            // Serial.printf("Data: Q=%.4f %.4f %.4f %.4f G=%.2f %.2f %.2f A=%.2f %.2f %.2f E=%.2f %.2f %.2f\n",
            //     chip.last_data.qw, chip.last_data.qx, chip.last_data.qy, chip.last_data.qz,
            //     chip.last_data.gx, chip.last_data.gy, chip.last_data.gz,
            //     chip.last_data.ax, chip.last_data.ay, chip.last_data.az,
            //     chip.last_data.euler_h, chip.last_data.euler_r, chip.last_data.euler_p);
        } else {
            // Skip reading if not initialized or not time yet
            if (!chip.initialized) {
                Serial.printf("BNO055 at 0x%02X is not initialized\n", chip.address);
                delay(10'000); // Wait before retrying
            } else {
                // Serial.printf("Skipping read for BNO055 at 0x%02X, not time yet\n", chip.address);
                // delay(1000); // Wait before retrying

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
    for (const auto& chip : chips_) {
        if (chip.initialized && !chip.last_data.valid) {
            return true; // Sensor failure detected
        }
    }
    return false;
}

void BNO055Module::resetSafetyFlags() {
    // Reset any safety flags related to the BNO055 sensors
    // Could implement sensor reset logic here if needed
}

bool BNO055Module::getIMUData(uint8_t chip_index, IMUData &data) {
    if (chip_index >= chips_.size()) {
        return false;
    }
    
    if (!chips_[chip_index].initialized) {
        return false;
    }
    
    data = chips_[chip_index].last_data;
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
    
    delayMicroseconds(150); // BNO055 needs time between register writes
    return true;
}

bool BNO055Module::bno_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    uint8_t result = Wire.endTransmission(false); // Send restart
    
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
    uint8_t chip_id;
    if (!bno_read(addr, BNO055_CHIP_ID, &chip_id, 1)) {
        return false;
    }
    return chip_id == BNO055_ID;
}

bool BNO055Module::bno_setup(uint8_t addr) {
    // Check if device is present and has correct ID
    if (!bno_check_id(addr)) {
        return false;
    }
    
    // Set to config mode first
    if (!bno_write(addr, BNO055_OPR_MODE, OPERATION_MODE_CONFIG)) {
        return false;
    }
    delay(25);
    
    // Reset the system
    if (!bno_write(addr, BNO055_SYS_TRIGGER, 0x20)) {
        return false;
    }
    delay(700);
    
    // Check if device is still there after reset
    if (!bno_check_id(addr)) {
        return false;
    }
    
    // Set normal power mode
    if (!bno_write(addr, BNO055_PWR_MODE, 0x00)) {
        return false;
    }
    delay(10);
    
    // Select page 0
    if (!bno_write(addr, BNO055_PAGE_ID, 0x00)) {
        return false;
    }
    delay(10);
    
    // Set NDOF fusion mode
    if (!bno_write(addr, BNO055_OPR_MODE, OPERATION_MODE_NDOF)) {
        return false;
    }
    delay(100);
    
    return true;
}

bool BNO055Module::read_quaternion(uint8_t addr, float *w, float *x, float *y, float *z) {
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

bool BNO055Module::read_linear_accel(uint8_t addr, float *x, float *y, float *z) {
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

bool BNO055Module::read_euler(uint8_t addr, float *heading, float *roll, float *pitch) {
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
    
    data->valid = read_quaternion(addr, &data->qw, &data->qx, &data->qy, &data->qz)
               && read_gyro(addr, &data->gx, &data->gy, &data->gz)
               && read_linear_accel(addr, &data->ax, &data->ay, &data->az)
               && read_euler(addr, &data->euler_h, &data->euler_r, &data->euler_p);
    
    uint32_t end_time = micros();
    data->read_time_tenths_ms = (end_time - start_time) / 100;
    
    return data->valid;
}

void BNO055Module::sendIMUData() {
    // Send data from all initialized chips in ROS format
    for (uint8_t i = 0; i < chips_.size(); i++) {
        if (chips_[i].initialized && chips_[i].last_data.valid) {
            const IMUData& data = chips_[i].last_data;
            
            // Convert to ROS format
            float ros_qx, ros_qy, ros_qz, ros_qw;
            convertQuaternionToROS(data.qw, data.qx, data.qy, data.qz, ros_qx, ros_qy, ros_qz, ros_qw);
            
            // Convert other data to ROS coordinate system
            float ros_gx = data.gy;    // BNO055 Y → ROS X
            float ros_gy = -data.gx;   // BNO055 X → ROS Y (flip sign)
            float ros_gz = data.gz;    // BNO055 Z → ROS Z
            
            float ros_ax = data.ay;    // BNO055 Y → ROS X
            float ros_ay = -data.ax;   // BNO055 X → ROS Y (flip sign)
            float ros_az = data.az;    // BNO055 Z → ROS Z
            
            // Format: IMU_DATA:chip_id,qx,qy,qz,qw,gx,gy,gz,ax,ay,az,h,r,p,read_time (ROS format)
            char message[256];
            snprintf(message, sizeof(message),
                "IMU_DATA:%d,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%d",
                (int)i,
                ros_qx, ros_qy, ros_qz, ros_qw,  // ROS quaternion order: x,y,z,w
                ros_gx, ros_gy, ros_gz,          // ROS coordinate system
                ros_ax, ros_ay, ros_az,          // ROS coordinate system
                data.euler_h, data.euler_r, data.euler_p,  // Keep Euler as-is for now
                (int)data.read_time_tenths_ms);
            
            SerialManager::singleton().SendDiagnosticMessage(message);
        }
    }
}

void BNO055Module::handleCommand(const String& command) {
    // Handle commands like:
    // "IMU_GET_DATA" - request immediate data
    // "IMU_SET_RATE:chip_id:rate_ms" - set read rate for specific chip
    // "IMU_CALIBRATE:chip_id" - trigger calibration for specific chip
    
    if (command == "IMU_GET_DATA") {
        sendIMUData();
    } else if (command.startsWith("IMU_SET_RATE:")) {
        // Parse "IMU_SET_RATE:chip_id:rate_ms"
        int firstColon = command.indexOf(':', 13); // After "IMU_SET_RATE:"
        int secondColon = command.indexOf(':', firstColon + 1);
        
        if (firstColon > 0 && secondColon > 0) {
            int chip_id = command.substring(13, firstColon).toInt();
            int rate_ms = command.substring(firstColon + 1, secondColon).toInt();
            
            if (chip_id >= 0 && chip_id < (int)chips_.size() && rate_ms > 0 && rate_ms <= 1000) {
                chips_[chip_id].read_interval_ms = rate_ms;
                SerialManager::singleton().SendDiagnosticMessage(
                    "IMU_RATE_SET:" + String(chip_id) + ":" + String(rate_ms));
            } else {
                SerialManager::singleton().SendDiagnosticMessage("IMU_ERROR:Invalid parameters");
            }
        }
    } else if (command.startsWith("IMU_CALIBRATE:")) {
        // Parse "IMU_CALIBRATE:chip_id"
        int chip_id = command.substring(14).toInt();
        
        if (chip_id >= 0 && chip_id < (int)chips_.size() && chips_[chip_id].initialized) {
            // Read calibration status
            uint8_t calib_stat;
            if (bno_read(chips_[chip_id].address, BNO055_CALIB_STAT, &calib_stat, 1)) {
                SerialManager::singleton().SendDiagnosticMessage(
                    "IMU_CALIB:" + String(chip_id) + ":" + String(calib_stat, HEX));
            } else {
                SerialManager::singleton().SendDiagnosticMessage("IMU_ERROR:Calibration read failed");
            }
        } else {
            SerialManager::singleton().SendDiagnosticMessage("IMU_ERROR:Invalid chip ID");
        }
    } else {
        SerialManager::singleton().SendDiagnosticMessage("IMU_ERROR:Unknown command");
    }
}

void BNO055Module::convertQuaternionToROS(float bno_w, float bno_x, float bno_y, float bno_z,
                                         float &ros_x, float &ros_y, float &ros_z, float &ros_w) {
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

bool BNO055Module::getIMUDataROS(uint8_t chip_index, float &qx, float &qy, float &qz, float &qw,
                                 float &gyro_x, float &gyro_y, float &gyro_z,
                                 float &accel_x, float &accel_y, float &accel_z) {
    if (chip_index >= chips_.size()) {
        return false;
    }
    
    const IMUData& data = chips_[chip_index].last_data;
    if (!data.valid) {
        return false;
    }
    
    // Convert quaternion to ROS format and coordinate system
    convertQuaternionToROS(data.qw, data.qx, data.qy, data.qz, qx, qy, qz, qw);
    
    // Convert gyroscope data to ROS coordinate system (rad/s)
    gyro_x = data.gyro_y;   // BNO055 Y → ROS X (forward)
    gyro_y = -data.gyro_x;  // BNO055 X → ROS Y (right→left, flip sign)
    gyro_z = data.gyro_z;   // BNO055 Z → ROS Z (up)
    
    // Convert linear acceleration to ROS coordinate system (m/s²)
    accel_x = data.accel_y;   // BNO055 Y → ROS X (forward)
    accel_y = -data.accel_x;  // BNO055 X → ROS Y (right→left, flip sign)
    accel_z = data.accel_z;   // BNO055 Z → ROS Z (up)
    
    return true;
}

BNO055Module &BNO055Module::singleton() {
    if (!g_instance_) {
        g_instance_ = new BNO055Module();
    }
    return *g_instance_;
}

BNO055Module *BNO055Module::g_instance_ = nullptr;
