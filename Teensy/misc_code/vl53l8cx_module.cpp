#include "vl53l8cx_module.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "serial_manager.h"

// Static member definitions
VL53L8CXModule* VL53L8CXModule::g_instance_ = nullptr;

VL53L8CXModule& VL53L8CXModule::singleton() {
  if (g_instance_ == nullptr) {
    g_instance_ = new VL53L8CXModule();
  }
  return *g_instance_;
}

VL53L8CXModule::VL53L8CXModule() : Module() {
  // Initialize timing for data transmission
  last_data_send_time_ = 0;
  data_send_interval_ms_ = 10;  // Send data every 10ms
  setup_completed_ = false;     // Track setup completion

  // Initialize sensor array
  sensors_.resize(ENABLED_SENSORS);
  for (uint8_t i = 0; i < ENABLED_SENSORS; i++) {
    sensors_[i].sensor = nullptr;
    sensors_[i].initialized = false;
    sensors_[i].last_read_time = 0;
    sensors_[i].last_distance_mm = NAN;
    sensors_[i].read_interval_ms = 10;  // Read every 10ms
    sensors_[i].mux_channel = i;
    sensors_[i].resolution =
        VL53L8CX_RESOLUTION_4X4;  // Default 4x4 resolution for faster readings
  }

  Serial.printf("VL53L8CXModule: Configured for %d sensors\n", ENABLED_SENSORS);
}

void VL53L8CXModule::setup() {
  if (setup_completed_) {
    Serial.println("setup: Setup already completed, skipping...");
    return;
  }

  Serial.println("setup: Starting sensor initialization...");

  // Initialize I2C if not already done
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C speed

  // Initialize each enabled sensor
  uint8_t initialized_count = 0;
  for (uint8_t i = 0; i < ENABLED_SENSORS; i++) {
    Serial.printf("setup: Attempting to initialize sensor %d...\n", i);

    if (initializeSensor(i)) {
      initialized_count++;
      Serial.printf("setup: Sensor %d initialized successfully\n", i);
    } else {
      Serial.printf("setup: Sensor %d initialization failed\n", i);
    }
  }

  Serial.printf("setup: %d/%d sensors initialized\n", initialized_count,
                ENABLED_SENSORS);

  if (initialized_count == 0) {
    Serial.println(
        "setup: WARNING - No sensors initialized! Check hardware "
        "connections.");
    Serial.println("setup: Troubleshooting tips:");
    Serial.println(
        "  - Verify VL53L8CX sensors are connected to multiplexer channels "
        "0-7");
    Serial.println("  - Check VL53L8CX power supply (3.3V)");
    Serial.println("  - Verify VL53L8CX I2C address (default 0x29)");
    Serial.println("  - Try with only one sensor connected");
  }

  setup_completed_ = true;  // Mark setup as completed
}

void VL53L8CXModule::loop() {
  uint32_t current_time = millis();

  // Read from each initialized sensor based on its interval
  for (uint8_t i = 0; i < ENABLED_SENSORS; i++) {
    if (sensors_[i].initialized && (current_time - sensors_[i].last_read_time >=
                                    sensors_[i].read_interval_ms)) {
      readSensorDistance(i);
      sensors_[i].last_read_time = current_time;
    }
  }

  // Send data periodically
  if (current_time - last_data_send_time_ >= data_send_interval_ms_) {
    sendDistanceData();
    last_data_send_time_ = current_time;
  }
}

bool VL53L8CXModule::isUnsafe() {
  // Check if any sensor is reporting unsafe conditions
  // For distance sensors, this could be detecting obstacles too close
  for (const auto& sensor : sensors_) {
    if (sensor.initialized && !isnan(sensor.last_distance_mm)) {
      // Example: Consider distances less than 50mm as unsafe
      if (sensor.last_distance_mm < 50.0f) {
        return true;
      }
    }
  }
  return false;
}

void VL53L8CXModule::resetSafetyFlags() {
  // Reset any safety-related state
  // For distance sensors, this might clear error conditions
  Serial.println("VL53L8CXModule: Safety flags reset");
}

float VL53L8CXModule::getDistanceMm(uint8_t sensor_index) {
  if (sensor_index >= ENABLED_SENSORS) {
    return NAN;
  }

  if (!sensors_[sensor_index].initialized) {
    return NAN;
  }

  return sensors_[sensor_index].last_distance_mm;
}

bool VL53L8CXModule::isSensorInitialized(uint8_t sensor_index) {
  if (sensor_index >= ENABLED_SENSORS) {
    return false;
  }

  return sensors_[sensor_index].initialized;
}

void VL53L8CXModule::selectSensor(uint8_t sensor_index) {
  if (sensor_index >= MAX_SENSORS) {
    return;
  }

  // Select the sensor through the I2C multiplexer
  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
  Wire.write(1 << sensor_index);
  Wire.endTransmission();

  // Small delay to allow multiplexer to switch
  delayMicroseconds(100);
}

bool VL53L8CXModule::initializeSensor(uint8_t sensor_index) {
  if (sensor_index >= ENABLED_SENSORS) {
    return false;
  }

  Serial.printf("initializeSensor: Selecting multiplexer channel %d...\n",
                sensor_index);

  // Select the sensor through multiplexer
  selectSensor(sensor_index);

  // Test if sensor responds on I2C bus
  Wire.beginTransmission(0x29);  // Default VL53L8CX I2C address
  uint8_t sensor_error = Wire.endTransmission();

  if (sensor_error != 0) {
    Serial.printf(
        "initializeSensor: Sensor %d not responding on I2C (error: %d)\n",
        sensor_index, sensor_error);
    return false;
  }

  Serial.printf(
      "initializeSensor: Sensor %d responding on I2C, initializing...\n",
      sensor_index);

  // Create VL53L8CX configuration object using the API
  VL53L8CX_Configuration* sensor = new VL53L8CX_Configuration();
  if (sensor == nullptr) {
    Serial.printf(
        "initializeSensor: Failed to create sensor object for sensor %d\n",
        sensor_index);
    return false;
  }

  // Initialize the sensor using the API
  Serial.printf("initializeSensor: Calling vl53l8cx_init for sensor %d...\n",
                sensor_index);
  uint8_t status = vl53l8cx_init(sensor);
  if (status != VL53L8CX_STATUS_OK) {
    Serial.printf(
        "initializeSensor: vl53l8cx_init failed for sensor %d (status: %d)\n",
        sensor_index, status);
    delete sensor;
    return false;
  }

  Serial.printf("initializeSensor: vl53l8cx_init successful for sensor %d\n",
                sensor_index);

  // Set resolution (4x4 for faster updates)
  status = vl53l8cx_set_resolution(sensor, sensors_[sensor_index].resolution);
  if (status != VL53L8CX_STATUS_OK) {
    Serial.printf("initializeSensor: Failed to set resolution for sensor %d\n",
                  sensor_index);
    delete sensor;
    return false;
  }

  // Set ranging frequency to 30Hz (good balance of speed vs power)
  status = vl53l8cx_set_ranging_frequency_hz(sensor, 30);
  if (status != VL53L8CX_STATUS_OK) {
    Serial.printf(
        "initializeSensor: Failed to set ranging frequency for sensor %d\n",
        sensor_index);
    delete sensor;
    return false;
  }

  // Start ranging
  status = vl53l8cx_start_ranging(sensor);
  if (status != VL53L8CX_STATUS_OK) {
    Serial.printf("initializeSensor: Failed to start ranging for sensor %d\n",
                  sensor_index);
    delete sensor;
    return false;
  }

  sensors_[sensor_index].sensor = sensor;
  sensors_[sensor_index].initialized = true;

  Serial.printf("initializeSensor: Successfully initialized sensor %d\n",
                sensor_index);
  return true;
}

void VL53L8CXModule::readSensorDistance(uint8_t sensor_index) {
  if (sensor_index >= ENABLED_SENSORS || !sensors_[sensor_index].initialized) {
    return;
  }

  // Select the sensor
  selectSensor(sensor_index);

  // Check if new data is available
  VL53L8CX_Configuration* sensor = sensors_[sensor_index].sensor;
  if (sensor == nullptr) {
    sensors_[sensor_index].last_distance_mm = NAN;
    return;
  }

  uint8_t data_ready = 0;
  uint8_t status = vl53l8cx_check_data_ready(sensor, &data_ready);

  if (status != VL53L8CX_STATUS_OK) {
    sensors_[sensor_index].last_distance_mm = NAN;
    return;
  }

  if (data_ready) {
    VL53L8CX_ResultsData results;
    status = vl53l8cx_get_ranging_data(sensor, &results);

    if (status != VL53L8CX_STATUS_OK) {
      sensors_[sensor_index].last_distance_mm = NAN;
      Serial.printf("VL53L8CX: Failed to get ranging data from sensor %d\n",
                    sensor_index);
    } else {
      // Use center zone distance (zone 7 for 4x4, zone 27 for 8x8)
      uint8_t center_zone =
          (sensors_[sensor_index].resolution == VL53L8CX_RESOLUTION_4X4) ? 7
                                                                         : 27;

      // Check if the measurement is valid using target status
      if (results.target_status[center_zone] == VL53L8CX_TARGET_STATUS_RANGED) {
        sensors_[sensor_index].last_distance_mm =
            static_cast<float>(results.distance_mm[center_zone]);
      } else {
        sensors_[sensor_index].last_distance_mm = NAN;
      }
    }
  }
}

void VL53L8CXModule::sendDistanceData() {
  // Send distance data for all enabled sensors
  for (uint8_t i = 0; i < ENABLED_SENSORS; i++) {
    float distance =
        sensors_[i].initialized ? sensors_[i].last_distance_mm : NAN;

    // Format: VL53L8CX_DISTANCE:sensor_id,distance_mm
    char message[64];
    if (isnan(distance)) {
      snprintf(message, sizeof(message), "VL53L8CX_DISTANCE:%d,NAN", i);
    } else {
      snprintf(message, sizeof(message), "VL53L8CX_DISTANCE:%d,%.1f", i,
               distance);
    }

    SerialManager::singleton().SendDiagnosticMessage(message);
  }
}
