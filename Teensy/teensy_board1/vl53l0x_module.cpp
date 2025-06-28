#include "vl53l0x_module.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Static member definitions
VL53L0XModule* VL53L0XModule::g_instance_ = nullptr;

VL53L0XModule& VL53L0XModule::singleton() {
  if (g_instance_ == nullptr) {
    g_instance_ = new VL53L0XModule();
  }
  return *g_instance_;
}

VL53L0XModule::VL53L0XModule() : Module() {
  // Initialize timing for data transmission
  last_data_send_time_ = 0;
  data_send_interval_ms_ = 10;  // Send data every 10ms
  setup_completed_ = false;

  // Initialize sensor array
  sensors_.resize(ENABLED_SENSORS);
  for (uint8_t i = 0; i < ENABLED_SENSORS; i++) {
    sensors_[i].sensor = nullptr;
    sensors_[i].initialized = false;
    sensors_[i].last_read_time = 0;
    sensors_[i].last_distance_mm = NAN;
    sensors_[i].read_interval_ms = 10;  // Read every 10ms
    sensors_[i].mux_channel = i;
    sensors_[i].i2c_address =
        VL53L0X_DEFAULT_ADDRESS + i;  // Different address for each sensor
  }

  Serial.printf("VL53L0XModule: Configured for %d sensors\n", ENABLED_SENSORS);
}

void VL53L0XModule::setup() {
  if (setup_completed_) {
    return;  // Already completed setup
  }

  Serial.println("VL53L0XModule: Starting sensor initialization...");

  // Initialize I2C if not already done
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  Wire.begin();
  Wire.setClock(400000);  // Set I2C to 400kHz for faster communication

  // Test I2C multiplexer connectivity
  if (!testI2CMultiplexer()) {
    Serial.println(
        "VL53L0XModule: I2C multiplexer test failed - continuing anyway");
  }

  // Initialize each enabled sensor
  uint8_t initialized_count = 0;
  for (uint8_t i = 0; i < ENABLED_SENSORS; i++) {
    Serial.printf("VL53L0XModule: Initializing sensor %d...\n", i);

    if (initializeSensor(i)) {
      initialized_count++;
      Serial.printf("VL53L0XModule: Sensor %d initialized successfully\n", i);
    } else {
      Serial.printf("VL53L0XModule: Sensor %d initialization failed\n", i);
    }
  }

  Serial.printf("VL53L0XModule: %d/%d sensors initialized\n", initialized_count,
                ENABLED_SENSORS);

  setup_completed_ = true;
}

void VL53L0XModule::loop() {
  if (!setup_completed_) {
    return;  // Skip if setup not completed
  }

  uint32_t current_time = millis();

  // Read from each initialized sensor based on its interval
  for (uint8_t i = 0; i < ENABLED_SENSORS; i++) {
    if (sensors_[i].initialized && (current_time - sensors_[i].last_read_time >=
                                    sensors_[i].read_interval_ms)) {
      readSensorDistance(i);
      sensors_[i].last_read_time = current_time;
    }
  }

  // Send data periodically (non-blocking)
  if (current_time - last_data_send_time_ >= data_send_interval_ms_) {
    sendDistanceData();
    last_data_send_time_ = current_time;
  }
}

bool VL53L0XModule::isUnsafe() {
  // Check if any sensor detects an obstacle within safety threshold
  for (const auto& sensor : sensors_) {
    if (sensor.initialized && !isnan(sensor.last_distance_mm)) {
      if (sensor.last_distance_mm < SAFETY_THRESHOLD_MM) {
        return true;  // Obstacle detected within safety threshold
      }
    }
  }
  return false;
}

void VL53L0XModule::resetSafetyFlags() {
  // Reset any safety flags related to the VL53L0X sensors
  // For now, this is just a placeholder as the safety state is derived
  // from current distance measurements
  Serial.println("VL53L0XModule: Safety flags reset");
}

float VL53L0XModule::getDistanceMm(uint8_t sensor_index) {
  if (sensor_index >= ENABLED_SENSORS) {
    return NAN;
  }

  if (!sensors_[sensor_index].initialized) {
    return NAN;
  }

  return sensors_[sensor_index].last_distance_mm;
}

bool VL53L0XModule::isSensorInitialized(uint8_t sensor_index) {
  if (sensor_index >= ENABLED_SENSORS) {
    return false;
  }

  return sensors_[sensor_index].initialized;
}

void VL53L0XModule::selectSensor(uint8_t sensor_index) {
  // Select the appropriate channel on the I2C multiplexer
  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
  Wire.write(1 << sensor_index);
  Wire.endTransmission();

  // Small delay to allow multiplexer to switch
  delayMicroseconds(100);
}

bool VL53L0XModule::initializeSensor(uint8_t sensor_index) {
  if (sensor_index >= ENABLED_SENSORS) {
    return false;
  }

  // Select the sensor through multiplexer
  selectSensor(sensor_index);

  // Create VL53L0X sensor object
  VL53L0X* sensor = new VL53L0X();
  if (sensor == nullptr) {
    Serial.printf("VL53L0X: Failed to create sensor object for sensor %d\n",
                  sensor_index);
    return false;
  }

  // Set I2C bus (use default Wire)
  sensor->setBus(&Wire);

  // Initialize the sensor
  if (!sensor->init()) {
    Serial.printf("VL53L0X: Failed to initialize sensor %d\n", sensor_index);
    delete sensor;
    return false;
  }

  // Set timeout for measurements (500ms)
  sensor->setTimeout(500);

  // Configure the sensor for good accuracy
  // Use high accuracy mode with longer range capability
  sensor->setSignalRateLimit(0.1);
  sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  // Set measurement timing budget to 33ms for good accuracy vs speed balance
  if (!sensor->setMeasurementTimingBudget(33000)) {
    Serial.printf("VL53L0X: Failed to set timing budget for sensor %d\n",
                  sensor_index);
    delete sensor;
    return false;
  }

  // Start continuous ranging with 50ms period (20Hz)
  sensor->startContinuous(50);

  // Change the sensor's I2C address to avoid conflicts when multiple sensors
  // are used Note: This should be done after initialization but before storing
  // the sensor
  sensor->setAddress(sensors_[sensor_index].i2c_address);

  sensors_[sensor_index].sensor = sensor;
  sensors_[sensor_index].initialized = true;

  Serial.printf(
      "VL53L0X: Successfully initialized sensor %d (address 0x%02X)\n",
      sensor_index, sensors_[sensor_index].i2c_address);
  return true;
}

void VL53L0XModule::readSensorDistance(uint8_t sensor_index) {
  if (sensor_index >= ENABLED_SENSORS || !sensors_[sensor_index].initialized) {
    return;
  }

  // Select the sensor
  selectSensor(sensor_index);

  VL53L0X* sensor = sensors_[sensor_index].sensor;
  if (sensor == nullptr) {
    sensors_[sensor_index].last_distance_mm = NAN;
    return;
  }

  // Read distance measurement
  uint16_t distance = sensor->readRangeContinuousMillimeters();

  // Check for timeout or error
  if (sensor->timeoutOccurred()) {
    sensors_[sensor_index].last_distance_mm = NAN;
    // Note: Don't spam error messages for timeouts
  } else {
    // VL53L0X can return readings up to ~2000mm, but values > 1200mm are less
    // reliable We'll accept all values but note that accuracy decreases with
    // distance
    sensors_[sensor_index].last_distance_mm = static_cast<float>(distance);
  }
}

void VL53L0XModule::sendDistanceData() {
  // Send distance data for all enabled sensors
  for (uint8_t i = 0; i < ENABLED_SENSORS; i++) {
    float distance =
        sensors_[i].initialized ? sensors_[i].last_distance_mm : NAN;

    // Format: VL53L0X_DISTANCE:sensor_id,distance_mm
    char message[64];
    if (isnan(distance)) {
      snprintf(message, sizeof(message), "VL53L0X_DISTANCE:%d,NAN", i);
    } else {
      snprintf(message, sizeof(message), "VL53L0X_DISTANCE:%d,%.1f", i,
               distance);
    }

    SerialManager::singleton().SendDiagnosticMessage(message);
  }
}

bool VL53L0XModule::testI2CMultiplexer() {
  // Test if the I2C multiplexer is present and responsive
  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    Serial.printf("VL53L0XModule: I2C multiplexer found at address 0x%02X\n",
                  I2C_MULTIPLEXER_ADDRESS);
    return true;
  } else {
    Serial.printf(
        "VL53L0XModule: I2C multiplexer NOT found at address 0x%02X (error: "
        "%d)\n",
        I2C_MULTIPLEXER_ADDRESS, error);
    return false;
  }
}
