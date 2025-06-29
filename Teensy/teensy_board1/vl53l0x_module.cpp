#include "vl53l0x_module.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "serial_manager.h"

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
  data_send_interval_ms_ = 50;  // Send data after this period.
  setup_completed_ = false;
  multiplexer_available_ = false;

  // Initialize sensor array
  sensors_.resize(ENABLED_SENSORS);
  for (uint8_t i = 0; i < ENABLED_SENSORS; i++) {
    sensors_[i].sensor = nullptr;
    sensors_[i].initialized = false;
    sensors_[i].last_read_time = 0;
    sensors_[i].last_distance_mm = NAN;
    sensors_[i].read_interval_ms = 50;
    sensors_[i].mux_channel = i;
  }

  SerialManager::singleton().SendDiagnosticMessage(
      String("VL53L0XModule: Configured for ") + String(ENABLED_SENSORS) + String(" sensors"));
}

void VL53L0XModule::setup() {
  if (setup_completed_) {
    return;  // Already completed setup
  }

  SerialManager::singleton().SendDiagnosticMessage(
      "VL53L0XModule: Starting sensor initialization...");

  // Initialize I2C if not already done
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  Wire.begin();
  Wire.setClock(400000);  // Set I2C to 400kHz for faster communication

  // Test I2C multiplexer connectivity
  multiplexer_available_ = testI2CMultiplexer();
  if (!multiplexer_available_) {
    SerialManager::singleton().SendDiagnosticMessage(
        "VL53L0XModule: I2C multiplexer test failed, no sensors will be "
        "initialized.");
    return;
  }

  // Initialize each enabled sensor
  uint8_t initialized_count = 0;
  for (uint8_t i = 0; i < ENABLED_SENSORS; i++) {
    if (initializeSensor(i)) {
      initialized_count++;
      SerialManager::singleton().SendDiagnosticMessage(
          "VL53L0XModule: Sensor " + String(i) + " initialized successfully");
    } else {
      SerialManager::singleton().SendDiagnosticMessage(
          "VL53L0XModule: Sensor " + String(i) + " initialization failed");
    }

    // Add a small delay between sensor initializations
    delay(1);
  }

  SerialManager::singleton().SendDiagnosticMessage(
      "VL53L0XModule: " + String(initialized_count) + "/" + String(ENABLED_SENSORS) + " sensors initialized");

  setup_completed_ = true;
}

void VL53L0XModule::loop() {
  if (!setup_completed_ || !multiplexer_available_) {
    return;  // Skip if setup not completed
  }

  uint32_t current_time = millis();

  // Use a static variable to keep track of the next sensor to read
  // This allows us to read each sensor in turn without blocking
  // and ensures that we don't read the same sensor too frequently.
  static uint8_t next_sensor_index = 0;

  // Read from the next sensor in the list
  if (sensors_[next_sensor_index].initialized &&
      (current_time - sensors_[next_sensor_index].last_read_time >=
       sensors_[next_sensor_index].read_interval_ms)) {
    readSensorDistance(next_sensor_index);
    sensors_[next_sensor_index].last_read_time = current_time;
  }

  // Move to the next sensor (wrap around if necessary)
  next_sensor_index = (next_sensor_index + 1) % ENABLED_SENSORS;

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
  SerialManager::singleton().SendDiagnosticMessage(
      "VL53L0XModule: Safety flags reset");
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
  // Only use multiplexer if it's available
  if (multiplexer_available_) {
    // Select the appropriate channel on the I2C multiplexer
    Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
    Wire.write(1 << sensor_index);
    Wire.endTransmission();

    // Small delay to allow multiplexer to switch
    delayMicroseconds(100);
  }
  // If no multiplexer, assume sensor is connected directly to I2C bus
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
    SerialManager::singleton().SendDiagnosticMessage(
        "VL53L0X: Failed to create sensor object for sensor " + String(sensor_index));
    return false;
  }

  // Set I2C bus (use default Wire)
  sensor->setBus(&Wire);

  // Initialize the sensor
  if (!sensor->init()) {
    SerialManager::singleton().SendDiagnosticMessage(
        "VL53L0X: Failed to initialize sensor " + String(sensor_index));
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
    SerialManager::singleton().SendDiagnosticMessage(
        "VL53L0X: Failed to set timing budget for sensor " + String(sensor_index));
    delete sensor;
    return false;
  }

  // Start continuous ranging with 50ms period (20Hz)
  sensor->startContinuous(50);

  sensors_[sensor_index].sensor = sensor;
  sensors_[sensor_index].initialized = true;
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

  // Note: readRangeContinuousMillimeters() is a blocking call that waits for
  // the measurement to complete. It can take up to 50ms depending on the
  // sensor configuration and distance to target.
  // If you wait 55ms betweeen calls, however, it will not block
  // and take about 0.325 ms.
  uint16_t distance = sensor->readRangeContinuousMillimeters();

  // Check for timeout or error
  if (sensor->timeoutOccurred()) {
    sensors_[sensor_index].last_distance_mm = NAN;
    // Only log timeout errors occasionally to avoid spam
    static uint32_t last_timeout_log = 0;
    uint32_t current_time = millis();
    if (current_time - last_timeout_log >
        5000) {  // Log timeout every 5 seconds max
      SerialManager::singleton().SendDiagnosticMessage(
          "VL53L0X: Sensor " + String(sensor_index) + " timeout");
      last_timeout_log = current_time;
    }
  } else {
    // VL53L0X can return readings up to ~2000mm, but values > 1200mm are less
    // reliable We'll accept all values but note that accuracy decreases with
    // distance
    sensors_[sensor_index].last_distance_mm = static_cast<float>(distance);
  }
}

void VL53L0XModule::sendDistanceData() {
  // Send distance data for all enabled sensors
  String message = "VL53L0X_DISTANCES:";

  uint8_t active_sensors = 0;
  for (uint8_t i = 0; i < ENABLED_SENSORS; i++) {
    float distance =
        sensors_[i].initialized ? sensors_[i].last_distance_mm : NAN;

    // Count active sensors for debugging
    if (sensors_[i].initialized) {
      active_sensors++;
    }

    // Append sensor data to message
    if (isnan(distance)) {
      message += String(i) + ":NAN";
    } else {
      message += String(i) + ":" + String(distance, 1);
    }

    // Add separator if not the last sensor
    if (i < ENABLED_SENSORS - 1) {
      message += ",";
    }
  }

  // Add active sensor count for debugging
  message += ",[" + String(active_sensors) + "/" + String(ENABLED_SENSORS) + " active]";

  SerialManager::singleton().SendDiagnosticMessage(message);
}

bool VL53L0XModule::testI2CMultiplexer() {
  // Test if the I2C multiplexer is present and responsive
  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    SerialManager::singleton().SendDiagnosticMessage(
        "VL53L0XModule: I2C multiplexer found at address 0x" + String(I2C_MULTIPLEXER_ADDRESS, HEX));
    return true;
  } else {
    SerialManager::singleton().SendDiagnosticMessage(
        "VL53L0XModule: I2C multiplexer NOT found at address 0x" + String(I2C_MULTIPLEXER_ADDRESS, HEX) + 
        " (error: " + String(error) + ")");
    return false;
  }
}
