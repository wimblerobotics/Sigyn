/**
 * @file battery_module.cpp
 * @brief Implementation of battery monitoring module for Teensy board 2.
 * 
 * This module provides comprehensive battery monitoring using a hybrid approach:
 * - Main battery (index 0): Analog voltage divider reading via MAIN_BATTERY_PIN
 * - Additional batteries: INA226 sensors accessed via I2C multiplexer
 * 
 * Features:
 * - Per-battery circular buffer averaging for noise reduction
 * - Independent monitoring of multiple battery systems
 * - Safety monitoring with configurable voltage thresholds
 * - Periodic status reporting via serial diagnostic messages
 * - I2C multiplexer management for multiple sensor access
 * 
 * @author Sigyn Robotics
 * @date 2025
 */

#include "battery_module.h"

#include "Arduino.h"         // For analogRead, millis, pinMode, etc.
#include "INA226.h"          // For INA226 battery monitoring sensors
#include "config.h"          // For battery configuration constants
#include "serial_manager.h"  // For sending diagnostic messages

/**
 * @brief Constructs a new BatteryModule instance.
 * 
 * Initializes all per-battery data structures including circular buffers,
 * index counters, and reading counts. Sets up diagnostic messaging and
 * prepares the module for hardware initialization.
 * 
 * @note This constructor is private to enforce the singleton pattern.
 */
BatteryModule::BatteryModule()
    : Module(),
      multiplexer_available_(false),
      setup_completed_(false) {
  // Initialize per-battery data structures
  for (size_t device = 0; device < kNumberOfBatteries; device++) {
    // Clear circular buffer for voltage averaging
    for (size_t reading = 0; reading < kNumberReadingsToAverage_; reading++) {
      g_averages_[device][reading] = 0.0;
    }
    g_next_average_index_[device] = 0;      // Reset circular buffer index
    total_battery_readings_[device] = 0;    // Reset reading counter
  }

  // Log successful configuration
  SerialManager::singleton().SendDiagnosticMessage(
      String("[BatteryModule::BatteryModule] Configured for ") +
      String(kNumberOfBatteries) + String(" sensors"));
}

/**
 * @brief Calculates battery state of charge percentage from voltage.
 * 
 * Uses linear interpolation between configured minimum and maximum battery
 * voltages to estimate the current state of charge. Clamps results to 0-100%.
 * 
 * @param voltage The measured battery voltage in volts.
 * @return Estimated state of charge as percentage (0.0-100.0).
 * 
 * @note Assumes linear discharge curve which may not be accurate for all
 *       battery chemistries. Consider implementing lookup tables for better accuracy.
 */
float BatteryModule::calculatePercentage(float voltage) {
  if (voltage >= MAIN_BATTERY_MAX_VOLTAGE) return 100.0f;
  if (voltage <= MAIN_BATTERY_MIN_VOLTAGE) return 0.0f;

  return (voltage - MAIN_BATTERY_MIN_VOLTAGE) / MAIN_BATTERY_LIPO_CELLS * 100.0f;
}

/**
 * @brief Reads current consumption from the specified battery.
 * 
 * Selects the appropriate INA226 sensor via I2C multiplexer and reads
 * the current measurement. Handles sensor selection and error conditions.
 * 
 * @param battery The battery type to read current from.
 * @return Current consumption in amperes, or 0.0f on error.
 */
float BatteryModule::getCurrent(BatteryType battery) const {
  // Validate battery index
  if (battery >= kNumberOfBatteries) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BatteryModule::getCurrent] Invalid battery index: " +
        String(static_cast<int>(battery)));
    return 0.0f;
  }

  // Select appropriate sensor and read current
  selectSensor(battery);
  return g_ina226_[battery].getCurrent();
}

/**
 * @brief Gets the averaged voltage reading for the specified battery.
 * 
 * Calculates the average voltage from the circular buffer of recent readings.
 * The number of samples used depends on how many readings have been taken
 * since initialization (up to kNumberReadingsToAverage_).
 * 
 * @param battery The battery type to read voltage from.
 * @return Averaged voltage in volts, or 0.0f if invalid battery index.
 * 
 * @note Returns 0.0f if no readings have been taken yet.
 */
float BatteryModule::getVoltage(BatteryType battery) const {
  // Validate battery index
  if (battery >= kNumberOfBatteries) {
    return 0.0f;
  }
  
  // Calculate average from available readings
  float voltage_sum = 0.0f;
  size_t number_readings_to_average =
      total_battery_readings_[battery] < kNumberReadingsToAverage_
          ? total_battery_readings_[battery]
          : kNumberReadingsToAverage_;
  
  for (size_t reading = 0; reading < number_readings_to_average; reading++) {
    voltage_sum += g_averages_[battery][reading];
  }

  return number_readings_to_average > 0 
         ? voltage_sum / number_readings_to_average 
         : 0.0f;
}

/**
 * @brief Checks if any battery is in a critical safety condition.
 * 
 * Iterates through all configured batteries and checks their voltage
 * against the critical threshold. Returns true if any battery requires
 * immediate attention or emergency shutdown.
 * 
 * @return true if any battery voltage is below critical threshold.
 * 
 * @note This method triggers safety protocols when batteries are dangerously low.
 */
bool BatteryModule::isUnsafe() {
  for (size_t battery_idx = 0; battery_idx < kNumberOfBatteries; battery_idx++) {
    if (getVoltage(static_cast<BatteryType>(battery_idx)) < MAIN_BATTERY_CRITICAL_VOLTAGE_THRESHOLD) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Main processing loop for battery monitoring operations.
 * 
 * Performs two main functions on separate timing intervals:
 * 1. Battery reading: Samples all batteries and updates circular buffers
 * 2. Status reporting: Sends diagnostic messages with current battery state
 * 
 * Reading strategy:
 * - Battery 0: Analog voltage divider via MAIN_BATTERY_PIN
 * - Battery 1+: INA226 bus voltage via I2C multiplexer
 * 
 * @note Called periodically by the base Module class framework.
 */
void BatteryModule::loop() {
  static uint32_t last_message_send_time_ms_ = millis();
  static uint32_t last_battery_read_time_ms_ = millis();
  unsigned long current_time_ms = millis();
  
  // Battery reading phase - sample all batteries
  if ((current_time_ms - last_battery_read_time_ms_) > MAIN_BATTERY_READ_INTERVAL_MS) {
    for (size_t battery_idx = 0; battery_idx < kNumberOfBatteries; battery_idx++) {
      float battery_v = 0.0f;
      
      if (battery_idx == 0) {
        // Main battery: analog voltage divider reading
        float raw = analogRead(MAIN_BATTERY_PIN);
        battery_v = raw * k36VAnalogToVoltageConversion;  // Use named constant
      } else {
        // Additional batteries: INA226 bus voltage reading
        selectSensor(static_cast<BatteryType>(battery_idx));
        battery_v = g_ina226_[battery_idx].getBusVoltage();
      }
      
      // Store reading in circular buffer
      g_averages_[battery_idx][g_next_average_index_[battery_idx]] = battery_v;
      
      // Advance circular buffer index
      g_next_average_index_[battery_idx]++;
      if (g_next_average_index_[battery_idx] >= kNumberReadingsToAverage_) {
        g_next_average_index_[battery_idx] = 0;
      }
      
      // Update total reading count (used for averaging)
      total_battery_readings_[battery_idx]++;
    }
    
    last_battery_read_time_ms_ = current_time_ms;
  }

  // Status reporting phase - send diagnostic messages
  if ((current_time_ms - last_message_send_time_ms_) > MAIN_BATTERY_REPORT_INTERVAL_MS) {
    for (size_t device_index = 0; device_index < kNumberOfBatteries; device_index++) {
      sendBatteryState(device_index);
    }
    last_message_send_time_ms_ = current_time_ms;
  }
}

/**
 * @brief Resets safety warning flags and conditions.
 * 
 * Clears any safety-related warning states when battery conditions improve.
 * Currently a placeholder for future safety flag implementation.
 * 
 * @todo Implement actual safety flag management when warning system is added.
 */
void BatteryModule::resetSafetyFlags() {
  // TODO: Implement safety flag reset logic when warning flags are added
}

/**
 * @brief Selects the specified battery sensor via I2C multiplexer.
 * 
 * Configures the I2C multiplexer to route communication to the INA226
 * sensor associated with the specified battery. Includes a settling delay
 * to ensure the multiplexer has switched channels properly.
 * 
 * @param battery The battery type whose sensor should be selected.
 * 
 * @note The 100μs delay is hardware-specific and may need adjustment
 *       for different multiplexer models.
 */
void BatteryModule::selectSensor(BatteryType battery) const {
  // Configure I2C multiplexer channel selection
  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
  Wire.write(1 << gINA226_DeviceIndexes_[battery]);  // Set channel bit
  Wire.endTransmission();

  // Allow multiplexer to settle on new channel
  delayMicroseconds(100);
}

/**
 * @brief Constructs and sends a formatted battery status message.
 * 
 * Creates a diagnostic message containing voltage, state of charge percentage,
 * and current consumption for the specified battery. Message format:
 * "BATTERY:index,voltage,percentage,current"
 * 
 * @param device_index The battery index to report (0-based).
 * 
 * @note Message buffer is limited to 64 characters. Consider increasing
 *       size if additional data fields are needed.
 */
void BatteryModule::sendBatteryState(uint8_t device_index) {
  // Validate device index
  if (device_index >= kNumberOfBatteries) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BatteryModule::sendBatteryState] Invalid device index: " +
        String(device_index));
    return;
  }

  // Format battery status message
  char message[64];
  BatteryType battery_type = static_cast<BatteryType>(device_index);
  float voltage = getVoltage(battery_type);
  float percentage = calculatePercentage(voltage);
  float current = getCurrent(battery_type);

  snprintf(message, sizeof(message), "BATTERY:%d,%.2fV,%.1f%%,%.2fA",
           device_index, voltage, percentage, current);
  
  SerialManager::singleton().SendDiagnosticMessage(message);
}

/**
 * @brief Initializes all battery monitoring hardware and sensors.
 * 
 * Performs complete system initialization including:
 * - GPIO pin configuration for analog reading and power control
 * - I2C bus initialization with high-speed configuration
 * - I2C multiplexer connectivity verification
 * - INA226 sensor initialization and configuration
 * 
 * @note Setup is idempotent - subsequent calls are safely ignored.
 * @warning If multiplexer is not available, sensor initialization is skipped.
 */
void BatteryModule::setup() {
  if (setup_completed_) {
    return;  // Prevent duplicate initialization
  }

  SerialManager::singleton().SendDiagnosticMessage(
      "[BatteryModule::setup] Starting sensor initialization...");

  // Configure hardware pins
  pinMode(MAIN_BATTERY_PIN, INPUT);              // Analog voltage reading
  pinMode(kI2CMultiplexorEnablePin, OUTPUT);     // I2C multiplexer power control
  digitalWrite(kI2CMultiplexorEnablePin, HIGH);  // Enable I2C multiplexer power

  // Initialize I2C with high-speed configuration
  Wire.begin();
  Wire.setClock(400000);  // 400kHz for faster sensor communication

  // Verify I2C multiplexer connectivity
  multiplexer_available_ = testI2CMultiplexer();
  if (!multiplexer_available_) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BatteryModule::setup] I2C multiplexer not available, cannot proceed.");
    return;
  }

  // Initialize all INA226 sensors
  for (size_t device = 0; device < kNumberOfBatteries; device++) {
    selectSensor(static_cast<BatteryType>(device));
    g_ina226_[device].begin();
    // Configure for 20A max current with 2mΩ shunt resistor
    g_ina226_[device].setMaxCurrentShunt(20, 0.002);
    // TODO: Consider enabling averaging and continuous mode for better accuracy
    // g_ina226_[device].setAverage(INA226_AVERAGE_16);
    // g_ina226_[device].setMode(INA226_MODE_CONTINUOUS);
  }

  setup_completed_ = true;
  SerialManager::singleton().SendDiagnosticMessage(
      "[BatteryModule::setup] Initialization completed successfully.");
}

/**
 * @brief Gets the singleton instance of BatteryModule.
 * 
 * Implements thread-safe lazy initialization of the singleton instance.
 * Creates the instance on first access and returns the same instance
 * for all subsequent calls.
 * 
 * @return Reference to the singleton BatteryModule instance.
 * 
 * @note Not thread-safe in multi-threaded environments. Consider adding
 *       mutex protection if used in multi-threaded context.
 */
BatteryModule& BatteryModule::singleton() {
  if (!g_instance_) {
    g_instance_ = new BatteryModule();
  }
  return *g_instance_;
}

/**
 * @brief Tests I2C multiplexer presence and responsiveness.
 * 
 * Attempts to establish communication with the I2C multiplexer to verify
 * it's present and functioning before attempting sensor initialization.
 * Provides diagnostic feedback on success or failure.
 * 
 * @return true if multiplexer responds successfully to I2C communication.
 * 
 * @note This is a simple presence test. Consider adding more comprehensive
 *       functionality tests for production use.
 */
bool BatteryModule::testI2CMultiplexer() {
  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BatteryModule::testI2CMultiplexer] I2C multiplexer found at address 0x" +
        String(I2C_MULTIPLEXER_ADDRESS, HEX));
    return true;
  } else {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BatteryModule::testI2CMultiplexer] I2C multiplexer NOT found at address 0x" +
        String(I2C_MULTIPLEXER_ADDRESS, HEX) + " (error: " + String(error) + ")");
    return false;
  }
}

// Static member variable definitions
BatteryModule* BatteryModule::g_instance_ = nullptr;

float BatteryModule::g_averages_[BatteryModule::kNumberOfBatteries]
                                [BatteryModule::kNumberReadingsToAverage_] = {};

size_t BatteryModule::g_next_average_index_[BatteryModule::kNumberOfBatteries] = {0};

uint8_t BatteryModule::gINA226_DeviceIndexes_[kNumberOfBatteries] = {
    2  // Main battery on multiplexer channel 2
};

INA226 BatteryModule::g_ina226_[kNumberOfBatteries] = {
    INA226(BatteryModule::INA226_ADDRESS)  // INA226 instance for battery monitoring
};