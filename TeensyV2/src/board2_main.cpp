#include <Arduino.h>
#include <cstdint>

/**
 * @file board2_main.ino
 // Core TeensyV2 system
#include "common/core/module.h"
#include "common/core/serial_manager.h"

// Board 2 specific modules
#include "modules/performance/performance_monitor.h"
#include "modules/battery/battery_monitor.h"
// #include "modules/sensors/imu_manager.h"         // To be implemented
// #include "modules/sensors/temperature_monitor.h" // To be implementedSensor
board (Board 2) for TeensyV2 system
 *
 * This is the sensor monitoring board responsible for:
 * - Battery monitoring (INA226 and analog sensors)
 * - IMU sensor data collection
 * - Temperature monitoring
 * - Additional environmental sensors
 * - Safety monitoring and reporting to Board 1
 *
 * Hardware Configuration:
 * - Teensy 4.1 microcontroller
 * - INA226 current/voltage sensors for battery monitoring
 * - IMU sensors (gyroscope, accelerometer, magnetometer)
 * - Temperature sensors
 * - Inter-board safety communication with Board 1
 *
 * Real-Time Requirements:
 * - Target loop frequency: 50Hz (adequate for sensor data)
 * - Battery monitoring: 10Hz minimum for safety
 * - IMU data collection: 100Hz for navigation
 * - Safety reporting: 20Hz to Board 1
 *
 * @author Wimble Robotics
 * @date 2025
 */

#include <Arduino.h>

#include "common/core/module.h"
#include "common/core/serial_manager.h"
#include "modules/battery/battery_monitor.h"
#include "modules/performance/performance_monitor.h"
#include "modules/bno055/bno055_monitor.h"
// #include "../../modules/sensors/temperature_monitor.h" // To be implemented

using namespace sigyn_teensy;

// Function declarations
void fault_handler();
uint32_t freeMemory();
void loop();
void processSensorData();
void serialEvent();
void setup();

// System timing
uint32_t loop_start_time;
uint32_t last_loop_time;
float loop_frequency;

// Module instances (created via singleton pattern)
SerialManager* serial_manager;
PerformanceMonitor* performance_monitor;
BatteryMonitor* battery_monitor;
BNO055Monitor* bno055_monitor;

/**
 * @brief Handle critical errors and system faults.
 *
 * This function is called by the Teensy runtime when critical errors occur.
 */
void fault_handler() {
  //### Serial.println("CRITICAL FAULT: Board2 system fault detected");

  // Send fault notification if possible
  if (serial_manager) {
    String fault_msg = "type=system,board=2,time=" + String(millis());
    serial_manager->sendMessage("FAULT", fault_msg.c_str());
  }

  // For Board 2, we don't have direct E-stop control, but we should signal
  // the problem This would be done via inter-board communication in a
  // complete implementation

  // Halt system with status indication
  while (true) {
    digitalWrite(LED_BUILTIN,
                 !digitalRead(LED_BUILTIN));  // Blink LED rapidly
    delay(50);
  }
}

/**
 * @brief Get free memory for monitoring.
 *
 * Simple free memory calculation for Teensy 4.1.
 *
 * @return Estimated free memory in bytes
 */
uint32_t freeMemory() {
  // For Teensy 4.1, we'll use a simple stack-based approach
  // This is not as accurate but works without linker issues
  char stack_var;
  extern char _ebss;
  return &stack_var - &_ebss;
}

void loop() {
  uint32_t current_time = micros();
  loop_start_time = current_time;

  // Calculate loop frequency
  uint32_t loop_time_us = current_time - last_loop_time;
  if (loop_time_us > 0) {
    loop_frequency = 1000000.0f / loop_time_us;
  }
  last_loop_time = current_time;

  // Execute all modules through the module system
  Module::loopAll();

  // Record performance metrics
  uint32_t execution_time = micros() - loop_start_time;

  // Board 2 status reporting (less frequent than Board 1)
  static uint32_t last_status_report_ms = 0;
  if (current_time - last_status_report_ms > 10000000) {  // Every 10 seconds
    last_status_report_ms = current_time;
    // Add any 10-second reporting tasks here
  }

  // Safety monitoring - check for critical battery conditions
  static uint32_t last_safety_check = 0;
  if (current_time - last_safety_check > 200000) {  // Every 200ms (5Hz)
    last_safety_check = current_time;

    // Check battery safety conditions
    float voltage = battery_monitor->getVoltage();
    float current = battery_monitor->getCurrent();

    // Send safety status to Board 1 (if inter-board communication is
    // implemented)
    bool battery_critical = (voltage < 32.0f) || (abs(current) > 15.0f);

    // For now, just report via serial - inter-board GPIO communication to be
    // added
    static bool last_critical_state = false;
    //### if (battery_critical != last_critical_state) {
    //   last_critical_state = battery_critical;

    //   if (battery_critical) {
    //     Serial.println("SAFETY: Battery critical condition detected!");
    //     Serial.println("  Voltage: " + String(voltage, 2) + " V");
    //     Serial.println("  Current: " + String(current, 2) + " A");

    //     // Send emergency message to ROS2 system
    //     if (serial_manager) {
    //       String safety_msg = "board=2,critical=true,reason=battery";
    //       if (!isnan(voltage)) safety_msg += ",voltage=" + String(voltage, 2);
    //       if (!isnan(current)) safety_msg += ",current=" + String(current, 2);
    //       serial_manager->sendMessage("SAFETY_CRITICAL", safety_msg.c_str());
    //     }
    //   } else {
    //     Serial.println("SAFETY: Battery condition returned to normal");

    //     if (serial_manager) {
    //       serial_manager->sendMessage(
    //           "SAFETY_CRITICAL",
    //           "board=2,critical=false,reason=battery_recovered");
    //     }
    //   }
    // }
  }

  //### // Performance warnings (less strict than Board 1)
  // if (execution_time > 15000) {  // 15ms is concerning for Board 2
  //   Serial.println("WARNING: Board2 execution time exceeded 15ms (" +
  //                  String(execution_time) + " us)");
  // }

  // if (loop_frequency < 20.0f) {  // Below 20Hz is concerning for Board 2
  //   Serial.println("WARNING: Board2 frequency below 20Hz (" +
  //                  String(loop_frequency, 1) + " Hz)");
  // }

  // Longer delay acceptable for Board 2 since it's primarily sensor
  // monitoring Target 50Hz = 20ms period
  delayMicroseconds(500);  // 0.5ms delay for stability
}

/**
 * @brief Handle sensor data collection and processing.
 *
 * This function demonstrates how sensor data would be collected and processed
 * in a complete implementation. Called from main loop at appropriate
 * intervals.
 */
void processSensorData() {
  // Example of how additional sensor modules would be integrated:

  // IMU data collection (100Hz)
  // static uint32_t last_imu_update = 0;
  // if (micros() - last_imu_update >= 10000) {  // 10ms = 100Hz
  //   last_imu_update = micros();
  //   imu_manager->ReadSensors();
  //   imu_manager->SendData();
  // }

  // Temperature monitoring (1Hz)
  // static uint32_t last_temp_update = 0;
  // if (millis() - last_temp_update >= 1000) {  // 1000ms = 1Hz
  //   last_temp_update = millis();
  //   temperature_monitor->ReadSensors();
  //   temperature_monitor->CheckThresholds();
  // }
}

/**
 * @brief Handle serial events for configuration updates.
 *
 * This function is called automatically when serial data is available.
 */
void serialEvent() {
  if (serial_manager) {
    serial_manager->processIncomingMessages();
  }
}

void setup() {
  // Initialize serial communication for debugging and ROS2 interface
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    // Wait for serial port to connect. Needed for native USB only.
  }

  // Initialize modules
  serial_manager = &SerialManager::getInstance();
  performance_monitor = &PerformanceMonitor::getInstance();
  battery_monitor = &BatteryMonitor::getInstance();
  bno055_monitor = &BNO055Monitor::getInstance();

  // Initialize all registered modules
  Module::setupAll();

  // Initialize system timing
  last_loop_time = micros();

  serial_manager->sendMessage("INFO", "Board 2 setup complete");
}
