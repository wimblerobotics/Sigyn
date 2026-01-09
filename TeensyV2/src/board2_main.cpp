// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file board2_main.cpp
 * @brief Sensor monitoring board (Board 2) for TeensyV2 system
 *
 * This is the sensor monitoring board responsible for:
 * - Battery monitoring (INA226 and analog sensors)
 * - IMU sensor data collection
 * - Additional environmental sensors
 * - Safety monitoring and reporting to Board 1
 *
 * Hardware Configuration:
 * - Teensy 4.1 microcontroller
 * - INA226 current/voltage sensors for battery monitoring
 * - IMU sensors (gyroscope, accelerometer, magnetometer)
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
#include <cstdint>

// Core TeensyV2 system
#include "common/core/config.h"
#include "common/core/module.h"
#include "common/core/serial_manager.h"

// Conditional module includes based on board configuration
#if ENABLE_PERFORMANCE
#include "modules/performance/performance_monitor.h"
#endif

#if ENABLE_SAFETY
#include "modules/safety/safety_coordinator.h"
#endif

#if BOARD_HAS_MOTOR_CONTROL
#include "modules/roboclaw/roboclaw_monitor.h"
#endif

#if ENABLE_VL53L0X
#include "modules/sensors/vl53l0x_monitor.h"
#endif

#if ENABLE_TEMPERATURE
#include "modules/sensors/temperature_monitor.h"
#endif

#if ENABLE_BATTERY
#include "modules/battery/battery_monitor.h"
#endif

#if ENABLE_IMU
#include "modules/bno055/bno055_monitor.h"
#endif

#if ENABLE_SD_LOGGING
#include "modules/storage/sd_logger.h"
#endif

using namespace sigyn_teensy;

// Function declarations
void loop();
void serialEvent();
void setup();

// Module instances (created via singleton pattern, conditionally based on board config)
SerialManager* serial_manager;

#if ENABLE_PERFORMANCE
PerformanceMonitor* performance_monitor;
#endif

#if ENABLE_SAFETY
SafetyCoordinator* safety_coordinator;
#endif

#if BOARD_HAS_MOTOR_CONTROL
RoboClawMonitor* roboclaw_monitor;
#endif

#if ENABLE_TEMPERATURE
TemperatureMonitor* temperature_monitor;
#endif

#if ENABLE_BATTERY
BatteryMonitor* battery_monitor;
#endif

#if ENABLE_IMU
BNO055Monitor* bno055_monitor;
#endif

#if ENABLE_SD_LOGGING
SDLogger* sd_logger;
#endif

void loop() {
  // Execute all modules through the module system
  Module::loopAll();

  // Drain outgoing serial queue without blocking the real-time loop.
  SerialManager::getInstance().processOutgoingMessages();
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
  // Initialize serial communication (do not block waiting for USB).
  Serial.begin(BOARD_SERIAL_BAUD_RATE);

  // Initialize SD logger first if enabled (required for other modules)
#if ENABLE_SD_LOGGING
  sd_logger = &SDLogger::getInstance();
#endif

  // Get singleton instances (this registers them with the module system)
  // Core modules are always initialized
  serial_manager = &SerialManager::getInstance();

  // Initialize modules based on board configuration
#if ENABLE_PERFORMANCE
  performance_monitor = &PerformanceMonitor::getInstance();
#endif

#if ENABLE_SAFETY
  safety_coordinator = &SafetyCoordinator::getInstance();
#endif

#if BOARD_HAS_MOTOR_CONTROL
  roboclaw_monitor = &RoboClawMonitor::getInstance();
#endif

#if ENABLE_VL53L0X
  VL53L0XMonitor::getInstance();  // Initialize VL53L0X monitor
#endif

#if ENABLE_TEMPERATURE
  temperature_monitor = &TemperatureMonitor::getInstance();
#endif

#if ENABLE_BATTERY
  battery_monitor = &BatteryMonitor::getInstance();
#endif

#if ENABLE_IMU
  bno055_monitor = &BNO055Monitor::getInstance();
#endif

  // Initialize serial communication
  serial_manager->initialize(BOARD_SERIAL_TIMEOUT_MS);

  // Initialize all registered modules
  Module::setupAll();

  {
    char msg[96] = {0};
    snprintf(msg, sizeof(msg), "===== Board %d Initialization Complete =====", BOARD_ID);
    SerialManager::getInstance().sendDiagnosticMessage("INFO", "board2", msg);
  }

  // Print enabled features for this board
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board2", "Enabled features:");
#if ENABLE_SD_LOGGING
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board2", "  - SD Logging");
#endif
#if BOARD_HAS_MOTOR_CONTROL
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board2", "  - Board has Motor Control");
#endif
#if ENABLE_VL53L0X
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board2", "  - VL53L0X Distance Sensors");
#endif
#if ENABLE_TEMPERATURE
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board2", "  - Temperature Monitoring");
#endif
#if ENABLE_PERFORMANCE
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board2", "  - Performance Monitoring");
#endif
#if ENABLE_SAFETY
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board2", "  - Safety Coordinator");
#endif
#if BOARD_HAS_MOTOR_CONTROL
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board2", "  - RoboClaw Motor Driver");
#endif
#if ENABLE_BATTERY
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board2", "  - Battery Monitoring");
#endif
#if ENABLE_IMU
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board2", "  - IMU (BNO055)");
#endif

 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board2", "Ready for operation");
}
