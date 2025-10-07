// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file elevator_board.cpp
 * @brief Elevator/Extender control (elevator_board) for TeensyV2 system
 *
 * @author Wimble Robotics
 * @date 2025
 */

#include <Arduino.h>

#include "../common/core/config.h"
#include "../common/core/module.h"
#include "../common/core/serial_manager.h"
#include "../modules/motors/stepper_motor.h"

 // Conditional module includes based on board configuration
#if ENABLE_PERFORMANCE
#include "modules/performance/performance_monitor.h"
#endif

#if ENABLE_SAFETY
#include "modules/safety/safety_coordinator.h"
#endif

#if ENABLE_ROBOCLAW
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

void serialEvent();

// Instantiate required singletons for this board
SerialManager* serial_manager;
static StepperMotor& g_stepper = StepperMotor::getInstance();

#if ENABLE_PERFORMANCE
PerformanceMonitor* performance_monitor;
#endif

#if ENABLE_SAFETY
SafetyCoordinator* safety_coordinator;
#endif

#if ENABLE_ROBOCLAW
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

void setup() {
  // Initialize serial communication first
  uint32_t start_time = millis();
  Serial.begin(BOARD_SERIAL_BAUD_RATE);
  while (!Serial && (start_time - millis() < BOARD_SERIAL_WAIT_MS)) {
    // Wait for serial connection
  }

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


  // Initialize serial communication
  serial_manager->initialize(BOARD_SERIAL_TIMEOUT_MS);

  // Setup all modules (SDLogger and StepperMotor are Modules)
  Module::setupAll();

  char init_msg[128];
  snprintf(init_msg, sizeof(init_msg),
    "===== Board %d Initialization Complete =====", BOARD_ID);
  SerialManager::getInstance().sendDiagnosticMessage("INFO", "elevator_board",
    init_msg);

  // Print enabled features for this board
  SerialManager::getInstance().sendDiagnosticMessage("INFO", "elevator_board",
    "Enabled features:");
#if ENABLE_SD_LOGGING
  SerialManager::getInstance().sendDiagnosticMessage("INFO", "elevator_board",
    "  - SD Logging");
#endif
#if ENABLE_MOTOR_CONTROL
  SerialManager::getInstance().sendDiagnosticMessage("INFO", "elevator_board",
    "  - Motor Control");
#endif
#if ENABLE_VL53L0X
  SerialManager::getInstance().sendDiagnosticMessage(
    "INFO", "elevator_board", "  - VL53L0X Distance Sensors");
#endif
#if ENABLE_TEMPERATURE
  SerialManager::getInstance().sendDiagnosticMessage(
    "INFO", "elevator_board", "  - Temperature Monitoring");
#endif
#if ENABLE_PERFORMANCE
  SerialManager::getInstance().sendDiagnosticMessage(
    "INFO", "elevator_board", "  - Performance Monitoring");
#endif
#if ENABLE_SAFETY
  SerialManager::getInstance().sendDiagnosticMessage("INFO", "elevator_board",
    "  - Safety Coordinator");
#endif
#if ENABLE_ROBOCLAW
  SerialManager::getInstance().sendDiagnosticMessage(
    "INFO", "elevator_board", "  - RoboClaw Motor Driver");
#endif
#if ENABLE_BATTERY
  SerialManager::getInstance().sendDiagnosticMessage("INFO", "elevator_board",
    "  - Battery Monitoring");
#endif
#if ENABLE_IMU
  SerialManager::getInstance().sendDiagnosticMessage("INFO", "elevator_board",
    "  - IMU (BNO055)");
#endif

  SerialManager::getInstance().sendDiagnosticMessage("INFO", "elevator_board",
    "Ready for operation");
}

void loop() {
  // Run all modules once
  Module::loopAll();
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