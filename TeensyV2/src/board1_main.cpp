// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file board1_main.cpp
 * @brief Main controller board (Board 1) for TeensyV2 system
 *
 * This is the primary controller board responsible for:
 * - Motor control and odometry
 * - VL53L0X distance sensors
 * - RoboClaw motor driver interface
 * - E-stop coordination and safety management
 * - Primary communication with ROS2 system
 *
 * Hardware Configuration:
 * - Teensy 4.1 microcontroller
 * - RoboClaw motor controllers
 * - VL53L0X time-of-flight sensors
 * - Hardware E-stop button
 * - Inter-board safety communication
 *
 * Real-Time Requirements:
 * - Target loop frequency: 85Hz (80-100Hz acceptable)
 * - Maximum module execution time: 2ms per iteration
 * - Motor control update rate: 50Hz minimum
 * - Safety monitoring: 100Hz
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

// Function declarations
void fault_handler();
uint32_t freeMemory();
void loop();
void serialEvent();
void setup();

// TODO: Uncomment when inter-board communication is implemented
// void interBoardSignalReceived();

// Module instances (created via singleton pattern, conditionally based on board config)
SerialManager* serial_manager;

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

/**
 * @brief Handle critical errors and system faults.
 *
 * This function is called by the Teensy runtime when critical errors occur.
 */
void fault_handler() {
  // Immediate safety response - signal other boards about the fault
  // TODO: Uncomment when inter-board communication is implemented
  // digitalWrite(INTER_BOARD_SIGNAL_OUTPUT_PIN, HIGH);  // Signal other boards
  
  String fault_msg = "CRITICAL FAULT: Board " + String(BOARD_ID) + " system fault detected, signaling Board 1";
  serial_manager->sendDiagnosticMessage("FAULT", "board1_main", fault_msg.c_str());

  // Try to send fault notification if possible
  if (serial_manager) {
    String fault_msg = "type=system,board=" + String(BOARD_ID) + ",time=" + String(millis());
    serial_manager->sendDiagnosticMessage("FAULT", "board1_main", fault_msg.c_str());
  }

  // Halt system
  while (true) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // Blink LED
    delay(100);
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
  // Execute all modules through the module system
  Module::loopAll();

  // Board-specific safety monitoring
  static uint32_t last_safety_check = 0;
  uint32_t current_time = micros();
  if (current_time - last_safety_check >= SAFETY_CHECK_INTERVAL_US) {
    last_safety_check = current_time;

    // Board-specific safety checks
#if ENABLE_VL53L0X
    // Check VL53L0X sensors for emergency obstacles
    // VL53L0X monitoring is handled by the VL53L0XMonitor module
    // Additional board-specific obstacle detection logic can be added here
#endif

    // TODO: Add other board-specific safety checks here
    // - Inter-board communication health check
    // - Hardware E-stop button status (Board 1 only)
    // - Critical system status verification
  }
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
  // Initialize serial communication first
  uint32_t start_time = millis();
  Serial.begin(BOARD_SERIAL_BAUD_RATE);
  while (!Serial && (start_time - millis() < BOARD_SERIAL_WAIT_MS)) {
    // Wait for serial connection
  }

#if ENABLE_ROBOCLAW
  RoboClawConfig config_;
  start_time = millis();
  Serial7.begin(config_.baud_rate); // RoboClaw serial port
  while (!Serial7 && (start_time - millis() < BOARD_SERIAL_WAIT_MS)) {
    // Wait for serial connection
  }
#endif

  // TODO: Setup inter-board communication pins (commented out for now)
  // pinMode(INTER_BOARD_SIGNAL_OUTPUT_PIN, OUTPUT);
  // pinMode(INTER_BOARD_SIGNAL_INPUT_PIN, INPUT_PULLUP);
  // digitalWrite(INTER_BOARD_SIGNAL_OUTPUT_PIN, LOW);  // Default to no signal
  // 
  // Setup interrupt for inter-board signal reception
  // attachInterrupt(digitalPinToInterrupt(INTER_BOARD_SIGNAL_INPUT_PIN), 
  //                 interBoardSignalReceived, RISING);

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

#if ENABLE_ROBOCLAW
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

 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board1", ("===== Board " + String(BOARD_ID) + " Initialization Complete =====").c_str());

  // Print enabled features for this board
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board1", "Enabled features:");
#if ENABLE_SD_LOGGING
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board1", "  - SD Logging");
#endif
#if ENABLE_MOTOR_CONTROL
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board1", "  - Motor Control");
#endif
#if ENABLE_VL53L0X
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board1", "  - VL53L0X Distance Sensors");
#endif
#if ENABLE_TEMPERATURE
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board1", "  - Temperature Monitoring");
#endif
#if ENABLE_PERFORMANCE
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board1", "  - Performance Monitoring");
#endif
#if ENABLE_SAFETY
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board1", "  - Safety Coordinator");
#endif
#if ENABLE_ROBOCLAW
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board1", "  - RoboClaw Motor Driver");
#endif
#if ENABLE_BATTERY
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board1", "  - Battery Monitoring");
#endif
#if ENABLE_IMU
 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board1", "  - IMU (BNO055)");
#endif

 SerialManager::getInstance().sendDiagnosticMessage("INFO", "board1", "Ready for operation");
}

// TODO: Uncomment when inter-board communication is implemented
// /**
//  * @brief Interrupt handler for inter-board signal reception.
//  * 
//  * This function is called when another board signals an E-stop condition.
//  * Different boards will have different responses to inter-board signals.
//  */
// void interBoardSignalReceived() {
//   // Board-specific response to inter-board E-stop signal
// #if BOARD_ID == 1
//   // Board 1: Activate hardware E-stop
//   digitalWrite(ESTOP_OUTPUT_PIN, HIGH);
// #elif BOARD_ID == 2
//   // Board 2: Enter safe mode, stop all sensors/monitoring
//   // Implementation will be board-specific
// #elif BOARD_ID == 3
//   // Board 3: Future expansion board response
//   // Implementation will be board-specific
// #endif
//   
//   // Common response: Log the event
//   // Note: Cannot use Serial or complex functions in interrupt handler
//   // Set a flag for main loop to handle the logging
// }
