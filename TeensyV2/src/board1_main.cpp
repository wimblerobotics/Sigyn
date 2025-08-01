/**
 * @file board1_main.ino
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
#include "common/core/module.h"
#include "common/core/serial_manager.h"

// Board 1 specific modules
#include "modules/performance/performance_monitor.h"
#include "modules/safety/safety_coordinator.h"
#include "modules/roboclaw/roboclaw_monitor.h"
#include "modules/sensors/vl53l0x_monitor.h"
#include "modules/sensors/temperature_monitor.h"
#include "modules/storage/sd_logger.h"

using namespace sigyn_teensy;

// Function declarations
void fault_handler();
uint32_t freeMemory();
void loop();
void serialEvent();
void setup();


// System timing
uint32_t loop_start_time;
uint32_t last_loop_time;
float loop_frequency;

// Module instances (created via singleton pattern)
SerialManager* serial_manager;
PerformanceMonitor* performance_monitor;
SafetyCoordinator* safety_coordinator;
RoboClawMonitor* roboclaw_monitor;
//###VL53L0XMonitor* vl53l0x_monitor;
TemperatureMonitor* temperature_monitor;
//### SDLogger* sd_logger;

/**
 * @brief Handle critical errors and system faults.
 * 
 * This function is called by the Teensy runtime when critical errors occur.
 */
void fault_handler() {
  // Immediate safety response
  digitalWrite(3, HIGH);  // Assert E-stop output
  digitalWrite(5, HIGH);  // Signal other boards
  
  Serial.println("CRITICAL FAULT: System fault detected, emergency stop activated");
  
  // Try to send fault notification if possible
  if (serial_manager) {
    String fault_msg = "type=system,board=1,time=" + String(millis());
    serial_manager->sendMessage("FAULT", fault_msg.c_str());
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
  
  // Safety monitoring - check for critical performance violations
  static uint32_t last_safety_check = 0;
  if (current_time - last_safety_check > 100000) {  // Every 100ms (10Hz)
    last_safety_check = current_time;
    
    // If we can't keep up with basic timing, trigger safety system
    if (execution_time > 20000) {  // 20ms is unacceptable
      safety_coordinator->triggerEstop(EstopSource::PERFORMANCE, 
                                       "Critical timing violation: " + String(execution_time) + "us");
    }
    
    //### // Check VL53L0X sensors for emergency obstacles
    // if (vl53l0x_monitor) {  // Remove isUnsafe call since it's protected
    //   // Could add other safety checks here for obstacle detection
    // }
  }

  //### // Performance warnings
  // if (execution_time > 10000) {  // 10ms is critically slow
  //   Serial.println("WARNING: Loop execution time exceeded 10ms (" + String(execution_time) + " us)");
  // }
  
  // if (loop_frequency < 50.0f) {  // Below 50Hz is critically slow
  //   Serial.println("WARNING: Loop frequency below 50Hz (" + String(loop_frequency, 1) + " Hz)");
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
  // Initialize serial communication first
  Serial.begin(921600);
  while (!Serial && millis() < 3000) {
    // Wait up to 3 seconds for serial connection
  }
  
  //###Serial.println("===== TeensyV2 Board 1 (Main Controller) Starting =====");
  
  // Get singleton instances (this registers them with the module system)
  serial_manager = &SerialManager::getInstance();
  performance_monitor = &PerformanceMonitor::getInstance();
  safety_coordinator = &SafetyCoordinator::getInstance();
  roboclaw_monitor = &RoboClawMonitor::getInstance();
  //###vl53l0x_monitor = &VL53L0XMonitor::getInstance();
  temperature_monitor = &TemperatureMonitor::getInstance();
  //###sd_logger = &SDLogger::getInstance();
  
  // Initialize serial communication (no return value to check)
  serial_manager->initialize(5000);
  
  // Configure safety system for Board 1
  SafetyConfig safety_config;
  safety_config.hardware_estop_pin = 2;        // Hardware E-stop button
  safety_config.estop_output_pin = 3;          // E-stop relay output
  safety_config.inter_board_input_pin = 4;     // Safety signal from Board 2
  safety_config.inter_board_output_pin = 5;    // Safety signal to Board 2
  safety_config.enable_inter_board_safety = true;
  safety_config.enable_auto_recovery = true;
  // safety_coordinator->Configure(safety_config); // TODO: Add updateConfig method
  
  Module::setupAll();
  
  // Initialize timing
  loop_start_time = micros();
  last_loop_time = loop_start_time;
  loop_frequency = 0.0f;
  
  // Serial.println("===== Board 1 Initialization Complete =====");
  // Serial.println("Target loop frequency: 85Hz");
  // Serial.println("Ready for operation");
}
