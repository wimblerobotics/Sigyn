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

// Core TeensyV2 system
#include "common/core/module.h"
#include "common/core/serial_manager.h"

// Board 1 specific modules
#include "modules/performance/performance_monitor.h"
#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <cmath>
#include "modules/safety/safety_coordinator.h"
// #include "modules/motor/motor_controller.h"      // To be implemented
// #include "modules/sensors/vl53l0x_manager.h"    // To be implemented
// #include "modules/navigation/odometry.h"        // To be implemented

using namespace sigyn_teensy;

// Function declarations
uint32_t freeMemory();

// System timing
uint32_t loop_start_time;
uint32_t last_loop_time;
float loop_frequency;

// Module instances (created via singleton pattern)
SerialManager* serial_manager;
PerformanceMonitor* performance_monitor;
SafetyCoordinator* safety_coordinator;

void setup() {
  // Initialize serial communication first
  Serial.begin(921600);
  while (!Serial && millis() < 3000) {
    // Wait up to 3 seconds for serial connection
  }
  
  Serial.println("===== TeensyV2 Board 1 (Main Controller) Starting =====");
  
  // Get singleton instances (this registers them with the module system)
  serial_manager = &SerialManager::GetInstance();
  performance_monitor = &PerformanceMonitor::GetInstance();
  safety_coordinator = &SafetyCoordinator::GetInstance();
  
  // Initialize serial communication
  if (!serial_manager->Initialize(5000)) {
    Serial.println("ERROR: Failed to initialize serial communication");
    // Continue anyway - might be running without PC connection
  }
  
  // Configure safety system for Board 1
  SafetyConfig safety_config;
  safety_config.hardware_estop_pin = 2;        // Hardware E-stop button
  safety_config.estop_output_pin = 3;          // E-stop relay output
  safety_config.inter_board_input_pin = 4;     // Safety signal from Board 2
  safety_config.inter_board_output_pin = 5;    // Safety signal to Board 2
  safety_config.enable_inter_board_safety = true;
  safety_config.enable_auto_recovery = true;
  safety_coordinator->Configure(safety_config);
  
  // Initialize all modules through the module system
  Serial.println("Initializing modules...");
  Module::SetupAll();
  
  // Initialize timing
  loop_start_time = micros();
  last_loop_time = loop_start_time;
  loop_frequency = 0.0f;
  
  Serial.println("===== Board 1 Initialization Complete =====");
  Serial.println("Target loop frequency: 85Hz");
  Serial.println("Ready for operation");
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
  Module::LoopAll();
  
  // Record performance metrics
  uint32_t execution_time = micros() - loop_start_time;
  
  // Basic performance monitoring (detailed monitoring handled by PerformanceMonitor module)
  static uint32_t last_perf_report = 0;
  if (current_time - last_perf_report > 5000000) {  // Every 5 seconds
    last_perf_report = current_time;
    Serial.println("Board1 Status:");
    Serial.println("  Loop frequency: " + String(loop_frequency, 1) + " Hz");
    Serial.println("  Execution time: " + String(execution_time) + " us");
    Serial.println("  Safety state: " + safety_coordinator->GetSafetyStatusDescription());
    Serial.println("  Free memory: " + String(freeMemory()) + " bytes");
  }
  
  // Check for critical performance violations
  if (execution_time > 10000) {  // 10ms is critically slow
    Serial.println("WARNING: Loop execution time exceeded 10ms (" + String(execution_time) + " us)");
  }
  
  if (loop_frequency < 50.0f) {  // Below 50Hz is critically slow
    Serial.println("WARNING: Loop frequency below 50Hz (" + String(loop_frequency, 1) + " Hz)");
  }
  
  // Safety check - emergency stop if system becomes completely unresponsive
  static uint32_t last_safety_check = 0;
  if (current_time - last_safety_check > 100000) {  // Every 100ms
    last_safety_check = current_time;
    
    // If we can't keep up with basic timing, trigger safety system
    if (execution_time > 20000) {  // 20ms is unacceptable
      safety_coordinator->TriggerEstop(EstopSource::PERFORMANCE, 
                                       "Critical timing violation: " + String(execution_time) + "us",
                                       execution_time);
    }
  }
  
  // Small delay to prevent overwhelming the system
  // Target 85Hz = ~11.7ms period, so we can afford a small delay
  delayMicroseconds(100);  // 0.1ms delay for stability
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
    serial_manager->SendMessage("FAULT", fault_msg.c_str());
  }
  
  // Halt system
  while (true) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // Blink LED
    delay(100);
  }
}

/**
 * @brief Handle serial events for configuration updates.
 * 
 * This function is called automatically when serial data is available.
 */
void serialEvent() {
  if (serial_manager) {
    serial_manager->ProcessIncomingMessages();
  }
}
