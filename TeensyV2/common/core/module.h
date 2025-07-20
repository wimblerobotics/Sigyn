/**
 * @file module.h
 * @brief Core module system for TeensyV2 real-time embedded framework
 * 
 * This file provides the foundational architecture for the TeensyV2 embedded system,
 * enabling modular, high-performance sensor and actuator management with automatic
 * registration, real-time performance monitoring, and safety coordination.
 * 
 * The Module system is designed to maintain strict real-time constraints while
 * providing a clean, object-oriented interface for adding new functionality.
 * All modules are automatically registered upon instantiation and participate
 * in a unified execution and monitoring framework.
 * 
 * Design Goals:
 * - Maintain 80-100Hz control loop frequency across all modules
 * - Automatic performance monitoring and violation detection
 * - Clean separation of concerns between different subsystems
 * - Easy addition of new functionality without architectural changes
 * - Zero-overhead abstractions where possible
 * 
 * Usage Pattern:
 * 1. Create subclass of Module (e.g., BatteryMonitor, MotorController)
 * 2. Implement singleton pattern with automatic registration
 * 3. Override virtual methods: setup(), loop(), name()
 * 4. Optional: Override isUnsafe() for safety monitoring
 * 
 * Performance Requirements:
 * - setup(): Can block during initialization (called once at startup)
 * - loop(): Must complete in ≤2ms (called at 80-100Hz in main loop)
 * - Use non-blocking operations and state machines for complex tasks
 * - Avoid dynamic memory allocation in loop() methods
 * 
 * Thread Safety:
 * - Module registration is NOT thread-safe (single-threaded initialization)
 * - Module execution is single-threaded (Arduino main loop model)
 * - No inter-module synchronization primitives needed
 * 
 * Memory Management:
 * - Modules are statically allocated via singleton pattern
 * - No dynamic allocation after initialization phase
 * - Performance statistics use fixed-size structures
 * 
 * Safety Integration:
 * - Each module can report unsafe conditions via isUnsafe()
 * - Safety violations automatically trigger emergency procedures
 * - Modules can implement recovery logic in resetSafetyFlags()
 * 
 * @author Wimble Robotics
 * @date 2025
 * @version 2.0
 * @copyright Copyright (c) 2025 Wimble Robotics. All rights reserved.
 */

#pragma once

#include "Arduino.h"
#include <cstdint>
#include <cstddef>
#include <cmath>
namespace sigyn_teensy {

/**
 * @brief Performance statistics for timing analysis and safety monitoring.
 * 
 * This structure tracks execution timing for each module to enable real-time
 * performance analysis and safety violation detection. Statistics are updated
 * automatically by the Module framework on each loop iteration.
 * 
 * All timing values are in microseconds for maximum precision while avoiding
 * floating-point arithmetic in the critical path. The statistics provide both
 * instantaneous and aggregated performance data for optimization and debugging.
 * 
 * Memory Layout:
 * - Total size: 16 bytes (cache-friendly)
 * - All members are POD types for optimal performance
 * - Aligned for efficient access on ARM Cortex-M7
 * 
 * Usage Notes:
 * - Values are reset only on system restart or explicit request
 * - Statistics accumulate over the entire runtime of the system
 * - Used by PerformanceMonitor for safety violation detection
 */
struct PerformanceStats {
  float duration_min_us = MAXFLOAT;  ///< Minimum execution time (microseconds) - tracks best-case performance
  float duration_max_us = 0.0f;      ///< Maximum execution time (microseconds) - tracks worst-case performance  
  float duration_sum_us = 0.0f;      ///< Cumulative execution time for averaging - used for mean calculation
  uint32_t loop_count = 0;           ///< Number of loops executed - enables average calculation and rate monitoring
};

/**
 * @brief Base class for all modular functionality in the TeensyV2 system.
 * 
 * The Module class provides the foundational framework for automatic module registration,
 * real-time performance monitoring, and safety coordination. All sensor modules, actuator
 * controllers, and system services inherit from this class to participate in the unified
 * execution and monitoring framework.
 * 
 * Architecture Overview:
 * The Module system implements an inversion-of-control pattern where individual modules
 * register themselves automatically upon instantiation and are managed collectively by
 * static methods. This approach eliminates the need for manual module management while
 * ensuring all modules participate in performance monitoring and safety checks.
 * 
 * Real-Time Performance:
 * The Module system maintains real-time performance by:
 * - Tracking execution time of each module's loop() method with microsecond precision
 * - Detecting performance violations when modules exceed 2ms execution time
 * - Monitoring overall system frequency with target of 80-100Hz
 * - Aggregating safety conditions across all modules for emergency response
 * - Providing detailed performance statistics for optimization and debugging
 * 
 * Execution Model:
 * 1. Initialization Phase (setup()):
 *    - All modules perform blocking initialization operations
 *    - Hardware interfaces are configured and tested
 *    - Initial sensor readings and calibration performed
 *    - Error conditions reported immediately
 * 
 * 2. Runtime Phase (loop()):
 *    - Non-blocking, real-time execution at 80-100Hz
 *    - State machines used for complex operations
 *    - Sensor data collection and filtering
 *    - Control output generation
 *    - Safety condition monitoring
 * 
 * Safety Integration:
 * Each module participates in the global safety system by:
 * - Implementing isUnsafe() to report critical conditions
 * - Supporting resetSafetyFlags() for recovery operations
 * - Following fail-safe design principles
 * - Providing detailed error reporting for diagnostics
 * 
 * Performance Characteristics:
 * - Module registration: O(1) time complexity, O(n) space where n = module count
 * - Loop execution: O(n) time complexity for n registered modules
 * - Memory overhead: ~24 bytes per module for performance tracking
 * - Maximum modules: 32 (configurable via kMaxModules)
 * 
 * Example Implementation:
 * @code
 * class BatteryMonitor : public Module {
 * public:
 *   static BatteryMonitor& getInstance() {
 *     static BatteryMonitor instance;  // Automatic registration
 *     return instance;
 *   }
 * 
 * protected:
 *   void setup() override { 
 *     // Initialize INA226 sensor, configure alerts
 *     if (!ina226_.begin()) {
 *       Serial.println("ERROR: Battery monitor initialization failed");
 *     }
 *   }
 *   
 *   void loop() override { 
 *     // Non-blocking sensor reading with state machine
 *     switch (state_) {
 *       case READING_VOLTAGE:
 *         voltage_ = ina226_.readVoltage();
 *         state_ = READING_CURRENT;
 *         break;
 *       case READING_CURRENT:
 *         current_ = ina226_.readCurrent();
 *         checkSafetyLimits();
 *         state_ = READING_VOLTAGE;
 *         break;
 *     }
 *   }
 *   
 *   const char* name() const override { return "BatteryMonitor"; }
 *   
 *   bool isUnsafe() override { 
 *     return voltage_ < CRITICAL_VOLTAGE_THRESHOLD || 
 *            current_ > CRITICAL_CURRENT_THRESHOLD;
 *   }
 * 
 * private:
 *   BatteryMonitor() : Module() {}  // Private constructor for singleton
 *   INA226 ina226_;
 *   float voltage_, current_;
 *   enum State { READING_VOLTAGE, READING_CURRENT } state_;
 * };
 * 
 * // In main.cpp:
 * void setup() {
 *   // Get singleton instance (automatically registers)
 *   BatteryMonitor::getInstance();
 *   
 *   // Initialize all registered modules
 *   Module::setupAll();
 * }
 * 
 * void loop() {
 *   // Execute all modules with performance monitoring
 *   Module::loopAll();
 * }
 * @endcode
 */
class Module {
 public:
  /**
   * @brief Maximum number of modules that can be registered.
   */
  static constexpr uint16_t kMaxModules = 32;

  /**
   * @brief Maximum execution time for any module's loop() method (microseconds).
   */
  static constexpr float kMaxLoopTimeUs = 2000.0f;  // 2ms

  /**
   * @brief Target minimum frequency for the main control loop (Hz).
   */
  static constexpr float kTargetLoopFrequencyHz = 80.0f;

  /**
   * @brief Virtual destructor for proper cleanup of derived classes.
   */
  virtual ~Module() = default;

  /**
   * @brief Initialize all registered modules.
   * 
   * Calls setup() on each registered module in registration order.
   * This is called once during system startup and may take significant
   * time as modules perform blocking initialization operations.
   * 
   * @note This method is blocking and should only be called during startup.
   */
  static void setupAll();

  /**
   * @brief Execute one iteration of all registered modules.
   * 
   * Calls loop() on each registered module and tracks performance statistics.
   * Detects performance violations and safety conditions.
   * 
   * This method must be called repeatedly at high frequency (80-100Hz target)
   * to maintain real-time system performance.
   * 
   * Performance monitoring:
   * - Tracks execution time of each module
   * - Detects modules exceeding time limits
   * - Monitors overall loop frequency
   * - Reports statistics periodically
   */
  static void loopAll();

  /**
   * @brief Check if any module reports an unsafe condition.
   * 
   * Aggregates safety conditions from all registered modules.
   * Used by safety systems to determine if emergency actions are needed.
   * 
   * @return true if any module reports unsafe condition, false otherwise
   */
  static bool isAnyModuleUnsafe();

  /**
   * @brief Reset safety flags for all modules.
   * 
   * Calls ResetSafetyFlags() on all modules to clear recoverable error states.
   * Used during emergency recovery procedures.
   */
  static void resetAllSafetyFlags();

  /**
   * @brief Get performance statistics for all modules.
   * 
   * @param[out] stats_json Buffer to write JSON-formatted statistics
   * @param[in] buffer_size Size of output buffer
   * 
   * Format: {"freq":95.2,"modules":[{"name":"Battery","min":0.1,"max":1.8,"avg":0.5},...]}
   */
  static void getPerformanceStats(char* stats_json, size_t buffer_size);

  /**
   * @brief Get number of registered modules.
   * 
   * @return Number of modules currently registered
   */
  static uint16_t getModuleCount() { return module_count_; }

  /**
   * @brief Get a pointer to a module by its index.
   * 
   * @param index The index of the module to retrieve.
   * @return A pointer to the module, or nullptr if the index is invalid.
   */
  static Module* getModule(uint16_t index) {
    if (index < module_count_) {
      return modules_[index];
    }
    return nullptr;
  }

  /**
   * @brief Check if this specific module reports an unsafe condition.
   * 
   * Override this method to implement module-specific safety monitoring.
   * Called by the safety system to determine if emergency actions are needed.
   * 
   * @return true if module detects unsafe condition, false otherwise
   * 
   * Examples of unsafe conditions:
   * - Battery voltage below critical threshold
   * - Motor overcurrent condition
   * - Sensor communication failure
   * - Temperature exceeding safe limits
   */
  virtual bool isUnsafe() { return false; }

  /**
   * @brief Reset module-specific safety flags.
   * 
   * Override this method to clear recoverable error states within the module.
   * Called during emergency recovery procedures when conditions improve.
   * 
   * Examples:
   * - Clear overcurrent flags when current returns to normal
   * - Reset communication timeout flags when communication resumes
   * - Clear temperature warnings when temperature drops
   */
  virtual void resetSafetyFlags() {}

  /**
   * @brief Return the name of this module.
   * 
   * @return Pointer to compile-time constant string
   * 
   * Should return a descriptive name like "BatteryMonitor", "MotorController",
   * "VL53L0XArray", etc. Used for identification in diagnostics and logging.
   */
  virtual const char* name() const = 0;

  /**
   * @brief Get the execution time of the last loop for this module.
   * 
   * @return The execution time in microseconds.
   */
  float getLastExecutionTimeUs() const { return last_execution_time_us_; }

  /**
   * @brief Get performance statistics for this module.
   * @return Const reference to the performance statistics
   */
  // const PerformanceStats& getStats() const { return stats_; } // Deprecated

 protected:
  /**
   * @brief Default constructor. Automatically registers the module.
   * @note This is protected to enforce singleton pattern for derived classes.
   */
  Module();

  // --- Virtual Methods for Derived Classes ---
  /**
   * @brief One-time setup for the module.
   */
  virtual void setup() = 0;

  /**
   * @brief Regular, cyclic work for the module.
   */
  virtual void loop() = 0;

  /**
   * @brief Update performance statistics for a module.
   * 
   * @param module Pointer to the module instance
   * @param execution_time_us Execution time of the module's loop in microseconds
   */
  static void updatePerformanceStats(Module* module,
                                     uint32_t execution_time_us);

  // --- Member Variables ---
  bool is_setup_ = false;               ///< True if the module's setup() has been called
  float last_execution_time_us_ = 0.0f; ///< Execution time of the last loop in microseconds

 private:
  // --- Static Helper Methods ---
  /**
   * @brief Check performance and safety for all modules.
   */
  static void checkPerformanceAndSafety();

  /**
   * @brief Report performance statistics for all modules.
   */
  static void reportPerformanceStats();

  // --- Static Member Variables ---
  /**
   * @brief Array of registered module instances.
   */
  static Module* modules_[kMaxModules];    ///< Array of registered modules
  static uint16_t module_count_;           ///< Number of registered modules
  static uint32_t total_loop_count_;       ///< Total loops executed
  static uint32_t last_stats_report_ms_;   ///< Timestamp of last statistics report
  static uint32_t loop_start_time_us_;     ///< Start time of current loop iteration
};

}  // namespace sigyn_teensy
