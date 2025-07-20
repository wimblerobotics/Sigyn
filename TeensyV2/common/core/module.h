/**
 * @file module.h
 * @brief Core module system for TeensyV2 real-time embedded framework
 * 
 * Provides a modular architecture for sensor and actuator management with
 * automatic registration, performance monitoring, and safety coordination.
 * 
 * Design Goals:
 * - Maintain 80-100Hz control loop frequency
 * - Automatic performance monitoring and violation detection
 * - Clean separation of concerns between modules
 * - Easy addition of new functionality
 * 
 * Usage Pattern:
 * 1. Create subclass of Module (e.g., BatteryMonitor, MotorController)
 * 2. Implement singleton pattern with automatic registration
 * 3. Override virtual methods: setup(), loop(), name()
 * 4. Optional: Override IsUnsafe() for safety monitoring
 * 
 * Performance Requirements:
 * - setup(): Can block during initialization (called once)
 * - loop(): Must complete in ≤2ms (called at 80-100Hz)
 * - Use non-blocking operations and state machines for complex tasks
 * 
 * @author Wimble Robotics
 * @date 2025
 */

#pragma once

#include "Arduino.h"
#include <cstdint>
#include <cstddef>
#include <cmath>
namespace sigyn_teensy {

/**
 * @brief Performance statistics for timing analysis and safety monitoring.
 */
struct PerformanceStats {
  float duration_min_us = MAXFLOAT;  ///< Minimum execution time (microseconds)
  float duration_max_us = 0.0f;      ///< Maximum execution time (microseconds)
  float duration_sum_us = 0.0f;      ///< Cumulative execution time for averaging
  uint32_t loop_count = 0;           ///< Number of loops executed
};

/**
 * @brief Base class for all modular functionality in the TeensyV2 system.
 * 
 * Provides framework for automatic module registration, performance monitoring,
 * and safety coordination. All sensor modules, actuator controllers, and
 * system services inherit from this class.
 * 
 * The Module system maintains real-time performance by:
 * - Tracking execution time of each module's loop() method
 * - Detecting performance violations (>2ms execution time)
 * - Monitoring overall system frequency (target 80-100Hz)
 * - Aggregating safety conditions across all modules
 * 
 * Example usage:
 * @code
 * class BatteryMonitor : public Module {
 * public:
 *   static BatteryMonitor& GetInstance() {
 *     static BatteryMonitor instance;
 *     return instance;
 *   }
 * 
 * protected:
 *   void setup() override { // Initialize sensors }
 *   void loop() override { // Read battery, check safety }
 *   const char* name() override { return "BatteryMonitor"; }
 *   bool IsUnsafe() override { return battery_voltage_critical_; }
 * 
 * private:
 *   BatteryMonitor() : Module() {}  // Automatic registration
 * };
 * 
 * // In main():
 * BatteryMonitor::GetInstance();  // Registers automatically
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
