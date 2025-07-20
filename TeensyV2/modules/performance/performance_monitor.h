/**
 * @file performance_monitor.h
 * @brief Real-time performance monitoring and safety violation detection
 * 
 * Monitors system performance to ensure real-time constraints are met.
 * Triggers safety violations when timing requirements are violated and
 * provides comprehensive diagnostic information for system optimization.
 * 
 * Key Responsibilities:
 * - Monitor main loop frequency (target 80-100Hz)
 * - Detect modules exceeding execution time limits (>2ms)
 * - Track consecutive performance violations
 * - Trigger safety alerts when thresholds exceeded
 * - Provide runtime configuration of performance parameters
 * - Generate detailed diagnostic reports
 * 
 * Safety Integration:
 * The PerformanceMonitor integrates with the global safety system by
 * implementing IsUnsafe() to report when performance violations reach
 * critical levels that could compromise robot safety.
 * 
 * @author Wimble Robotics
 * @date 2025
 */

#pragma once

#include <Arduino.h>
#include <cstdint>
#include <cstddef>
#include <cmath>
#include "module.h"

namespace sigyn_teensy {

/**
 * @brief Configuration parameters for performance monitoring.
 * 
 * These values can be updated at runtime via ROS2 parameter interface
 * to tune performance monitoring for specific operational requirements.
 */
struct PerformanceConfig {
  float min_loop_frequency_hz = 80.0f;    ///< Minimum acceptable loop frequency
  float max_module_time_ms = 2.0f;        ///< Maximum allowed module execution time
  uint8_t max_violation_count = 5;        ///< Consecutive violations before unsafe
  uint8_t max_frequency_violations = 3;   ///< Consecutive frequency violations before unsafe
  uint32_t stats_report_interval_ms = 1000; ///< How often to send performance stats
  bool enable_detailed_logging = false;   ///< Enable verbose performance logging
};

/**
 * @brief Performance violation tracking.
 */
struct ViolationTracker {
  uint8_t consecutive_module_violations = 0;    ///< Count of consecutive module time violations
  uint8_t consecutive_frequency_violations = 0; ///< Count of consecutive frequency violations
  uint32_t total_module_violations = 0;         ///< Total module violations since reset
  uint32_t total_frequency_violations = 0;      ///< Total frequency violations since reset
  uint32_t last_violation_time_ms = 0;          ///< Timestamp of last violation
  bool safety_violation_active = false;         ///< True if performance violations are unsafe
};

/**
 * @brief Real-time performance monitor for TeensyV2 system.
 * 
 * Continuously monitors system performance to ensure real-time constraints
 * are met. Provides early warning of performance degradation and triggers
 * safety violations when timing becomes critical to robot operation.
 * 
 * The monitor tracks:
 * - Overall loop frequency (target 80-100Hz)
 * - Individual module execution times
 * - Consecutive violation counts
 * - Performance trends over time
 * 
 * Safety thresholds are configurable to allow tuning for different
 * operational scenarios while maintaining safety-critical timing.
 * 
 * Example usage:
 * @code
 *   PerformanceMonitor& monitor = PerformanceMonitor::GetInstance();
 *   monitor.beginModule("MyModule");
 *   // ... module code ...
 *   monitor.endModule("MyModule");
 * @endcode
 */
class PerformanceMonitor : public Module {
 public:
  // --- Singleton Access ---
  /**
   * @brief Get singleton instance of PerformanceMonitor.
   * @return Reference to singleton PerformanceMonitor
   */
  static PerformanceMonitor& getInstance();

  // --- Public API ---
  /**
   * @brief Check for performance violations and update safety status.
   * 
   * This method is called automatically by the main loop and should not be
   * called directly by other modules.
   */
  void checkPerformance();

  /**
   * @brief Get current performance configuration.
   * @return Const reference to the current configuration
   */
  const PerformanceConfig& getConfig() const;

  /**
   * @brief Get current violation status.
   * @return Const reference to the violation tracker
   */
  const ViolationTracker& getViolations() const;

  /**
   * @brief Update performance monitoring configuration.
   * @param new_config New configuration to apply
   */
  void updateConfig(const PerformanceConfig& new_config);

  // --- Module Overrides ---
  /**
   * @brief Initialize the performance monitor.
   * 
   * This method is called once at system startup to initialize the monitor
   * and configure initial settings.
   */
  void setup() override;

  /**
   * @brief Main loop for performance monitoring.
   * 
   * This method is called automatically by the main loop. It checks for
   * performance violations and reports statistics.
   */
  void loop() override;

  /**
   * @brief Return the name of this module.
   */
  const char* name() const override;

  /**
   * @brief Check if the performance monitor has detected an unsafe condition.
   * @return True if performance violations exceed critical thresholds
   */
  bool isUnsafe() override;

  /**
   * @brief Reset safety violation flags.
   * 
   * Called when a safety condition is acknowledged and cleared.
   */
  void resetSafetyFlags() override;

 private:
  // --- Singleton ---
  PerformanceMonitor();
  PerformanceMonitor(const PerformanceMonitor&) = delete;
  PerformanceMonitor& operator=(const PerformanceMonitor&) = delete;

  // --- Private Methods ---
  void checkLoopFrequency();
  void checkModuleExecutionTimes();
  void getPerformanceStats(char* buffer, size_t size);
  void reportStats();

  // --- Member Variables ---
  PerformanceConfig config_;
  ViolationTracker violations_;
  float current_loop_frequency_hz_ = 0.0f;
  uint32_t last_loop_start_time_us_ = 0;
  uint32_t last_stats_report_ms_ = 0;
};

}  // namespace sigyn_teensy
