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
   * @brief Mark the start of a module's execution for timing.
   * @param module_name Name of the module
   */
  void beginModule(const char* module_name);

  /**
   * @brief Mark the end of a module's execution and record timing.
   * @param module_name Name of the module
   */
  void endModule(const char* module_name);

  /**
   * @brief Return the name of this module.
   */
  const char* name() const override;

 protected:
  // --- Module Overrides ---
  void setup() override;
  void loop() override;

 private:
  // --- Private Constructor ---
  PerformanceMonitor();
  ~PerformanceMonitor() = default;
  PerformanceMonitor(const PerformanceMonitor&) = delete;
  PerformanceMonitor& operator=(const PerformanceMonitor&) = delete;

  // --- Member Functions ---
  void checkLoopFrequency();
  void sendStatistics();

  // --- Data Members ---
  PerformanceConfig config_;
  ViolationTracker violations_;

  uint32_t loop_start_time_us_ = 0;
  uint32_t last_stats_send_time_ms_ = 0;
  uint32_t module_start_time_us_ = 0;
  const char* current_module_name_ = nullptr;
};

}  // namespace sigyn_teensy
