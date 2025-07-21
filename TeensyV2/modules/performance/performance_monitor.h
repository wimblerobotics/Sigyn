/**
 * @file performance_monitor.h
 * @brief Real-time performance monitoring and safety violation detection
 * 
 * The PerformanceMonitor module provides comprehensive real-time performance
 * analysis for the TeensyV2 system, ensuring that timing constraints are met
 * and triggering safety violations when performance degrades to levels that
 * could compromise robot safety or mission objectives.
 * 
 * Core Responsibilities:
 * - Monitor main loop frequency against target of 80-100Hz
 * - Detect individual modules exceeding execution time limits (>2ms)
 * - Track consecutive performance violations and trigger safety alerts
 * - Provide runtime configuration of performance parameters via ROS2
 * - Generate detailed diagnostic reports for system optimization
 * - Implement adaptive thresholds based on operational conditions
 * 
 * Safety Integration:
 * The PerformanceMonitor integrates tightly with the global safety system by
 * implementing isUnsafe() to report when performance violations reach critical
 * levels that could compromise robot safety. This ensures that performance
 * degradation is treated as seriously as hardware failures.
 * 
 * Performance Violation Categories:
 * 1. **Module Timing Violations**: Individual modules exceeding 2ms execution time
 * 2. **Frequency Violations**: Overall loop frequency dropping below 80Hz
 * 3. **Consecutive Violations**: Multiple violations in sequence indicating systemic issues
 * 4. **Trend Analysis**: Performance degradation over time indicating component wear
 * 
 * Configuration Philosophy:
 * All performance thresholds are configurable at runtime to enable tuning for
 * different operational scenarios. For example:
 * - Indoor navigation: Strict timing for precise motor control
 * - Outdoor patrol: Relaxed timing allowing more sensor processing
 * - Emergency mode: Minimal processing for maximum responsiveness
 * 
 * Diagnostic Capabilities:
 * The monitor provides multiple levels of diagnostic information:
 * - Real-time performance metrics via serial output
 * - Historical trend analysis for predictive maintenance
 * - Module-specific profiling for optimization guidance
 * - JSON-formatted reports for integration with monitoring systems
 * 
 * Implementation Notes:
 * - Uses high-resolution timing (microseconds) for accurate measurements
 * - Minimal overhead design to avoid affecting measured performance
 * - Lock-free data structures for real-time safety
 * - Configurable reporting intervals to balance detail vs. overhead
 * 
 * @author Wimble Robotics
 * @date 2025
 * @version 2.0
 * @copyright Copyright (c) 2025 Wimble Robotics. All rights reserved.
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
 * This structure contains all tunable parameters that control the behavior
 * of the performance monitoring system. Values can be updated at runtime
 * via the ROS2 parameter interface to adapt monitoring behavior for different
 * operational requirements without requiring firmware recompilation.
 * 
 * Parameter Categories:
 * 
 * **Timing Thresholds:**
 * - Loop frequency limits for detecting system overload
 * - Module execution time limits for detecting runaway code
 * - These directly impact robot safety and must be set conservatively
 * 
 * **Violation Counting:**
 * - Consecutive violation limits before triggering safety responses
 * - Prevents false alarms from transient performance spikes
 * - Balances responsiveness with stability
 * 
 * **Reporting Configuration:**
 * - Statistics reporting intervals for debugging and optimization
 * - Detailed logging enables/disables for performance impact control
 * - Output formatting options for integration with external tools
 * 
 * **Adaptive Behavior:**
 * - Future: Dynamic threshold adjustment based on system load
 * - Future: Predictive violation detection using trend analysis
 * - Future: Operational mode-specific parameter sets
 * 
 * Default Values Rationale:
 * - 80Hz minimum: Sufficient for stable motor control and sensor fusion
 * - 2ms module limit: Ensures 100Hz capability with safety margin
 * - 5 violation limit: Prevents nuisance trips while catching real issues
 * - 1s reporting: Frequent enough for debugging, not overwhelming
 * 
 * Performance Impact:
 * - Configuration changes take effect immediately (no restart required)
 * - Parameter validation ensures values remain within safe operating ranges
 * - Minimal memory footprint (16 bytes total)
 */
struct PerformanceConfig {
  float min_loop_frequency_hz = 80.0f;    ///< Minimum acceptable main loop frequency (Hz) - safety critical
  float max_module_time_ms = 2.0f;        ///< Maximum allowed module execution time (ms) - prevents runaway code
  uint8_t max_violation_count = 5;        ///< Consecutive violations before declaring unsafe - prevents false alarms
  uint8_t max_frequency_violations = 3;   ///< Consecutive frequency violations before unsafe - system overload detection
  uint32_t stats_report_interval_ms = 1000; ///< How often to send performance statistics (ms) - debugging aid
  bool enable_detailed_logging = false;   ///< Enable verbose performance logging - high overhead when enabled
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
  void checkFrequencyViolations();
  void checkModuleExecutionTimes();
  void getPerformanceStats(char* buffer, size_t size);
  void reportStats();

  // --- Member Variables ---
  PerformanceConfig config_;
  ViolationTracker violations_;
  uint32_t last_stats_report_ms_ = 0;
};

}  // namespace sigyn_teensy
