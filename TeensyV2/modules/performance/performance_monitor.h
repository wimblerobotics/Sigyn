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
 * @author Sigyn Robotics
 * @date 2025
 */

#pragma once

#include "common/core/module.h"

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
 * // In main program initialization:
 * PerformanceMonitor& perf_monitor = PerformanceMonitor::GetInstance();
 * 
 * // The monitor automatically tracks performance via the Module system
 * // and reports violations through IsUnsafe() and diagnostic messages
 * @endcode
 */
class PerformanceMonitor : public Module {
 public:
  /**
   * @brief Get the singleton instance of PerformanceMonitor.
   * 
   * @return Reference to the singleton PerformanceMonitor instance
   */
  static PerformanceMonitor& GetInstance();

  /**
   * @brief Update performance monitoring configuration.
   * 
   * Allows runtime adjustment of performance thresholds and monitoring
   * parameters via ROS2 parameter interface or other configuration sources.
   * 
   * @param[in] config New configuration parameters
   * @return true if configuration valid and applied, false otherwise
   */
  bool UpdateConfiguration(const PerformanceConfig& config);

  /**
   * @brief Get current performance configuration.
   * 
   * @return Reference to current configuration parameters
   */
  const PerformanceConfig& GetConfiguration() const { return config_; }

  /**
   * @brief Get current violation tracking status.
   * 
   * @return Reference to current violation tracking data
   */
  const ViolationTracker& GetViolationStatus() const { return violations_; }

  /**
   * @brief Reset violation counters and safety flags.
   * 
   * Clears accumulated violation counts and resets safety violation state.
   * Used during system recovery procedures.
   */
  void ResetViolationCounters();

  /**
   * @brief Check if performance violations have reached unsafe levels.
   * 
   * Returns true when consecutive performance violations exceed configured
   * thresholds, indicating that real-time constraints are compromised
   * to a degree that could affect robot safety.
   * 
   * @return true if performance violations are safety-critical
   */
  bool IsUnsafe() override;

  /**
   * @brief Reset safety-related performance flags.
   * 
   * Clears safety violation flags when system performance returns to
   * acceptable levels. Called by safety recovery procedures.
   */
  void ResetSafetyFlags() override;

  /**
   * @brief Force generation of detailed performance report.
   * 
   * Immediately generates and sends a comprehensive performance report
   * regardless of normal reporting schedule. Useful for debugging and
   * system optimization.
   */
  void GenerateDetailedReport();

 protected:
  /**
   * @brief One-time initialization of performance monitoring.
   * 
   * Sets up monitoring parameters and initializes tracking variables.
   * Called once during system startup.
   */
  void setup() override;

  /**
   * @brief Main performance monitoring loop.
   * 
   * Analyzes current system performance, detects violations, and
   * generates reports. This method itself must be very fast (≤1ms)
   * to avoid affecting the performance it monitors.
   */
  void loop() override;

  /**
   * @brief Get module name for identification.
   * 
   * @return Module name string "PerformanceMonitor"
   */
  const char* name() override { return "PerformanceMonitor"; }

 private:
  /**
   * @brief Private constructor for singleton pattern.
   */
  PerformanceMonitor();

  /**
   * @brief Analyze current loop frequency and detect violations.
   */
  void CheckLoopFrequency();

  /**
   * @brief Analyze individual module performance and detect violations.
   */
  void CheckModulePerformance();

  /**
   * @brief Process a detected performance violation.
   * 
   * @param[in] violation_type Type of violation detected
   * @param[in] details Additional details about the violation
   */
  void ProcessViolation(const char* violation_type, const char* details);

  /**
   * @brief Send performance statistics report.
   */
  void SendPerformanceReport();

  /**
   * @brief Validate configuration parameters.
   * 
   * @param[in] config Configuration to validate
   * @return true if configuration is valid, false otherwise
   */
  bool ValidateConfiguration(const PerformanceConfig& config) const;

  // Singleton instance
  static PerformanceMonitor* instance_;

  // Configuration and state
  PerformanceConfig config_;              ///< Current configuration parameters
  ViolationTracker violations_;           ///< Violation tracking state

  // Performance tracking
  uint32_t last_loop_time_us_;           ///< Timestamp of last loop execution
  uint32_t last_report_time_ms_;         ///< Timestamp of last statistics report
  float current_loop_frequency_;         ///< Current measured loop frequency
  uint32_t loop_count_since_report_;     ///< Loops executed since last report

  // Module performance history (for trend analysis)
  struct ModulePerformanceHistory {
    float recent_max_time_ms;            ///< Maximum time in recent history
    uint8_t consecutive_violations;       ///< Consecutive violations for this module
  };
  
  static constexpr size_t kMaxTrackedModules = 16;
  ModulePerformanceHistory module_history_[kMaxTrackedModules];
  size_t tracked_module_count_;

  // Prevent copying
  PerformanceMonitor(const PerformanceMonitor&) = delete;
  PerformanceMonitor& operator=(const PerformanceMonitor&) = delete;
};

}  // namespace sigyn_teensy
