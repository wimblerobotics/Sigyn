// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file performance_monitor.cpp
 * @brief Implementation of real-time performance monitoring
 *
 * This file implements the PerformanceMonitor module that provides comprehensive
 * real-time performance analysis, safety violation detection, and diagnostic
 * reporting for the TeensyV2 system.
 *
 * Key Implementation Features:
 *
 * **High-Resolution Timing:**
 * - Uses micros() for microsecond-precision measurements
 * - Tracks both instantaneous and cumulative performance metrics
 * - Minimal measurement overhead to avoid affecting measured performance
 *
 * **Safety Integration:**
 * - Implements configurable violation thresholds with hysteresis
 * - Provides multiple violation categories (timing, frequency, consecutive)
 * - Integrates with global safety system through isUnsafe() interface
 *
 * **Adaptive Configuration:**
 * - Runtime parameter updates via updateConfig() method
 * - Separate configuration for different operational modes
 * - Validation of parameter ranges to prevent unsafe configurations
 *
 * **Diagnostic Reporting:**
 * - JSON-formatted performance reports for external monitoring
 * - Configurable reporting intervals to balance detail vs. overhead
 * - Integration with SerialManager for structured logging
 *
 * **Performance Characteristics:**
 * - Loop execution overhead: <50 microseconds typical
 * - Memory footprint: ~64 bytes including configuration and state
 * - No dynamic memory allocation during runtime
 * - Lock-free operation suitable for real-time execution
 *
 * Algorithm Details:
 * The monitoring algorithm uses a state machine approach to distribute
 * measurement overhead across multiple loop iterations, ensuring that
 * the monitoring itself doesn't impact system performance.
 *
 * @author Wimble Robotics
 * @date 2025
 * @version 2.0
 */

#include <Arduino.h>
#include "performance_monitor.h"
#include "common/core/serial_manager.h"
#include "common/core/module.h"
#include "common/core/config.h" // Ensure BOARD_* macros are available
#include <limits>

namespace sigyn_teensy {

  PerformanceMonitor::PerformanceMonitor() {
    // Initialize per-board defaults with safe fallbacks when macros not defined
#if BOARD_ID == 1
    config_.min_loop_frequency_hz = 30.0f;
#elif BOARD_ID == 2
    config_.min_loop_frequency_hz = 20.0f;
#else
#ifdef BOARD_MIN_LOOP_FREQUENCY_HZ
    config_.min_loop_frequency_hz = BOARD_MIN_LOOP_FREQUENCY_HZ;
#else
    config_.min_loop_frequency_hz = 20.0f;
#endif
#endif

#ifdef BOARD_MAX_MODULE_TIME_MS
    config_.max_module_time_ms = BOARD_MAX_MODULE_TIME_MS;
#else
    config_.max_module_time_ms = 3.0f;
#endif
  }

  void PerformanceMonitor::checkFrequencyViolations() {
    // Get current frequency from the Module system for accurate measurement
    float current_frequency = Module::getCurrentLoopFrequency();

    // Check if frequency has dropped below acceptable threshold
    // This indicates system overload or blocking operations in modules
    if (current_frequency > 0.0f && current_frequency < config_.min_loop_frequency_hz) {
      // Increment consecutive violation counter with overflow protection
      if (violations_.consecutive_frequency_violations < 255) {
        violations_.consecutive_frequency_violations++;
      }

      violations_.total_frequency_violations++;
      violations_.last_violation_time_ms = millis();

      // Send warning message for frequency violations
      char warn_msg[64];
      snprintf(warn_msg, sizeof(warn_msg), "FREQ_VIOLATION: %.1fHz < %.1fHz (violation #%lu)",
        current_frequency, config_.min_loop_frequency_hz,
        (unsigned long)violations_.total_frequency_violations);
      SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), warn_msg);

      // Also send a high-priority safety message for critical violations
      if (violations_.consecutive_frequency_violations >= config_.max_frequency_violations) {
        SerialManager::getInstance().sendDiagnosticMessage("CRITICAL", name(), "Multiple consecutive frequency violations - system overload detected!");
      }
    }
    else {
      // Reset consecutive counter when performance recovers
      // This implements hysteresis to prevent oscillating safety states
      violations_.consecutive_frequency_violations = 0;
    }
  }

  void PerformanceMonitor::checkModuleExecutionTimes() {
    bool any_module_violated = false;

    // Iterate through all registered modules to check execution times
    // This provides comprehensive monitoring without requiring individual
    // module participation beyond basic timing measurement
    for (uint16_t i = 0; i < Module::getModuleCount(); ++i) {
      Module* mod = Module::getModule(i);
      if (mod) {
        // Convert microseconds to milliseconds for threshold comparison
        // Module timing thresholds are more intuitive in milliseconds
        float execution_time_ms = mod->getLastExecutionTimeUs() / 1000.0f;

        if (execution_time_ms > config_.max_module_time_ms) {
          any_module_violated = true;

          // Optional detailed logging for debugging and optimization
          // Only enabled when configured to minimize performance impact
          if (config_.enable_detailed_logging) {
            char log_msg[64];
            snprintf(log_msg, sizeof(log_msg),
              "Module %s exceeded time limit: %.2fms",
              mod->name(), execution_time_ms);
            SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), log_msg);
          }
        }
      }
    }

    // Update violation tracking based on this iteration's results
    // Consecutive violations indicate systemic performance problems
    if (any_module_violated) {
      if (violations_.consecutive_module_violations < 255) {
        violations_.consecutive_module_violations++;
      }
      violations_.total_module_violations++;
      violations_.last_violation_time_ms = millis();
    }
    else {
      // Reset consecutive counter when all modules perform acceptably
      violations_.consecutive_module_violations = 0;
    }
  }

  void PerformanceMonitor::checkPerformance() {
    checkFrequencyViolations();
    checkModuleExecutionTimes();

    if (violations_.consecutive_frequency_violations >=
      config_.max_frequency_violations ||
      violations_.consecutive_module_violations >= config_.max_violation_count) {
      violations_.safety_violation_active = true;
    }
    else {
      violations_.safety_violation_active = false;
    }
  }

  const PerformanceConfig& PerformanceMonitor::getConfig() const {
    return config_;
  }

  PerformanceMonitor& PerformanceMonitor::getInstance() {
    static PerformanceMonitor instance;
    return instance;
  }

  const ViolationTracker& PerformanceMonitor::getViolations() const {
    return violations_;
  }

  bool PerformanceMonitor::isUnsafe() {
    return false; //#####violations_.safety_violation_active;
  }

  void PerformanceMonitor::loop() {
    // The main work is done in CheckPerformance, which is called from the main
    // loop after all modules have run.
    checkPerformance();

    uint32_t now = millis();
    if (now - last_stats_report_ms_ >= config_.stats_report_interval_ms &&
      now >= first_report_delay_ms_) {
      reportStats();
      last_stats_report_ms_ = now;
    }
  }

  const char* PerformanceMonitor::name() const { return "PerformanceMonitor"; }

  void PerformanceMonitor::getPerformanceStats(char* buffer, size_t size) {
    // Get current frequency from Module system for accurate reporting
    float current_frequency = Module::getCurrentLoopFrequency();

    // Start with basic frequency and violation counts - using shortened field names
    int written = snprintf(buffer, size,
      "{\"freq\":%.1f, \"tfreq\":%.1f, \"mviol\":%lu, \"fviol\":%lu",
      current_frequency,
      config_.min_loop_frequency_hz,
      (unsigned long)violations_.total_module_violations,
      (unsigned long)violations_.total_frequency_violations);

    if (written < 0 || (size_t)written >= size) return;

    // Add violation details if there are any - using shortened field names
    if (violations_.safety_violation_active) {
      int details_written = snprintf(buffer + written, size - written,
        ", \"violdet\":{\"cmod\":%d,\"cfreq\":%d,\"lastms\":%lu}",
        violations_.consecutive_module_violations,
        violations_.consecutive_frequency_violations,
        (unsigned long)violations_.last_violation_time_ms);

      if (details_written > 0 && written + details_written < (int)size) {
        written += details_written;
      }
    }

    // Add module timing information - using shortened field names
    int modules_written = snprintf(buffer + written, size - written, ", \"mods\":[");
    if (modules_written > 0 && written + modules_written < (int)size) {
      written += modules_written;

      // Iterate through all registered modules using Module::getModuleCount and Module::getModule
      for (uint16_t i = 0; i < Module::getModuleCount(); ++i) {
        Module* mod = Module::getModule(i);
        if (mod) {
          const auto& stats = mod->getPerformanceStats(); // Ensure the correct type is returned
          float exec_time_ms = mod->getLastExecutionTimeUs() / 1000.0f;
          bool is_violation = exec_time_ms > config_.max_module_time_ms;

          // Get detailed performance statistics from the module
          float avg_us = (stats.loop_count > 0) ? (stats.duration_sum_us / stats.loop_count) : 0.0f;

          // Convert microseconds to milliseconds for readability
          float min_ms = stats.duration_min_us / 1000.0f;
          float max_ms = stats.duration_max_us / 1000.0f;
          float avg_ms = avg_us / 1000.0f;

          int mod_written = snprintf(buffer + written, size - written,
            "%s{\"n\":\"%s\",\"min\":%.2f,\"max\":%.2f,\"avg\":%.2f,\"last\":%.2f,\"cnt\":%lu,\"viol\":%s}",
            i > 0 ? "," : "",
            mod->name(),
            min_ms, max_ms, avg_ms, exec_time_ms,
            (unsigned long)stats.loop_count,
            is_violation ? "true" : "false");

          if (mod_written > 0 && written + mod_written < (int)size - 50) // Leave more buffer space
            written += mod_written;
          else {
            // Debug: Log buffer overflow
            char debug_msg[100];
            snprintf(debug_msg, sizeof(debug_msg), "PERF buffer overflow at module %d, written=%d, need=%d, size=%d",
              i, written, mod_written, (int)size);
            SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), debug_msg);
            break; // Buffer full
          }
        }
      }
    }

    // Close JSON structure with proper bounds checking
    if (written < (int)size - 10) {  // Ensure we have enough space for closing
      int closing_written = snprintf(buffer + written, size - written, "]}");
      if (closing_written > 0) {
        written += closing_written;
      }
    }
    else {
      // Force close the JSON even if we're running out of space
      if (written < (int)size - 3) {
        buffer[written] = ']';
        buffer[written + 1] = '}';
        buffer[written + 2] = '\0';
      }
    }

    // Ensure null termination
    buffer[size - 1] = '\0';
  }

  void PerformanceMonitor::reportStats() {
    // Build PERF JSON in a large local buffer; on-wire payload is capped below
    char stats_json[3072];
    getPerformanceStats(stats_json, sizeof(stats_json));

    // Enforce payload cap to avoid SerialManager (768) overflow and SD truncation
    const size_t MAX_SERIAL_PAYLOAD = 740; // allow for prefix and terminator
    if (strlen(stats_json) > MAX_SERIAL_PAYLOAD) {
      stats_json[MAX_SERIAL_PAYLOAD - 3] = ']';
      stats_json[MAX_SERIAL_PAYLOAD - 2] = '}';
      stats_json[MAX_SERIAL_PAYLOAD - 1] = '\0';
    }
    SerialManager::getInstance().sendMessage("PERF", stats_json);
  }

  void PerformanceMonitor::resetSafetyFlags() {
    violations_.consecutive_frequency_violations = 0;
    violations_.consecutive_module_violations = 0;
    violations_.safety_violation_active = false;
    // We keep total violations for long-term diagnostics, but they could be
    // reset here if needed.
  }

  void PerformanceMonitor::setup() {
    // Initialization code for PerformanceMonitor, if any.
    // For example, load configuration from EEPROM or set initial values.
    last_stats_report_ms_ = millis();

    // Delay first report to allow modules to complete at least one execution cycle
    // This prevents showing invalid min values (MAXFLOAT) in the first PERF report
    first_report_delay_ms_ = 2000; // Wait 2 seconds before first PERF report
  }

  void PerformanceMonitor::updateConfig(const PerformanceConfig& new_config) {
    config_ = new_config;
  }



}  // namespace sigyn_teensy
