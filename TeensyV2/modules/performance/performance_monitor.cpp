/**
 * @file performance_monitor.cpp
 * @brief Implementation of real-time performance monitoring
 * 
 * @author Wimble Robotics
 * @date 2025
 */

#include "performance_monitor.h"
#include "serial_manager.h"

namespace sigyn_teensy {

PerformanceMonitor::PerformanceMonitor() {
  // Constructor body can be empty if all initialization is done in the
  // initializer list
}

void PerformanceMonitor::checkLoopFrequency() {
  uint32_t now_us = micros();
  if (last_loop_start_time_us_ > 0) {
    uint32_t loop_duration_us = now_us - last_loop_start_time_us_;
    if (loop_duration_us > 0) {
      current_loop_frequency_hz_ = 1000000.0f / loop_duration_us;
    }
  }
  last_loop_start_time_us_ = now_us;

  if (current_loop_frequency_hz_ < config_.min_loop_frequency_hz) {
    if (violations_.consecutive_frequency_violations < 255) {
      violations_.consecutive_frequency_violations++;
    }
    violations_.total_frequency_violations++;
    violations_.last_violation_time_ms = millis();
  } else {
    violations_.consecutive_frequency_violations = 0;
  }
}

void PerformanceMonitor::checkModuleExecutionTimes() {
  bool any_module_violated = false;
  for (uint16_t i = 0; i < Module::getModuleCount(); ++i) {
    Module* mod = Module::getModule(i);
    if (mod) {
      float execution_time_ms = mod->getLastExecutionTimeUs() / 1000.0f;
      if (execution_time_ms > config_.max_module_time_ms) {
        any_module_violated = true;
        // Optional: Log which module violated
        if (config_.enable_detailed_logging) {
          char log_msg[64];
          snprintf(log_msg, sizeof(log_msg), "Module %s exceeded time limit: %.2fms", mod->name(), execution_time_ms);
          SerialManager::getInstance().sendMessage("WARN", log_msg);
        }
      }
    }
  }

  if (any_module_violated) {
    if (violations_.consecutive_module_violations < 255) {
      violations_.consecutive_module_violations++;
    }
    violations_.total_module_violations++;
    violations_.last_violation_time_ms = millis();
  } else {
    violations_.consecutive_module_violations = 0;
  }
}

void PerformanceMonitor::checkPerformance() {
  checkLoopFrequency();
  checkModuleExecutionTimes();

  if (violations_.consecutive_frequency_violations >=
          config_.max_frequency_violations ||
      violations_.consecutive_module_violations >= config_.max_violation_count) {
    violations_.safety_violation_active = true;
  } else {
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
  return violations_.safety_violation_active;
}

void PerformanceMonitor::loop() {
  // The main work is done in CheckPerformance, which is called from the main
  // loop after all modules have run.
  checkPerformance();

  uint32_t now = millis();
  if (now - last_stats_report_ms_ >= config_.stats_report_interval_ms) {
    reportStats();
    last_stats_report_ms_ = now;
  }
}

const char* PerformanceMonitor::name() const { return "PerformanceMonitor"; }

void PerformanceMonitor::getPerformanceStats(char* buffer, size_t size) {
  snprintf(buffer, size, "{\"freq\":%.1f, \"mod_viol\":%lu, \"freq_viol\":%lu}",
           current_loop_frequency_hz_,
           (unsigned long)violations_.total_module_violations,
           (unsigned long)violations_.total_frequency_violations);
}

void PerformanceMonitor::reportStats() {
  char stats_json[256];
  getPerformanceStats(stats_json, sizeof(stats_json));
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
}

void PerformanceMonitor::updateConfig(const PerformanceConfig& new_config) {
  config_ = new_config;
}

}  // namespace sigyn_teensy
