/**
 * @file performance_monitor.cpp
 * @brief Implementation of real-time performance monitoring
 * 
 * @author Wimble Robotics
 * @date 2025
 */

#include "performance_monitor.h"
#include <serial_manager.h>

namespace sigyn_teensy {

// Static member definition
PerformanceMonitor* PerformanceMonitor::instance_ = nullptr;

PerformanceMonitor::PerformanceMonitor()
    : Module(),
      last_loop_time_us_(0),
      last_report_time_ms_(0),
      current_loop_frequency_(0.0f),
      loop_count_since_report_(0),
      tracked_module_count_(0) {
  // Initialize with default configuration
  config_ = PerformanceConfig{};
  violations_ = ViolationTracker{};
  
  // Clear module history
  for (size_t i = 0; i < kMaxTrackedModules; ++i) {
    module_history_[i] = ModulePerformanceHistory{};
  }
}

PerformanceMonitor& PerformanceMonitor::GetInstance() {
  if (instance_ == nullptr) {
    instance_ = new PerformanceMonitor();
  }
  return *instance_;
}

bool PerformanceMonitor::UpdateConfiguration(const PerformanceConfig& config) {
  if (!ValidateConfiguration(config)) {
    return false;
  }

  config_ = config;
  
  // Log configuration update
  char config_msg[128];
  snprintf(config_msg, sizeof(config_msg),
          "min_freq=%.1f,max_time=%.1f,max_violations=%d",
          config_.min_loop_frequency_hz,
          config_.max_module_time_ms,
          config_.max_violation_count);
  SerialManager::GetInstance().SendMessage("PERF_CONFIG", config_msg);
  
  return true;
}

void PerformanceMonitor::ResetViolationCounters() {
  violations_.consecutive_module_violations = 0;
  violations_.consecutive_frequency_violations = 0;
  violations_.total_module_violations = 0;
  violations_.total_frequency_violations = 0;
  violations_.last_violation_time_ms = 0;
  violations_.safety_violation_active = false;
  
  // Reset module-specific violation counters
  for (size_t i = 0; i < tracked_module_count_; ++i) {
    module_history_[i].consecutive_violations = 0;
  }
  
  SerialManager::GetInstance().SendMessage("PERF_RESET", "violations_cleared=true");
}

bool PerformanceMonitor::IsUnsafe() {
  // Check if consecutive violations exceed safety thresholds
  bool unsafe = (violations_.consecutive_module_violations >= config_.max_violation_count) ||
                (violations_.consecutive_frequency_violations >= config_.max_frequency_violations);
  
  violations_.safety_violation_active = unsafe;
  return unsafe;
}

void PerformanceMonitor::ResetSafetyFlags() {
  violations_.safety_violation_active = false;
  violations_.consecutive_module_violations = 0;
  violations_.consecutive_frequency_violations = 0;
  
  SerialManager::GetInstance().SendMessage("PERF_SAFETY", "flags_reset=true");
}

void PerformanceMonitor::GenerateDetailedReport() {
  char detailed_report[512];
  snprintf(detailed_report, sizeof(detailed_report),
          "freq=%.1f,target=%.1f,mod_viol=%d,freq_viol=%d,total_mod=%lu,total_freq=%lu,unsafe=%s",
          current_loop_frequency_,
          config_.min_loop_frequency_hz,
          violations_.consecutive_module_violations,
          violations_.consecutive_frequency_violations,
          violations_.total_module_violations,
          violations_.total_frequency_violations,
          violations_.safety_violation_active ? "true" : "false");
  
  SerialManager::GetInstance().SendMessage("PERF_DETAILED", detailed_report);
}

void PerformanceMonitor::setup() {
  last_loop_time_us_ = micros();
  last_report_time_ms_ = millis();
  
  SerialManager::GetInstance().SendMessage("PERF_INIT", "monitor_started=true");
}

void PerformanceMonitor::loop() {
  uint32_t current_time_us = micros();
  uint32_t current_time_ms = millis();
  
  // Calculate loop frequency
  if (last_loop_time_us_ > 0) {
    uint32_t loop_time_us = current_time_us - last_loop_time_us_;
    if (loop_time_us > 0) {
      current_loop_frequency_ = 1000000.0f / loop_time_us;
    }
  }
  last_loop_time_us_ = current_time_us;
  loop_count_since_report_++;
  
  // Check for performance violations
  CheckLoopFrequency();
  CheckModulePerformance();
  
  // Periodic reporting
  if (current_time_ms - last_report_time_ms_ >= config_.stats_report_interval_ms) {
    SendPerformanceReport();
    last_report_time_ms_ = current_time_ms;
    loop_count_since_report_ = 0;
  }
}

void PerformanceMonitor::CheckLoopFrequency() {
  if (current_loop_frequency_ < config_.min_loop_frequency_hz) {
    violations_.consecutive_frequency_violations++;
    violations_.total_frequency_violations++;
    violations_.last_violation_time_ms = millis();
    
    // Report frequency violation
    char violation_msg[64];
    snprintf(violation_msg, sizeof(violation_msg),
            "freq=%.1f,target=%.1f,consecutive=%d",
            current_loop_frequency_,
            config_.min_loop_frequency_hz,
            violations_.consecutive_frequency_violations);
    
    ProcessViolation("FREQ_VIOLATION", violation_msg);
  } else {
    // Frequency is acceptable, reset consecutive counter
    violations_.consecutive_frequency_violations = 0;
  }
}

void PerformanceMonitor::CheckModulePerformance() {
  // Get current module count and check if we need to track more modules
  uint16_t current_module_count = Module::GetModuleCount();
  if (current_module_count > tracked_module_count_ && 
      current_module_count <= kMaxTrackedModules) {
    tracked_module_count_ = current_module_count;
  }
  
  bool any_module_violation = false;
  
  // Check each module's recent performance
  for (uint16_t i = 0; i < tracked_module_count_ && i < kMaxTrackedModules; ++i) {
    // Note: This is a simplified approach. In a full implementation,
    // we would need access to individual module statistics from the Module class.
    // For now, we'll rely on the Module class's built-in performance violation detection.
  }
  
  // Update consecutive violation counter based on overall module performance
  if (any_module_violation) {
    violations_.consecutive_module_violations++;
    violations_.total_module_violations++;
    violations_.last_violation_time_ms = millis();
    
    ProcessViolation("MODULE_VIOLATION", "modules_exceeding_time_limit");
  } else {
    violations_.consecutive_module_violations = 0;
  }
}

void PerformanceMonitor::ProcessViolation(const char* violation_type, const char* details) {
  if (config_.enable_detailed_logging) {
    char violation_msg[128];
    snprintf(violation_msg, sizeof(violation_msg),
            "type=%s,details=%s,time=%lu",
            violation_type, details, millis());
    SerialManager::GetInstance().SendMessage("PERF_VIOLATION", violation_msg);
  }
  
  // Check if we've reached unsafe levels
  if (IsUnsafe() && !violations_.safety_violation_active) {
    violations_.safety_violation_active = true;
    SerialManager::GetInstance().SendMessage("PERF_UNSAFE", "performance_critical=true");
  }
}

void PerformanceMonitor::SendPerformanceReport() {
  char report_msg[256];
  snprintf(report_msg, sizeof(report_msg),
          "freq=%.1f,loops=%lu,mod_viol=%d,freq_viol=%d,unsafe=%s",
          current_loop_frequency_,
          loop_count_since_report_,
          violations_.consecutive_module_violations,
          violations_.consecutive_frequency_violations,
          violations_.safety_violation_active ? "true" : "false");
  
  SerialManager::GetInstance().SendMessage("PERF_STATS", report_msg);
}

bool PerformanceMonitor::ValidateConfiguration(const PerformanceConfig& config) const {
  // Validate reasonable ranges for configuration parameters
  if (config.min_loop_frequency_hz < 10.0f || config.min_loop_frequency_hz > 1000.0f) {
    return false;  // Frequency out of reasonable range
  }
  
  if (config.max_module_time_ms < 0.1f || config.max_module_time_ms > 100.0f) {
    return false;  // Module time out of reasonable range
  }
  
  if (config.max_violation_count == 0 || config.max_violation_count > 100) {
    return false;  // Violation count out of reasonable range
  }
  
  if (config.max_frequency_violations == 0 || config.max_frequency_violations > 100) {
    return false;  // Frequency violation count out of reasonable range
  }
  
  if (config.stats_report_interval_ms < 100 || config.stats_report_interval_ms > 60000) {
    return false;  // Report interval out of reasonable range
  }
  
  return true;
}

void PerformanceMonitor::PrintMetrics() const {
  Serial.print("Performance Metrics - Loop Freq: ");
  Serial.print(current_loop_frequency_);
  Serial.print("Hz, Loops: ");
  Serial.print(loop_count_since_report_);
  Serial.print(", Module Violations: ");
  Serial.print(violations_.consecutive_module_violations);
  Serial.print("/");
  Serial.println(violations_.total_module_violations);
}

}  // namespace sigyn_teensy
