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

PerformanceMonitor::PerformanceMonitor()
    : loop_start_time_us_(0),
      last_stats_send_time_ms_(0),
      module_start_time_us_(0),
      current_module_name_(nullptr) {}

void PerformanceMonitor::beginModule(const char* module_name) {
  module_start_time_us_ = micros();
  current_module_name_ = module_name;
}

void PerformanceMonitor::checkLoopFrequency() {
  uint32_t loop_time_us = micros() - loop_start_time_us_;
  float loop_frequency_hz = 1000000.0f / loop_time_us;

  if (loop_frequency_hz < config_.min_loop_frequency_hz) {
    violations_.consecutive_frequency_violations++;
    violations_.total_frequency_violations++;
    violations_.last_violation_time_ms = millis();
  } else {
    violations_.consecutive_frequency_violations = 0;
  }
}

void PerformanceMonitor::endModule(const char* module_name) {
  uint32_t module_time_us = micros() - module_start_time_us_;
  float module_time_ms = module_time_us / 1000.0f;

  if (module_time_ms > config_.max_module_time_ms) {
    violations_.consecutive_module_violations++;
    violations_.total_module_violations++;
    violations_.last_violation_time_ms = millis();
  } else {
    violations_.consecutive_module_violations = 0;
  }
  current_module_name_ = nullptr;
}

PerformanceMonitor& PerformanceMonitor::getInstance() {
  static PerformanceMonitor instance;
  return instance;
}

void PerformanceMonitor::loop() {
  loop_start_time_us_ = micros();

  checkLoopFrequency();

  if (millis() - last_stats_send_time_ms_ > config_.stats_report_interval_ms) {
    sendStatistics();
    last_stats_send_time_ms_ = millis();
  }
}

const char* PerformanceMonitor::name() const {
  return "PerformanceMonitor";
}

void PerformanceMonitor::sendStatistics() {
  char stats_msg[128];
  snprintf(stats_msg, sizeof(stats_msg), "freq_viol=%d, mod_viol=%d",
           violations_.consecutive_frequency_violations,
           violations_.consecutive_module_violations);
  SerialManager::getInstance().sendMessage("PERF_STATS", stats_msg);
}

void PerformanceMonitor::setup() {
  // Initialization code for PerformanceMonitor, if any
}

}  // namespace sigyn_teensy
