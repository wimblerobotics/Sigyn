/**
 * @file module.cpp
 * @brief Implementation of the core Module system for TeensyV2
 * 
 * Provides automatic module registration, performance monitoring, and
 * safety coordination for the real-time embedded framework.
 * 
 * @author Wimble Robotics
 * @date 2025
 */

#include "module.h"
#include "serial_manager.h"

namespace sigyn_teensy {

// Static member definitions
Module* Module::modules_[Module::kMaxModules] = {nullptr};
uint16_t Module::module_count_ = 0;
uint32_t Module::total_loop_count_ = 0;
uint32_t Module::last_stats_report_ms_ = 0;
uint32_t Module::loop_start_time_us_ = 0;

void Module::CheckPerformanceAndSafety() {
  // Check for modules exceeding maximum execution time
  for (uint16_t i = 0; i < module_count_; ++i) {
    if (modules_[i] != nullptr) {
      if (modules_[i]->last_execution_time_us_ > kMaxLoopTimeUs) {
        // Performance violation detected
        char violation_msg[128];
        snprintf(violation_msg, sizeof(violation_msg),
                "module=%s,time_us=%lu,limit_us=%.0f",
                modules_[i]->GetName(),
                modules_[i]->last_execution_time_us_,
                kMaxLoopTimeUs);
        SerialManager::GetInstance().SendMessage("PERF_VIOLATION", violation_msg);
      }
    }
  }
  
  // Check overall loop frequency
  uint32_t total_loop_time_us = micros() - loop_start_time_us_;
  float current_frequency = total_loop_time_us > 0 ? 
                           1000000.0f / total_loop_time_us : 100'000.0f;
  
  if (current_frequency < kTargetLoopFrequencyHz) {
    char freq_violation_msg[64];
    snprintf(freq_violation_msg, sizeof(freq_violation_msg),
            "freq=%.1f,target=%.1f", current_frequency, kTargetLoopFrequencyHz);
    SerialManager::GetInstance().SendMessage("FREQ_VIOLATION", freq_violation_msg);
  }
}

void Module::GetPerformanceStats(char* stats_json, size_t buffer_size) {
  // Start JSON object
  int written = snprintf(stats_json, buffer_size, 
                        "{\"loop_count\":%lu,\"freq\":%.1f,\"modules\":[",
                        total_loop_count_, 
                        total_loop_count_ > 0 ? 
                          total_loop_count_ * 1000.0f / millis() : 0.0f);
  
  if (written < 0 || (size_t)written >= buffer_size) return;
  
  // Add module statistics
  for (uint16_t i = 0; i < module_count_; ++i) {
    if (modules_[i] != nullptr) {
      const PerformanceStats& stats = modules_[i]->GetStats();
      float avg_time_us = stats.loop_count > 0 ? 
                         stats.duration_sum_us / stats.loop_count : 0.0f;
      
      int module_written = snprintf(stats_json + written, buffer_size - written,
                                   "%s{\"name\":\"%s\",\"min\":%.1f,\"max\":%.1f,\"avg\":%.1f}",
                                   i > 0 ? "," : "",
                                   modules_[i]->GetName(),
                                   stats.duration_min_us / 1000.0f,  // Convert to ms
                                   stats.duration_max_us / 1000.0f,
                                   avg_time_us / 1000.0f);
      
      if (module_written < 0 || written + module_written >= (int)buffer_size) break;
      written += module_written;
    }
  }
  
  // Close JSON object
  if (written < (int)buffer_size - 2) {
    snprintf(stats_json + written, buffer_size - written, "]}");
  }
}

bool Module::IsAnyModuleUnsafe() {
  for (uint16_t i = 0; i < module_count_; ++i) {
    if (modules_[i] != nullptr && modules_[i]->IsUnsafe()) {
      return true;
    }
  }
  return false;
}

void Module::LoopAll() {
  loop_start_time_us_ = micros();
  
  // Execute all modules and track performance
  for (uint16_t i = 0; i < module_count_; ++i) {
    if (modules_[i] != nullptr) {
      uint32_t module_start_us = micros();
      
      // Execute module's main loop
      modules_[i]->loop();
      
      uint32_t execution_time_us = micros() - module_start_us;
      
      // Update performance statistics
      UpdatePerformanceStats(modules_[i], execution_time_us);
    }
  }
  
  total_loop_count_++;
  
  // Check for performance violations and safety conditions
  CheckPerformanceAndSafety();
  
  // Periodic statistics reporting
  uint32_t current_time_ms = millis();
  if (current_time_ms - last_stats_report_ms_ >= 1000) {  // Every 1 second
    ReportPerformanceStats();
    last_stats_report_ms_ = current_time_ms;
  }
}

Module::Module() {
  // Automatic registration when module is constructed
  if (module_count_ < kMaxModules) {
    modules_[module_count_] = this;
    module_count_++;
  } else {
    // This is a critical error - we've exceeded our module limit
    // In a real system, this might trigger a hardware fault
    // For now, we'll continue but the module won't be registered
  }
}

void Module::ReportPerformanceStats() {
  char stats_buffer[1024];
  GetPerformanceStats(stats_buffer, sizeof(stats_buffer));
  SerialManager::GetInstance().SendMessage("PERF", stats_buffer);
  
  // Reset statistics for next reporting period
  for (uint16_t i = 0; i < module_count_; ++i) {
    if (modules_[i] != nullptr) {
      PerformanceStats& stats = modules_[i]->stats_;
      stats.duration_min_us = MAXFLOAT;
      stats.duration_max_us = 0.0f;
      stats.duration_sum_us = 0.0f;
      stats.loop_count = 0;
    }
  }
  
  total_loop_count_ = 0;
}

void Module::ResetAllSafetyFlags() {
  for (uint16_t i = 0; i < module_count_; ++i) {
    if (modules_[i] != nullptr) {
      modules_[i]->ResetSafetyFlags();
    }
  }
}

void Module::SetupAll() {
  SerialManager::GetInstance().SendMessage("INIT", "starting_module_setup");
  
  for (uint16_t i = 0; i < module_count_; ++i) {
    if (modules_[i] != nullptr) {
      uint32_t start_time = millis();
      
      modules_[i]->setup();
      
      uint32_t setup_time = millis() - start_time;
      
      // Report module initialization time
      char init_msg[128];
      snprintf(init_msg, sizeof(init_msg), "module=%s,setup_time_ms=%lu", 
               modules_[i]->GetName(), setup_time);
      SerialManager::GetInstance().SendMessage("INIT", init_msg);
    }
  }
  
  SerialManager::GetInstance().SendMessage("INIT", "module_setup_complete");
}

void Module::UpdatePerformanceStats(Module* module, uint32_t execution_time_us) {
  PerformanceStats& stats = module->stats_;
  
  // Update timing statistics
  if (execution_time_us < stats.duration_min_us) {
    stats.duration_min_us = execution_time_us;
  }
  if (execution_time_us > stats.duration_max_us) {
    stats.duration_max_us = execution_time_us;
  }
  
  stats.duration_sum_us += execution_time_us;
  stats.loop_count++;
  module->last_execution_time_us_ = execution_time_us;
}

}  // namespace sigyn_teensy
