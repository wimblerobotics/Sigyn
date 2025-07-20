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
  // This function is a placeholder for a more robust implementation.
  // The logic has been moved to the PerformanceMonitor and SafetyCoordinator.
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
                                   modules_[i]->name(),
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
    is_setup_ = false;
  }
}

void Module::ReportPerformanceStats() {
  // This function is a placeholder. Performance reporting is now handled by
  // the PerformanceMonitor module.
}

void Module::ResetAllSafetyFlags() {
  for (uint16_t i = 0; i < module_count_; ++i) {
    if (modules_[i] != nullptr) {
      modules_[i]->ResetSafetyFlags();
    }
  }
}

void Module::SetupAll() {
  // Initialize SerialManager first, as other modules may depend on it for
  // logging.
  SerialManager::getInstance().sendMessage("INIT", "starting_module_setup");

  for (size_t i = 0; i < module_count_; ++i) {
    Module* mod = modules_[i];
    if (mod) {
      mod->setup();
      char init_msg[50];
      snprintf(init_msg, sizeof(init_msg), "module_initialized:%s",
               mod->name());
      SerialManager::getInstance().sendMessage("INIT", init_msg);
    }
  }
  SerialManager::getInstance().sendMessage("INIT", "module_setup_complete");
}

void Module::UpdatePerformanceStats(Module* module,
                                    uint32_t execution_time_us) {
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
