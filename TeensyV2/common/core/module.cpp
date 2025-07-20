/**
 * @file module.cpp
 * @brief Implementation of the core Module system for TeensyV2
 * 
 * This file implements the Module framework that provides automatic module
 * registration, centralized execution control, real-time performance monitoring,
 * and safety coordination for the TeensyV2 embedded system.
 * 
 * Key Implementation Details:
 * 
 * **Module Registration:**
 * - Modules self-register in their constructors (RAII pattern)
 * - Registration is O(1) and happens during static initialization
 * - Array-based storage for cache efficiency and deterministic access
 * - Maximum module count enforced to prevent memory exhaustion
 * 
 * **Execution Control:**
 * - setupAll(): One-time initialization of all registered modules
 * - loopAll(): Real-time execution with microsecond-precision timing
 * - Performance monitoring integrated into execution path
 * - Minimal overhead to preserve real-time characteristics
 * 
 * **Performance Monitoring:**
 * - Per-module execution time tracking with high resolution
 * - Automatic statistics collection for optimization analysis
 * - Safety violation detection integrated into main loop
 * - JSON report generation for external monitoring systems
 * 
 * **Safety Integration:**
 * - Global safety state aggregation across all modules
 * - Emergency response coordination through resetAllSafetyFlags()
 * - Fail-safe design principles throughout implementation
 * 
 * **Memory Management:**
 * - No dynamic allocation after initialization
 * - Fixed-size data structures for predictable memory usage
 * - Cache-friendly data layouts for ARM Cortex-M7 optimization
 * 
 * Thread Safety:
 * This implementation assumes single-threaded execution following the
 * Arduino programming model. No synchronization primitives are used.
 * 
 * @author Wimble Robotics
 * @date 2025
 * @version 2.0
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

Module::Module() {
  // Automatic registration when module is constructed (RAII pattern)
  // This ensures all modules participate in the framework without manual registration
  if (module_count_ < kMaxModules) {
    modules_[module_count_] = this;
    module_count_++;
  } else {
    // This is a critical error - we've exceeded our module limit
    // In a production system, this would trigger a hardware fault or safe shutdown
    // For now, we continue but the module won't be registered or executed
    // TODO: Consider triggering immediate E-stop or fault indicator
    is_setup_ = false;
  }
}

void Module::checkPerformanceAndSafety() {
  // This function is a placeholder for a more robust implementation.
  // The logic has been moved to the PerformanceMonitor and SafetyCoordinator.
}

void Module::getPerformanceStats(char* stats_json, size_t buffer_size) {
  // Start JSON object
  int written = snprintf(stats_json, buffer_size,
                         "{\"loop_count\":%lu,\"freq\":%.1f,\"modules\":[",
                         total_loop_count_,
                         total_loop_count_ > 0
                             ? total_loop_count_ * 1000.0f / millis()
                             : 0.0f);

  if (written < 0 || (size_t)written >= buffer_size) return;

  // Add module statistics (simplified - detailed stats now handled by PerformanceMonitor)
  for (uint16_t i = 0; i < module_count_; ++i) {
    if (modules_[i] != nullptr) {
      int module_written =
          snprintf(stats_json + written, buffer_size - written,
                   "%s{\"name\":\"%s\",\"last_exec\":%.1f}",
                   i > 0 ? "," : "", modules_[i]->name(),
                   modules_[i]->getLastExecutionTimeUs() / 1000.0f);  // Convert to ms

      if (module_written < 0 || written + module_written >= (int)buffer_size)
        break;
      written += module_written;
    }
  }

  // Close JSON object
  if (written < (int)buffer_size - 2) {
    snprintf(stats_json + written, buffer_size - written, "]}");
  }
}

bool Module::isAnyModuleUnsafe() {
  for (uint16_t i = 0; i < module_count_; ++i) {
    if (modules_[i] != nullptr && modules_[i]->isUnsafe()) {
      return true;
    }
  }
  return false;
}

void Module::loopAll() {
  // Record loop start time for frequency calculation and performance monitoring
  loop_start_time_us_ = micros();

  // Execute all registered modules with high-precision timing measurement
  // This is the critical real-time path - minimize overhead here
  for (uint16_t i = 0; i < module_count_; ++i) {
    if (modules_[i] != nullptr) {
      // Measure individual module execution time with microsecond precision
      uint32_t module_start_us = micros();

      // Execute module's main loop - this must complete quickly (<2ms target)
      // Modules should use state machines for long operations
      modules_[i]->loop();

      // Calculate and store execution time for performance monitoring
      uint32_t execution_time_us = micros() - module_start_us;
      modules_[i]->last_execution_time_us_ = execution_time_us;
      
      // Note: Detailed performance statistics are now handled by PerformanceMonitor
      // This keeps the critical path lean while still providing basic timing data
    }
  }

  // Update global loop counter for frequency calculations
  total_loop_count_++;

  // Note: Performance violation detection and reporting are now handled
  // by the dedicated PerformanceMonitor module to maintain separation of concerns
}

void Module::reportPerformanceStats() {
  // This function is a placeholder. Performance reporting is now handled by
  // the PerformanceMonitor module.
}

void Module::resetAllSafetyFlags() {
  for (uint16_t i = 0; i < module_count_; ++i) {
    if (modules_[i] != nullptr) {
      modules_[i]->resetSafetyFlags();
    }
  }
}

void Module::setupAll() {
  // Initialize SerialManager first, as other modules may depend on it for logging
  // This establishes communication early for debugging and error reporting
  SerialManager::getInstance().sendMessage("INIT", "starting_module_setup");

  // Initialize all registered modules in registration order
  // Order dependencies should be handled by controlling registration order
  // through careful static initialization sequencing
  for (size_t i = 0; i < module_count_; ++i) {
    Module* mod = modules_[i];
    if (mod) {
      // Call module-specific setup - this may block for hardware initialization
      // Modules should validate hardware and report errors clearly
      mod->setup();
      
      // Log successful initialization for debugging and system health monitoring
      char init_msg[50];
      snprintf(init_msg, sizeof(init_msg), "module_initialized:%s",
               mod->name());
      SerialManager::getInstance().sendMessage("INIT", init_msg);
    }
  }
  SerialManager::getInstance().sendMessage("INIT", "module_setup_complete");
}

void Module::updatePerformanceStats(Module* module,
                                    uint32_t execution_time_us) {
  // PerformanceStats& stats = module->stats_;

  // // Update timing statistics
  // if (execution_time_us < stats.duration_min_us) {
  //   stats.duration_min_us = execution_time_us;
  // }
  // if (execution_time_us > stats.duration_max_us) {
  //   stats.duration_max_us = execution_time_us;
  // }
  
  // stats.duration_sum_us += execution_time_us;
  // stats.loop_count++;
  // module->last_execution_time_us_ = execution_time_us;
}

}  // namespace sigyn_teensy
