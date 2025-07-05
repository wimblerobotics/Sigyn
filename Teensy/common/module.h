/*
 * module.h - Base class for modular functionality on Teensy microcontroller
 * 
 * The Module class provides a framework for adding functionality to the Teensy application
 * by simply creating subclasses and instantiating them in the main program. This design
 * allows for clean separation of concerns and easy addition of new features.
 * 
 * Usage Pattern:
 * 1. Create a subclass of Module (e.g., BNO055Module, MotorModule)
 * 2. Implement the singleton pattern with automatic registration
 * 3. Create an instance in main program (.ino file) - the singleton registers itself automatically
 * 4. Module::Setup() and Module::Loop() will handle all registered modules
 * 
 * Timing Constraints:
 * - Constructor & setup(): Can take any reasonable time (blocking operations OK)
 *   These are called once during initialization before real-time operation begins
 * 
 * - loop(): Must be VERY FAST (≤ 2ms recommended)
 *   Called repeatedly at high frequency to maintain real-time performance
 *   For long operations, implement a state machine to break work into small chunks
 * 
 * Performance Requirements:
 * The system targets ~100Hz operation for critical sensors (motor control, odometry).
 * With multiple modules, each loop() call should complete in 1-2ms to maintain
 * overall system performance.
 * 
 * Statistics & Monitoring:
 * Module automatically tracks and reports performance statistics every second:
 * - Total loops per second across all modules
 * - Min/max/average execution time per module (in fractional milliseconds)
 * - Helps identify performance bottlenecks and timing violations
 * 
 * Example Subclass:
 * 
 * class MyModule : public Module {
 * public:
 *   static MyModule& singleton() {
 *     if (!instance_) instance_ = new MyModule();
 *     return *instance_;
 *   }
 * protected:
 *   MyModule() : Module() { // blocking init OK here }
 *   void setup() override { // one-time setup }
 *   void loop() override { // FAST operations only! }
 *   const char* name() override { return "MyModule"; }
 * private:
 *   static MyModule* instance_;
 * };
 * 
 * // In main():
 * MyModule::singleton(); // Registers automatically
 */

#pragma once

#include "Arduino.h"

/*
 * Base class for modular functionality system
 * 
 * Provides framework for registering modules and managing their lifecycle.
 * Modules automatically register themselves when their singleton is created.
 */
class Module {
 public:
  virtual ~Module() {}

  /*
   * Initialize all registered modules
   * 
   * Calls setup() on each registered module. This is called once during
   * system initialization and may take significant time as modules perform
   * blocking initialization operations.
   */
  static void Setup();

  /*
   * Execute one iteration of all registered modules
   * 
   * Calls loop() on each registered module and tracks timing statistics.
   * This is called repeatedly at high frequency (~100Hz target) and must
   * complete quickly to maintain real-time performance.
   * 
   * Each module's loop() should complete in ≤2ms. For longer operations,
   * implement a state machine to break work into smaller chunks.
   */
  static void Loop();

  /*
   * Check if module detects an unsafe condition
   * Returns true if unsafe condition detected, false otherwise
   * 
   * Override this to implement safety monitoring specific to your module.
   * Called by safety systems to determine if emergency actions are needed.
   */
  virtual bool isUnsafe() { return false; }

  /*
   * Reset any unsafe condition flags within the module
   * 
   * Override this to clear error states and attempt recovery from
   * unsafe conditions. Called after safety systems handle the emergency.
   */
  virtual void resetSafetyFlags() {}

  /*
   * Get the name of this module
   * Returns Module name string
   * 
   * Used for debugging, statistics reporting, and system identification.
   */
  const char* getName() { return name(); }

 protected:
  /*
   * Protected constructor - only subclass singletons should create instances
   * 
   * Automatically registers this module with the global module system.
   * This ensures all modules are tracked for setup/loop calls and statistics.
   */
  Module();

  /*
   * Perform regular, cyclic work for the module
   * 
   * CRITICAL: This must execute very quickly (≤2ms recommended)!
   * 
   * This is called repeatedly at high frequency to maintain real-time
   * performance. For operations that might take longer:
   * - Implement a state machine
   * - Break work into small chunks across multiple calls
   * - Use non-blocking I/O operations
   * - Cache results and update periodically
   * 
   * WARNING: Long-running operations in loop() will degrade system performance
   *          and may cause control loops to miss their timing requirements.
   */
  virtual void loop() = 0;

  /*
   * Return the name of this module
   * Returns Module name string (must be compile-time constant)
   * 
   * Used for identification in statistics and debugging output.
   * Should return a descriptive name like "BNO055", "MotorController", etc.
   */
  virtual const char* name() = 0;

  /*
   * Perform one-time initialization for the module
   * 
   * This is called once during system startup, after all module constructors
   * have completed. Unlike loop(), this may take significant time as it's
   * not called during real-time operation.
   * 
   * Use this for:
   * - Sensor calibration
   * - Network connections
   * - File system operations
   * - Other initialization that might block
   */
  virtual void setup() = 0;

 private:
  /*
   * Performance statistics for timing analysis
   * 
   * Tracks min/max/average execution times for each module's loop() function.
   * Used to identify performance bottlenecks and verify timing constraints.
   */
  typedef struct Statistics {
    float duration_min_us = MAXFLOAT;  // Minimum execution time (microseconds)
    float duration_max_us = 0.0f;      // Maximum execution time (microseconds)  
    float duration_sum_us = 0.0f;      // Cumulative execution time (microseconds)
  } Statistics;

  // Prevent copying and moving - modules should be singletons
  Module(const Module&) = delete;             // Disable copy constructor
  Module& operator=(const Module&) = delete;  // Disable assignment operator
  Module(Module&&) = delete;                  // Disable move constructor
  Module& operator=(Module&&) = delete;       // Disable move assignment operator

  /*
   * Generate performance statistics report
   * 
   * outString: Buffer to write statistics to
   * outStringSize: Size of output buffer
   * 
   * Creates a formatted string with performance metrics for all modules:
   * - Loops per second across all modules
   * - Min/max/average execution time per module
   * - Performance warnings for slow modules
   */
  static void GetStatistics(char* outString, unsigned int outStringSize);

  // This module's performance statistics
  Statistics statistics_;
  
  // Timestamp when current loop() call started (microseconds)
  unsigned long loop_start_time_us_ = 0;

  // Maximum number of modules that can be registered
  static const int MAX_NUMBER_OF_MODULES = 10;  // Adjust as needed
  
  // Array of all registered module instances
  static Module* all_modules_[MAX_NUMBER_OF_MODULES];

  // Total number of loop iterations across all modules
  static uint32_t total_do_loop_count_;

  // Index for the next module to be registered
  static uint16_t g_next_module_index_;
};
