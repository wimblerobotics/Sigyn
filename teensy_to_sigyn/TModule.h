#pragma once

#include "Arduino.h"

class TModule {
 public:
  virtual ~TModule() {}

  // Called once during setup
  static void Setup();

  // Called repeatedly in the main loop
  // Should execute quickly or use a state machine
  static void Loop();

  // Returns true if the module detects an unsafe condition
  virtual bool IsUnsafe() { return false; }

  // Resets any unsafe condition flags within the module
  virtual void ResetSafetyFlags() {}

  // Returns the name of the module
  const char* GetName() const { return module_name_; }

 protected:
  TModule();

  // Perform regular, cyclic work for the module.
  virtual void loop() = 0;

  // Return module name.
  virtual const char* name() = 0;

  // Perform one-time setup for the module.
  virtual void setup() = 0;

  const char* module_name_;

 private:
  typedef struct Statistics {
    float duration_min_us = MAXFLOAT;
    float duration_max_us = 0.0f;
    float duration_sum_us = 0.0f;
  } Statistics;

  TModule(const TModule&) = delete;             // Disable copy constructor
  TModule& operator=(const TModule&) = delete;  // Disable assignment operator
  TModule(TModule&&) = delete;                  // Disable move constructor
  TModule& operator=(TModule&&) = delete;  // Disable move assignment operator

  static void GetStatistics(char* outString, size_t outStringSize);

  Statistics statistics_;
  unsigned long loop_start_time_us_ = 0;

  static const int MAX_NUMBER_OF_MODULES = 10;  // Adjust as needed
  static TModule* all_modules_[MAX_NUMBER_OF_MODULES];

  // Number of times DoLoop() was called.
  static uint32_t total_do_loop_count_;

  // Index for the next module to be added
  static uint16_t g_next_module_index_;
};
