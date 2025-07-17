# Performance Monitor Module Specification

## Purpose
Monitor real-time performance of Teensy modules and trigger safety violations when timing constraints are violated.

## Requirements
- Target: 80-100Hz main loop frequency
- Detect when frequency drops below threshold for N consecutive cycles
- Configurable thresholds per module type
- Integration with safety system for emergency stops
- Diagnostic reporting to ROS2

## Implementation Plan
```cpp
class PerformanceMonitor : public Module {
private:
  struct PerformanceThresholds {
    float min_loop_frequency_hz = 80.0f;
    uint8_t max_violation_count = 5;
    float max_module_time_ms = 2.0f;
  };
  
  PerformanceThresholds thresholds_;
  uint8_t consecutive_violations_ = 0;
  unsigned long last_loop_time_us_;
  
public:
  bool isUnsafe() override; // Returns true if performance violations detected
  void resetSafetyFlags() override;
  
private:
  void checkLoopFrequency();
  void checkModulePerformance();
  void triggerSafetyViolation();
};
```

## Configuration
Load thresholds from config.h or EEPROM:
- MIN_LOOP_FREQUENCY_HZ = 80
- MAX_VIOLATION_COUNT = 5  
- MAX_MODULE_TIME_MS = 2.0
