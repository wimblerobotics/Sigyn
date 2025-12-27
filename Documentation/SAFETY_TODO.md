# Safety System TODO - Critical Gaps and Recommendations

## Overview

Based on detailed code analysis of the TeensyV2 safety system, this document identifies critical gaps between what is configured/monitored and what actually triggers safety responses. The most significant gap is that **BatteryMonitor has all the infrastructure for safety but doesn't implement `isUnsafe()`**.

## Critical Priority Issues

### 1. Battery Safety Not Implemented (CRITICAL)

**Problem**: BatteryMonitor monitors voltages and currents but does not implement safety enforcement.

**Current State**:
- ✅ INA226 sensors configured and reading voltage/current
- ✅ Critical thresholds defined in `BatteryConfig` (32.0V critical, 34.0V warning, 20.0A critical current)
- ✅ Battery state tracking (CHARGING, DISCHARGING, CRITICAL, WARNING, NORMAL)
- ❌ **No `isUnsafe()` implementation** - uses default Module behavior (always returns `false`)
- ❌ No emergency stops triggered by battery conditions

**Required Implementation**:
```cpp
// In battery_monitor.h, add to protected section:
bool isUnsafe() override;
void resetSafetyFlags() override;

// In battery_monitor.cpp, implement:
bool BatteryMonitor::isUnsafe() {
  // Check main battery (index 0) - the 36V LiPo pack
  if (state_[0] == BatteryState::CRITICAL) {
    return true;
  }
  
  // Check for critical low voltage
  if (voltage_ema_[0] < g_battery_config_[0].critical_low_voltage && 
      total_readings_[0] > 10) { // Ensure valid readings
    return true;
  }
  
  // Check for critical high current (overcurrent condition)
  if (abs(current_ema_[0]) > g_battery_config_[0].critical_high_current && 
      total_readings_[0] > 10) {
    return true;
  }
  
  return false;
}

void BatteryMonitor::resetSafetyFlags() {
  // Reset critical states that can be recovered
  for (size_t i = 0; i < kNumberOfBatteries; i++) {
    if (state_[i] == BatteryState::CRITICAL && 
        voltage_ema_[i] > g_battery_config_[i].critical_low_voltage) {
      state_[i] = BatteryState::NORMAL;
    }
  }
}
```

**Impact**: Without this, the robot has **no battery protection** at the embedded level.

### 2. Safety Message Sources Not Mapped

**Problem**: SafetyCoordinator uses `EstopSource::UNKNOWN` for module safety violations.

**Current Code**:
```cpp
activateEstop(EstopSource::UNKNOWN, desc); // TODO: Better source mapping
```

**Required Enhancement**:
- Add module-specific E-stop sources
- Map module names to specific E-stop sources
- Provide detailed safety violation context

### 3. Inconsistent Safety Threshold Enforcement

**Problem**: Thresholds are configured but not consistently enforced across modules.

**Examples**:
- BatteryMonitor: Thresholds configured but not enforced
- RoboClawMonitor: Thresholds configured AND enforced
- Temperature sensors: Unknown enforcement status

## High Priority Improvements

### 4. Enhanced Motor Safety

**Current Limitations**:
- Runaway detection threshold hardcoded at 100 QPPS
- Recovery mechanisms not fully tested
- Limited motor state validation

**Recommended Enhancements**:
```cpp
// Configurable runaway detection
struct RoboClawConfig {
  // Add configurable thresholds
  int32_t runaway_speed_threshold_qpps = 100;
  uint32_t runaway_detection_delay_ms = 500;
  bool enable_auto_recovery = true;
  
  // Enhanced current monitoring
  float current_violation_duration_ms = 100; // Sustained overcurrent time
  bool enable_current_ramping = false; // Gradual current limiting
};
```

### 5. Sensor Communication Safety

**Current State**: Sensors may monitor communication but don't enforce safety.

**Required Implementation**:
- Systematic timeout detection across all sensors
- Safety responses for critical sensor failures
- Sensor redundancy validation

### 6. Temperature Safety Integration

**Missing Features**:
- Temperature-based safety thresholds
- Thermal shutdown procedures
- Temperature trend analysis for predictive safety

## Medium Priority Enhancements

### 7. Safety System Diagnostics

**Needed Features**:
- Safety system self-test procedures
- Historical safety violation tracking
- Safety performance metrics
- Diagnostic commands for testing safety responses

### 8. Coordinated Safety Responses

**Current Limitation**: Each module handles safety independently.

**Recommended Architecture**:
```cpp
// Enhanced safety coordination
class SafetyCoordinator {
  // Add coordinated shutdown procedures
  void initiateControlledShutdown(const String& reason);
  void executeEmergencySequence(EstopSource source);
  void validateSystemRecovery();
  
  // Add safety state transitions
  enum SafetyState {
    NORMAL,
    WARNING,        // New: Non-critical but concerning
    DEGRADED,       // New: Reduced capability mode
    EMERGENCY_STOP,
    RECOVERY        // New: Recovery in progress
  };
};
```

### 9. Inter-Board Safety Enhancements

**Current Implementation**: Basic digital signal propagation.

**Enhancements Needed**:
- Safety message protocol between boards
- Cross-validation of safety states
- Coordinated recovery procedures

### 10. ROS2 Safety Integration

**Missing Features**:
- Navigation stack safety integration
- Behavior tree safety node integration
- Safety-aware path planning
- Emergency return-to-base procedures

## Low Priority Improvements

### 11. Advanced Battery Management

**Beyond basic safety**:
- Individual cell monitoring (if hardware supports)
- Battery degradation tracking
- Predictive battery failure detection
- Smart charging safety

### 12. Performance Safety Enhancements

**Current**: Basic execution time monitoring.

**Enhancements**:
- Memory usage safety monitoring
- Real-time constraint validation
- System load balancing for safety

### 13. Storage Safety Improvements

**Beyond current write error detection**:
- Critical data backup strategies
- Storage capacity management
- Data integrity verification

## Implementation Priorities

### Phase 1 (URGENT - Safety Critical)
1. **Implement BatteryMonitor::isUnsafe()** - Critical battery safety
2. **Map safety sources properly** - Better diagnostics
3. **Test all existing safety mechanisms** - Validation

### Phase 2 (HIGH - System Reliability)
1. Enhanced motor safety with configurable thresholds
2. Sensor communication timeout safety
3. Safety system self-diagnostics

### Phase 3 (MEDIUM - Advanced Features)
1. Coordinated safety responses
2. Temperature safety integration
3. ROS2 safety integration improvements

### Phase 4 (LOW - Optimization)
1. Advanced battery management
2. Performance safety enhancements
3. Storage safety improvements

## Testing Requirements

### Safety System Validation Needed

1. **BatteryMonitor Safety Testing**:
   - Verify low voltage triggers E-stop
   - Verify overcurrent triggers E-stop  
   - Test recovery when conditions improve
   - Validate threshold accuracy

2. **Module Safety Integration**:
   - Test `Module::isAnyModuleUnsafe()` with all modules
   - Verify SafetyCoordinator responds to each module
   - Test recovery procedures

3. **Communication Safety**:
   - Test RoboClaw communication timeout safety
   - Test serial communication to ROS2 during safety events
   - Test inter-board safety signal propagation

4. **Hardware Integration**:
   - Test physical E-stop button
   - Test hardware E-stop output signal
   - Verify fail-safe behavior on power loss

## Architecture Recommendations

### 1. Safety-First Module Design
All modules should implement safety as a primary concern, not an afterthought.

### 2. Configuration Validation
Add compile-time and runtime validation of safety configuration consistency.

### 3. Safety Event Logging
Comprehensive logging of all safety events for analysis and improvement.

### 4. Graceful Degradation
Implement degraded operation modes rather than complete shutdowns when possible.

## Conclusion

The TeensyV2 safety system has a solid architectural foundation but critical gaps in implementation. The most urgent issue is implementing battery safety, which is completely absent despite having all the necessary monitoring infrastructure. 

The safety system shows good design patterns (modular safety, central coordination, hardware integration) but needs completion of the safety enforcement logic, particularly in the BatteryMonitor module.