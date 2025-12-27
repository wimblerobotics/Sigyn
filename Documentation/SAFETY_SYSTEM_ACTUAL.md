# Safety System Architecture - Sigyn TeensyV2 (ACTUAL IMPLEMENTATION)

## Overview

This document describes the **actually implemented** safety features in the Sigyn TeensyV2 system, based on detailed code analysis. It corrects previous documentation that included speculative or planned features.

The Sigyn robot employs a modular safety system designed to prevent damage and ensure safe operation. The safety system spans both embedded (TeensyV2) and ROS2 (sigyn_to_sensor_v2) components.

## Core Safety Philosophy

The safety system is built on these principles:
- **Fail-Safe Design**: Safety mechanisms default to the safest state on failure
- **Modular Safety**: Each module can report unsafe conditions via `isUnsafe()`
- **Central Coordination**: SafetyCoordinator manages emergency stops and system-wide safety
- **Hardware E-stop**: Physical emergency stop capability with immediate motor cutoff

## Actually Implemented Safety Components

### 1. SafetyCoordinator (Central Safety Management)

**Location**: `TeensyV2/modules/safety/safety_coordinator.h/cpp`

The SafetyCoordinator serves as the central safety manager, coordinating emergency stops and monitoring system-wide safety conditions.

**Actually Implemented Features**:
- Hardware emergency stop button monitoring (`digitalRead(config_.hardware_estop_pin)`)
- Inter-board safety signal coordination (input/output pins)
- Module safety aggregation via `Module::isAnyModuleUnsafe()`
- Emergency stop state management
- Safety status reporting via serial JSON messages

**Safety States**:
- `NORMAL`: System operating normally
- `EMERGENCY_STOP`: Emergency condition detected, all systems halted

**E-stop Sources**:
- `HARDWARE_ESTOP`: Physical emergency stop button pressed
- `INTER_BOARD`: Safety signal from other boards
- `MODULE_SAFETY`: Safety violation detected in a module

**Key Methods**:
```cpp
bool isUnsafe() { return current_state_ == SafetyState::EMERGENCY_STOP; }
void activateEstop(EstopSource source, const String& description);
void checkSafetyStatus(); // Calls checkModuleSafety()
void checkModuleSafety(); // Iterates through all modules checking isUnsafe()
```

**Safety Check Flow**:
```cpp
void SafetyCoordinator::checkModuleSafety() {
  if (Module::isAnyModuleUnsafe()) {
    // Find which module is unsafe
    for (uint16_t i = 0; i < Module::getModuleCount(); ++i) {
      Module* mod = Module::getModule(i);
      if (mod && mod->isUnsafe()) {
        activateEstop(EstopSource::UNKNOWN, "Module unsafe: " + mod->name());
        break;
      }
    }
  }
}
```

### 2. RoboClawMonitor (Motor Safety System)

**Location**: `TeensyV2/modules/roboclaw/roboclaw_monitor.h/cpp`

The RoboClawMonitor provides motor safety including overcurrent detection, runaway detection, and communication monitoring.

**Actually Implemented Safety Features**:

#### Overcurrent Detection
- **Detection Logic**: Compares actual motor current against configurable thresholds
- **Thresholds**: `config_.max_current_m1` and `config_.max_current_m2` 
- **Response**: Sets `motor_status_.overcurrent = true` and sends CRITICAL message
- **Auto-Recovery**: Clears overcurrent flags when current drops below 0.5A

```cpp
if (motor1_status_.current_valid && 
    abs(motor1_status_.current_amps) > config_.max_current_m1) {
  motor1_status_.overcurrent = true;
  // Send CRITICAL message: "active:true,source:ROBOCLAW_CURRENT,reason:Motor 1 overcurrent"
}
```

#### Runaway Detection
- **Definition**: Motor speed > 100 QPPS when commanded speed = 0
- **Detection**: Compares `last_commanded_m*_qpps_` vs actual `motor*_status_.speed_qpps`
- **Response**: Sets `motor_status_.runaway_detected = true` and sends CRITICAL message
- **Rate Limiting**: 1 second intervals between runaway messages

```cpp
if (last_commanded_m1_qpps_ == 0 && abs(motor1_status_.speed_qpps) > 100) {
  motor1_status_.runaway_detected = true;
  // Send CRITICAL message: "active:true,source:MOTOR_RUNAWAY,reason:Motor 1 runaway detected"
}
```

#### Communication Safety  
- **Timeout Detection**: Monitors communication with RoboClaw controller
- **Error Status**: Checks RoboClaw internal error flags including `M1_OVERCURRENT`, `M2_OVERCURRENT`
- **Response**: Communication failures and hardware errors trigger safety violations

**Safety Interface**:
```cpp
bool isUnsafe() override {
  return emergency_stop_active_ || 
         motor1_status_.runaway_detected || motor2_status_.runaway_detected ||
         motor1_status_.overcurrent || motor2_status_.overcurrent ||
         (system_status_.error_status & critical_error_mask) ||
         communication_timeout_;
}
```

### 3. PerformanceMonitor (Timing Safety)

**Location**: `TeensyV2/modules/performance/performance_monitor.h/cpp`

Ensures system timing constraints are met to prevent real-time violations.

**Actually Implemented Features**:
- **Module Execution Time Monitoring**: Tracks individual module loop times
- **Violation Detection**: Detects when modules exceed `config_.max_module_time_ms`
- **Safety Integration**: Reports unsafe conditions when timing violations persist

**Safety Interface**:
```cpp
bool isUnsafe() { return violations_.safety_violation_active; }
```

### 4. SDLogger (Storage Safety)

**Location**: `TeensyV2/modules/storage/sd_logger.h/cpp`

Monitors storage system health and prevents failures due to storage issues.

**Actually Implemented Features**:
- **Write Error Detection**: Counts and monitors SD card write failures
- **Card Presence Monitoring**: Detects SD card removal/insertion
- **Error Threshold**: Becomes unsafe after > 10 write errors
- **Recovery**: Provides `resetSafetyFlags()` to clear error counts

**Safety Interface**:
```cpp
bool isUnsafe() {
  return status_.write_errors > 10 || 
         (!status_.card_present && logger_state_ != LoggerState::UNINITIALIZED);
}
```

## ⚠️ CRITICAL: What is NOT Implemented

### BatteryMonitor (NO Safety Implementation)

**Location**: `TeensyV2/modules/battery/battery_monitor.h/cpp`

**❌ IMPORTANT**: The BatteryMonitor does **NOT** implement safety checks despite having comprehensive monitoring capabilities.

**What's Implemented**:
- Voltage and current monitoring via INA226 sensors
- Battery state tracking (CHARGING, DISCHARGING, etc.)
- Status reporting and diagnostics
- Configuration for critical thresholds (in config structs)

**What's NOT Implemented**:
- **No `isUnsafe()` override**: Uses default Module implementation that returns `false`
- **No actual safety enforcement**: Critical voltage/current thresholds are configured but not enforced
- **No emergency stop triggers**: Battery conditions do not trigger safety responses

This means the robot has **NO BATTERY SAFETY PROTECTION** at the embedded level.

### Other Missing Safety Features

Based on code analysis, the following safety features are **NOT implemented**:

1. **Individual Cell Monitoring**: No individual battery cell voltage monitoring
2. **Current Limiting**: Only overcurrent detection, no active current limiting
3. **Temperature Safety**: No temperature-based safety responses
4. **Sensor Communication Timeout Safety**: Sensors may monitor timeouts but don't enforce safety responses
5. **Coordinated Shutdown Procedures**: No systematic safe shutdown sequences

## Safety System Operation Flow

### 1. Module Safety Polling
The SafetyCoordinator periodically checks all modules:

```cpp
bool Module::isAnyModuleUnsafe() {
  for (uint16_t i = 0; i < module_count_; ++i) {
    if (modules_[i] != nullptr && modules_[i]->isUnsafe()) {
      return true;
    }
  }
  return false;
}
```

### 2. Emergency Stop Activation
When any unsafe condition is detected:

```cpp
void SafetyCoordinator::activateEstop(EstopSource source, const String& description) {
  current_state_ = SafetyState::EMERGENCY_STOP;
  estop_condition_.active = true;
  digitalWrite(config_.estop_output_pin, HIGH);  // Hardware E-stop signal
  sendStatusUpdate();  // Notify ROS2 via serial
}
```

### 3. Hardware Integration
- **E-stop Output Pin**: HIGH when emergency stop active (fail-safe design)
- **Inter-board Safety**: Monitors and broadcasts safety signals to/from other boards
- **Serial Communication**: JSON messages to ROS2 via `sigyn_to_sensor_v2`

## Communication with ROS2

### Serial Message Protocol
Safety events are communicated via JSON messages over serial:

**CRITICAL Messages (Immediate Safety Violations)**:
```json
{
  "active": true,
  "source": "MOTOR_RUNAWAY",
  "reason": "Motor 1 runaway detected",
  "value": 150,
  "manual_reset": false,
  "time": 12345
}
```

**Sources Implemented**:
- `MOTOR_RUNAWAY`: Motor continues moving when commanded to stop
- `ROBOCLAW_CURRENT`: Motor overcurrent condition
- Hardware E-stop and inter-board signals

### ROS2 Integration Package
**Package**: `sigyn_to_sensor_v2`
- Receives serial messages from TeensyV2
- Publishes safety status to ROS2 topics
- Enables navigation stack integration for safety responses

## Recovery Mechanisms

### Implemented Recovery
- **RoboClaw Auto-Recovery**: Clears latched overcurrent errors when current is low
- **Safety Flag Reset**: `Module::resetAllSafetyFlags()` clears recoverable errors
- **Communication Reconnection**: Automatic RoboClaw communication recovery

### Manual Recovery
- **E-stop Reset**: Requires explicit recovery attempt via `attemptRecovery()`
- **Module Reset**: Individual modules can implement `resetSafetyFlags()`

```cpp
void SafetyCoordinator::deactivateEstop() {
  current_state_ = SafetyState::NORMAL;
  estop_condition_.active = false;
  digitalWrite(config_.estop_output_pin, LOW);
  Module::resetAllSafetyFlags();  // Clear all module safety flags
  sendStatusUpdate();
}
```

## Configuration

Safety thresholds are configured via static configuration structures:
- **RoboClawConfig**: Motor current limits, timeout values
- **BatteryConfig**: Voltage/current thresholds (configured but not enforced!)  
- **PerformanceConfig**: Timing violation thresholds

Example from RoboClawMonitor:
```cpp
if (motor1_status_.current_valid && 
    abs(motor1_status_.current_amps) > config_.max_current_m1) {
  // Overcurrent detected
}
```

## Summary of Actual Safety Implementation

### ✅ Working Safety Features
1. **SafetyCoordinator**: Central E-stop coordination and module monitoring
2. **RoboClawMonitor**: Motor overcurrent detection, runaway detection, communication monitoring
3. **PerformanceMonitor**: Module execution time violation detection  
4. **SDLogger**: Storage error monitoring
5. **Hardware E-stop**: Physical emergency stop button

### ❌ Missing Critical Safety Features
1. **Battery Safety**: No automated battery protection despite monitoring capabilities
2. **Individual Cell Protection**: No per-cell voltage monitoring
3. **Temperature Safety**: No thermal protection
4. **Current Limiting**: Only detection, no active limiting
5. **Sensor Safety**: Limited sensor communication timeout enforcement

### 🔧 Architecture Gaps
1. **BatteryMonitor** has all the infrastructure for safety but doesn't implement `isUnsafe()`
2. Safety thresholds are configured but not always enforced
3. Limited coordination between safety systems
4. No systematic safe shutdown procedures

This analysis reveals that while the safety system architecture is well-designed, several critical safety features (especially battery protection) are not implemented despite being configured and monitored.