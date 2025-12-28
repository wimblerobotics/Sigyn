# Sigyn Robot Safety System Architecture

## Executive Summary

The Sigyn robot implements a comprehensive multi-layered safety system designed to prevent hardware damage, ensure operational safety, and provide graceful degradation under fault conditions. The system spans from low-level hardware protection on Teensy microcontrollers to high-level ROS2 coordination, creating a robust safety framework suitable for autonomous operation.

This document serves as both an architectural overview for ROS2 developers interested in safety system design and a technical specification for the Sigyn platform.

## 1. System Architecture Overview

### 1.1 Safety System Hierarchy

```
┌─────────────────────────────────────────────────────────────────┐
│                     ROS2 Safety Layer                          │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │ Safety          │  │ Navigation      │  │ Behavior Tree   │  │
│  │ Coordinator     │  │ Safety Monitor  │  │ Safety Checks   │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                               │
┌─────────────────────────────────────────────────────────────────┐
│                TeensyV2 Embedded Safety Layer                   │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │ Safety          │  │ RoboClaw        │  │ Battery         │  │
│  │ Coordinator     │  │ Monitor         │  │ Monitor         │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │ Performance     │  │ Temperature     │  │ Sensor          │  │
│  │ Monitor         │  │ Monitor         │  │ Health Monitor  │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                               │
┌─────────────────────────────────────────────────────────────────┐
│                     Hardware Safety Layer                      │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │ E-Stop Button   │  │ Current Sensors │  │ Voltage         │  │
│  │ (Physical)      │  │ (Hardware)      │  │ Monitoring      │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │ Motor Driver    │  │ Thermal         │  │ Encoder         │  │
│  │ Protection      │  │ Protection      │  │ Fault Detection │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 Design Principles

1. **Defense in Depth**: Multiple layers of protection prevent single points of failure
2. **Fail-Safe Design**: System defaults to safe state when any component fails
3. **Real-Time Response**: Critical safety decisions made within millisecond timeframes
4. **Graceful Degradation**: Non-critical failures allow continued operation with reduced capability
5. **Comprehensive Monitoring**: All critical parameters continuously monitored and logged
6. **Inter-System Coordination**: Safety state synchronized across all system components

## 2. TeensyV2 Embedded Safety Architecture

### 2.1 SafetyCoordinator Module

The `SafetyCoordinator` is the central safety authority on each TeensyV2 board, implementing a singleton pattern with the following responsibilities:

#### Core Features:
- **E-Stop State Management**: Tracks and coordinates emergency stop conditions
- **Multi-Source Safety Monitoring**: Handles safety violations from hardware, software, and inter-board sources
- **Hardware E-Stop Integration**: Direct GPIO control for immediate motor cutoff
- **Recovery Logic**: Automatic and manual recovery mechanisms based on fault type
- **Inter-Board Coordination**: Safety signal propagation between multiple Teensy boards

#### E-Stop Sources:
```cpp
enum class EstopSource {
  HARDWARE_BUTTON,     // Physical E-stop button pressed
  SOFTWARE_COMMAND,    // Software E-stop from ROS2
  PERFORMANCE,         // Performance violation (timing/frequency)
  BATTERY_LOW_VOLTAGE, // Battery voltage critically low
  BATTERY_HIGH_CURRENT,// Battery current critically high
  MOTOR_OVERCURRENT,   // Motor drawing excessive current
  MOTOR_RUNAWAY,       // Motor speed control failure
  SENSOR_FAILURE,      // Critical sensor offline
  INTER_BOARD,         // Safety signal from other board
  UNKNOWN              // Undefined or multiple sources
};
```

#### Safety States:
```cpp
enum class SafetyState {
  NORMAL,              // All systems operational
  WARNING,             // Minor issues detected, monitoring
  DEGRADED,            // Operating with reduced functionality
  EMERGENCY_STOP,      // Emergency stop active
  SYSTEM_SHUTDOWN      // Complete system shutdown required
};
```

### 2.2 RoboClaw Motor Safety System

The `RoboClawMonitor` module provides comprehensive motor controller safety with multiple protection mechanisms:

#### Motor Runaway Detection:
- **Encoder Feedback Monitoring**: Continuously compares commanded vs actual motor speeds
- **Threshold-Based Detection**: Triggers E-stop when encoder changes exceed safe limits
- **Time-Based Analysis**: Monitors sustained speed deviations over configurable periods
- **Automatic Recovery**: Can resume operation once conditions normalize

```cpp
struct RoboClawConfig {
  uint32_t runaway_encoder_threshold = 1000;   // Encoder change threshold
  uint32_t command_timeout_ms = 1000;          // Command timeout for detection
  uint32_t runaway_check_interval_ms = 200;    // Check frequency
};
```

#### Overcurrent Protection:
- **Real-Time Current Monitoring**: Continuously reads motor current from RoboClaw
- **Configurable Thresholds**: Separate limits for warning and emergency conditions
- **Thermal Protection**: Prevents motor winding damage through current limiting
- **Historical Tracking**: Maintains current consumption statistics for analysis

#### Communication Safety:
- **Heartbeat Monitoring**: Detects communication failures with RoboClaw hardware
- **Command Acknowledgment**: Verifies command execution through status feedback
- **Error Status Decoding**: Comprehensive interpretation of RoboClaw error codes
- **Automatic Reconnection**: Attempts to restore communication after failures

### 2.3 Battery Monitoring System

The battery monitoring system protects against power-related failures:

#### Voltage Monitoring:
- **Multi-Cell Monitoring**: Tracks individual cell voltages in battery pack
- **Under-Voltage Protection**: Prevents deep discharge damage to batteries
- **Over-Voltage Protection**: Guards against charging system failures
- **Voltage Trend Analysis**: Predicts battery depletion for proactive shutdowns

#### Current Monitoring:
- **High-Current Detection**: Protects against short circuits and overcurrent conditions
- **Power Consumption Analysis**: Tracks system power usage for optimization
- **Charging Current Monitoring**: Ensures safe battery charging rates
- **Load Balancing**: Monitors current distribution across multiple power rails

### 2.4 Performance Monitoring System

Real-time performance monitoring ensures system timing constraints:

#### Loop Timing Analysis:
- **Execution Time Tracking**: Monitors each module's loop execution time
- **Performance Violation Detection**: Identifies modules exceeding time budgets
- **Statistical Analysis**: Maintains min/max/average execution time statistics
- **Performance Trend Monitoring**: Detects degrading performance over time

#### Module Health Monitoring:
- **Module Registration Tracking**: Ensures all required modules are active
- **Health Status Aggregation**: Combines health status from all system modules
- **Fault Isolation**: Identifies which specific modules are experiencing issues
- **Recovery Coordination**: Manages module restart and recovery procedures

## 3. ROS2 Safety Integration

### 3.1 sigyn_to_sensor_v2 Safety Coordinator

The ROS2 `SafetyCoordinator` node provides system-wide safety coordination:

#### Multi-Board Coordination:
- **Board Safety State Aggregation**: Combines safety status from multiple Teensy boards
- **Global E-Stop Management**: Coordinates emergency stops across entire system
- **Safety Message Routing**: Routes safety commands between boards and ROS2 nodes
- **Timeout Detection**: Identifies boards that have stopped communicating

#### ROS2 Integration:
```cpp
// Publishers
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr global_estop_pub_;
rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr safety_diagnostics_pub_;

// Services
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_estop_service_;
```

#### Safety Services:
- **Emergency Stop Service**: Allows ROS2 nodes to trigger system-wide E-stop
- **E-Stop Reset Service**: Provides controlled recovery from E-stop conditions
- **Safety Status Publishing**: Broadcasts current safety state to all ROS2 nodes
- **Diagnostic Integration**: Publishes detailed safety diagnostics for monitoring

### 3.2 TeensyBridge Safety Integration

The `TeensyBridge` node handles safety communication between Teensy and ROS2:

#### Message Handling:
- **Safety Message Parsing**: Decodes safety status from Teensy serial protocol
- **E-Stop Command Forwarding**: Sends ROS2 E-stop commands to Teensy hardware
- **Status Aggregation**: Combines data from multiple safety sources
- **Real-Time Communication**: Maintains low-latency safety message handling

#### Communication Protocol:
```cpp
void HandleSafetyMessage(const MessageData& data, rclcpp::Time timestamp);
void HandleEstopMessage(const MessageData& data, rclcpp::Time timestamp);
void EstopCommandCallback(const std_msgs::msg::Bool::SharedPtr msg);
```

## 4. Safety Features Implementation Status

### 4.1 Implemented Features

#### ✅ Hardware E-Stop System
- **Physical E-Stop Button**: Immediate hardware-level motor cutoff
- **GPIO Integration**: Direct hardware control through Teensy GPIO pins
- **Inter-Board Propagation**: E-stop signals shared between multiple boards
- **Fail-Safe Design**: Active-high signals ensure safe failure modes

#### ✅ Motor Safety Systems
- **Runaway Detection**: Encoder-based motor runaway detection and prevention
- **Overcurrent Protection**: Real-time current monitoring with configurable thresholds
- **Communication Monitoring**: Heartbeat and command acknowledgment systems
- **Automatic Recovery**: Conditional automatic recovery from transient faults

#### ✅ Battery Protection
- **Voltage Monitoring**: Multi-level voltage monitoring with warning and emergency thresholds
- **Current Limiting**: High-current detection and protection systems
- **Trend Analysis**: Battery depletion prediction for proactive management
- **Multi-Cell Support**: Individual cell monitoring in multi-cell battery packs

#### ✅ Performance Monitoring
- **Real-Time Performance Analysis**: Continuous monitoring of module execution times
- **Violation Detection**: Automatic detection of timing constraint violations
- **Statistical Tracking**: Comprehensive performance statistics collection
- **Trend Analysis**: Long-term performance trend monitoring and analysis

#### ✅ ROS2 Integration
- **System-Wide Coordination**: Global safety state management across all ROS2 nodes
- **Service Interface**: Emergency stop and recovery services for ROS2 integration
- **Diagnostic Publishing**: Comprehensive safety diagnostics for monitoring and debugging
- **Multi-Board Support**: Coordination of safety state across multiple Teensy boards

### 4.2 Safety Communication Protocol

The safety system uses a structured JSON-based communication protocol between Teensy and ROS2:

```json
{
  "type": "SAFETY",
  "module": "SafetyCoordinator",
  "data": {
    "state": 2,
    "estop_active": true,
    "source": "MOTOR_OVERCURRENT",
    "description": "Motor 1 overcurrent: 15.2A > 10.0A threshold"
  }
}
```

#### Message Types:
- **SAFETY**: General safety status updates
- **ESTOP**: Emergency stop events and status
- **BATTERY**: Battery-related safety conditions
- **PERFORMANCE**: Performance violation reports
- **DIAGNOSTIC**: Detailed diagnostic information

## 5. Safety Design Patterns for ROS2

### 5.1 Hierarchical Safety Architecture

The Sigyn safety system demonstrates a hierarchical approach suitable for complex ROS2 systems:

1. **Hardware Layer**: Immediate, non-negotiable protection (< 1ms response)
2. **Embedded Layer**: Real-time safety logic (< 10ms response)
3. **ROS2 Layer**: System coordination and decision making (< 100ms response)
4. **Application Layer**: High-level safety policies and recovery strategies

### 5.2 Safety State Management

**Centralized Coordination**: Single authority for system-wide safety decisions
```cpp
class SafetyCoordinator {
  SafetyState current_state_;
  std::map<uint8_t, BoardSafetyState> board_safety_states_;
  void UpdateGlobalEstopState();
};
```

**Distributed Monitoring**: Local safety decisions with global coordination
```cpp
class Module {
  virtual bool isUnsafe() = 0;
  virtual void resetSafetyFlags() = 0;
  PerformanceStats performance_stats_;
};
```

### 5.3 Real-Time Safety Patterns

**Event-Driven Safety**: Immediate response to critical conditions
```cpp
void SafetyCoordinator::checkHardwareEstop() {
  if (digitalRead(config_.hardware_estop_pin) == LOW) {
    activateEstop(EstopSource::HARDWARE_BUTTON, "Hardware E-stop pressed");
  }
}
```

**Periodic Safety Audits**: Regular system health verification
```cpp
void SafetyCoordinator::loop() {
  uint32_t now = millis();
  if (now - last_check_time_ms_ >= config_.estop_check_interval_ms) {
    checkSafetyStatus();
    last_check_time_ms_ = now;
  }
}
```

### 5.4 Recovery Strategies

**Automatic Recovery**: For transient conditions
```cpp
bool condition_cleared = true;
switch (estop_condition_.source) {
  case EstopSource::PERFORMANCE:
    // Check if performance has improved
    condition_cleared = !Module::isAnyModuleUnsafe();
    break;
  // ... other cases
}
```

**Manual Recovery**: For persistent or safety-critical conditions
```cpp
case EstopSource::HARDWARE_BUTTON:
  // Requires physical button release
  if (digitalRead(config_.hardware_estop_pin) == LOW)
    condition_cleared = false;
  break;
```

## 6. Integration with ROS2 Navigation and Behavior Trees

### 6.1 Navigation Safety Integration

The safety system integrates with ROS2 navigation stack through:

- **Velocity Command Monitoring**: Tracks cmd_vel commands for safety compliance
- **Path Planning Safety**: Validates planned paths against safety constraints  
- **Obstacle Avoidance**: Coordinates with navigation for emergency maneuvers
- **Localization Health**: Monitors localization system health and accuracy

### 6.2 Behavior Tree Safety Integration

Safety checks are integrated into behavior trees for mission-level safety:

```cpp
// Example behavior tree safety nodes
CheckBatteryLevel        // Verify sufficient battery for mission
CheckTemperature         // Ensure operating temperature within limits
CheckDoorState          // Verify doors/barriers are in safe positions
SendThreatAlert         // Notify operators of detected safety threats
```

## 7. Diagnostic and Monitoring Capabilities

### 7.1 Real-Time Diagnostics

The system provides comprehensive real-time diagnostic information:

```cpp
diagnostic_msgs::msg::DiagnosticArray diagnostics_msg;
// Populate with safety status, performance metrics, error counts
safety_diagnostics_pub_->publish(diagnostics_msg);
```

### 7.2 Performance Analytics

Detailed performance monitoring enables system optimization:

```cpp
struct PerformanceStats {
  float duration_min_us = MAXFLOAT;  // Best case performance
  float duration_max_us = 0.0f;      // Worst case performance  
  float duration_sum_us = 0.0f;      // For average calculation
  uint32_t loop_count = 0;           // Total iterations
};
```

### 7.3 Safety Event Logging

All safety events are logged with timestamps and context:

```cpp
void SafetyCoordinator::sendStatusUpdate() {
  char status_msg[128];
  snprintf(status_msg, sizeof(status_msg),
           "{\"state\":%d,\"estop_active\":%d,\"source\":\"%s\"}",
           static_cast<int>(current_state_), estop_condition_.active,
           estop_condition_.description.c_str());
  SerialManager::getInstance().sendMessage("SAFETY", status_msg);
}
```

## 8. Testing and Validation

### 8.1 Safety System Testing

The safety system requires comprehensive testing across multiple layers:

1. **Unit Testing**: Individual safety module functionality
2. **Integration Testing**: Multi-module safety coordination  
3. **Hardware-in-the-Loop**: Real hardware safety response testing
4. **Failure Mode Testing**: Systematic testing of failure scenarios
5. **Performance Testing**: Real-time constraint validation

### 8.2 Compliance and Standards

The system design considers relevant safety standards:

- **IEC 61508**: Functional safety of electrical systems
- **ISO 13849**: Safety of machinery control systems
- **ROS 2 Safety Working Group**: Guidelines for safe ROS2 systems

## 9. Conclusion

The Sigyn robot safety system demonstrates a comprehensive approach to robotics safety that balances real-time performance requirements with robust protection mechanisms. The multi-layered architecture ensures that safety is maintained even under complex failure scenarios, while the modular design allows for easy extension and customization.

Key strengths of the system include:

1. **Multi-layered Defense**: Hardware, embedded, and software safety layers
2. **Real-time Performance**: Microsecond-level response times for critical conditions
3. **Comprehensive Monitoring**: All critical parameters continuously tracked
4. **Graceful Degradation**: System continues operating safely under partial failures
5. **ROS2 Integration**: Seamless integration with standard ROS2 navigation and control

This architecture provides a solid foundation for safe autonomous operation and serves as a reference implementation for other ROS2-based robotic systems requiring comprehensive safety frameworks.

## References and Further Reading

1. TeensyV2 Safety Coordinator Source Code: `TeensyV2/modules/safety/`
2. RoboClaw Motor Safety Implementation: `TeensyV2/modules/roboclaw/`
3. ROS2 Safety Integration: `sigyn_to_sensor_v2/include/sigyn_to_sensor_v2/`
4. IEC 61508 Functional Safety Standard
5. ISO 13849 Safety of Machinery Standard
6. ROS 2 Safety Working Group Documentation