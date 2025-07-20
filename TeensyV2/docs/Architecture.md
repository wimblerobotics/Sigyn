# TeensyV2 System Architecture

## Design Principles

### 1. Real-Time Performance First
- Target: 80-100Hz main control loop
- Maximum module execution time: 2ms per loop()
- Non-blocking I/O operations throughout
- Performance monitoring with automatic safety violations

### 2. Modular Design
- Clean separation of concerns
- Automatic module registration via singleton pattern
- Standardized interfaces for sensors and actuators
- Easy addition of new functionality

### 3. Safety-Critical Operation
- Multiple independent safety monitoring systems
- Fail-safe defaults for all operations
- Comprehensive E-stop coordination
- Automatic recovery when conditions clear

### 4. Efficient Communication
- Optimized message protocol for low latency
- Structured data format for reliable parsing
- Minimal bandwidth usage for real-time performance
- Error detection and recovery mechanisms

## System Components

### Core Framework

#### Module System (`common/core/module.h`)
```cpp
class Module {
public:
  static void setupAll();           // Initialize all modules
  static void loopAll();            // Execute all modules (target 80-100Hz)
  virtual bool isUnsafe();          // Safety violation detection
  virtual void resetSafetyFlags();  // Clear recoverable errors

protected:
  virtual void setup() = 0;         // One-time initialization
  virtual void loop() = 0;          // High-frequency execution (≤2ms)
  virtual const char* name() = 0;   // Module identification

private:
  // Performance tracking and statistics
  // Automatic module registration
};
```

#### Serial Communication (`common/core/serial_manager.h`)
```cpp
class SerialManager {
public:
  static SerialManager& Singleton();
  
  // Single, efficient message interface
  void SendMessage(const char* message_type, const char* data);
  
  // Message parsing for incoming commands
  void ProcessIncomingMessage(const String& message);
  
private:
  // Non-blocking serial I/O
  // Message queuing for high throughput
  // Error detection and recovery
};
```

#### Performance Monitoring (`modules/performance/performance_monitor.h`)
```cpp
class PerformanceMonitor : public Module {
public:
  // Safety violation when timing constraints violated
  bool isUnsafe() override;
  
  // Configure thresholds via runtime parameters
  void UpdateConfiguration(const PerformanceConfig& config);
  
private:
  // Loop frequency tracking
  // Per-module timing analysis  
  // Violation detection and reporting
  
  struct PerformanceConfig {
    float min_loop_frequency_hz = 80.0f;
    uint8_t max_violation_count = 5;
    float max_module_time_ms = 2.0f;
  };
};
```

### Safety System

#### Safety Coordinator (`common/safety/safety_coordinator.h`)
The Safety Coordinator runs on Board 1 and aggregates safety conditions from all sources:

```cpp
class SafetyCoordinator : public Module {
public:
  // E-stop trigger sources
  enum class EStopSource {
    kMotorOvercurrent,    // Detected by motor module
    kMotorRunaway,        // Detected by motor module  
    kBatteryUndervoltage, // Received from Board 2
    kOvertemperature,     // Received from Board 2 or local
    kPerformanceViolation,// Detected by performance monitor
    kExternalCommand,     // Received from ROS2
    kCommunicationLoss,   // Detected locally
  };

  // E-stop state management
  void TriggerEStop(EStopSource source, const char* reason);
  void ClearEStop(EStopSource source);
  bool IsEStopped() const;
  
  // Auto-recovery for conditions that clear themselves
  bool IsAutoRecoverable(EStopSource source) const;

private:
  // Track individual E-stop sources
  // Send aggregated status to ROS2
  // Control hardware E-stop relays
};
```

#### Message Protocol

All messages use a compact key-value format for efficient parsing:

**Outgoing Messages (Teensy → ROS2):**
```
BATT:id=0,v=39.8,p=0.82,c=1.2,alarms=OV,state=OK
IMU:id=0,qx=0.1,qy=0.2,qz=0.3,qw=0.9,gx=0.0,gy=0.0,gz=0.0
ODOM:px=1.2,py=0.5,theta=0.3,vx=0.1,vy=0.0,omega=0.0
ESTOP:active=true,sources=motor:overcurrent+battery:undervoltage
PERF:freq=95.2,violations=0,max_time=1.8
```

**Incoming Messages (ROS2 → Teensy):**
```
CMD_VEL:vx=0.5,omega=0.2
ESTOP:trigger=true,source=external,reason=user_command
CONFIG:param=min_loop_freq,value=85.0
PARAM_REQ:param=all                    // Request all parameter values
```

### Board-Specific Implementations

#### Board 1: Motor Control & Safety Coordination
**Primary Responsibilities:**
- RoboClaw motor control with safety monitoring
- Wheel odometry calculation and reporting
- VL53L0X distance sensor array
- E-stop coordination (receives from Board 2, triggers hardware)
- SONAR sensor management
- Performance monitoring for entire system

**Critical Timing:**
- Motor commands: 50Hz minimum (every 20ms)
- Safety monitoring: 100Hz (every 10ms)
- Sensor readings: Variable by sensor type

#### Board 2: Sensor Monitoring
**Primary Responsibilities:**
- Battery voltage/current monitoring (36V LIPO + power supplies)
- Dual BNO055 IMU sensors
- Temperature monitoring (motor, ambient)
- Safety condition detection and reporting to Board 1

**Data Flow:**
- High-frequency sensor data (IMU): 100Hz
- Battery monitoring: 10Hz
- Temperature monitoring: 1Hz
- Safety alerts: Immediate (interrupt-driven)

### Configuration Management

#### Compile-Time Configuration (`config.h`)
```cpp
// Performance Requirements
#define MIN_LOOP_FREQUENCY_HZ 80.0f
#define MAX_MODULE_TIME_MS 2.0f
#define MAX_VIOLATION_COUNT 5

// Hardware Configuration (cannot be changed at runtime)
#define VL53L0X_SENSOR_COUNT 8
#define BNO055_SENSOR_COUNT 2
#define BATTERY_MONITOR_COUNT 5

// Safety Thresholds (defaults, can be updated via ROS2)
#define BATTERY_CRITICAL_VOLTAGE 33.0f
#define MOTOR_MAX_CURRENT_A 15.0f
#define MAX_MOTOR_TEMP_C 80.0f
```

#### Runtime Configuration
- ROS2 parameter interface for safety thresholds
- EEPROM storage for persistent configuration
- Configuration validation before applying changes
- Rollback mechanism for invalid configurations

### Inter-Board Communication

#### Board 1 ← Board 2 (Safety Critical)
```
SAFETY:battery_critical=true,temp_high=false,imu_fault=false
BATT:id=0,unsafe=true,reason=undervoltage
TEMP:motor1=75.2,motor2=78.1,ambient=25.3
```

#### Board 1 → Board 2 (Status Updates)
```
ESTOP_STATUS:active=true,source=motor
TIME_SYNC:ros_time_ms=1234567890,offset_ms=10
CONFIG_UPDATE:param=battery_critical_v,value=32.5
```

### Error Handling & Recovery

#### Communication Failures
- Watchdog timers for inter-board communication
- Automatic E-stop when communication lost >500ms
- Graceful degradation when possible
- Diagnostic reporting for troubleshooting

#### Sensor Failures
- Individual sensor health monitoring
- Graceful degradation (e.g., continue with fewer VL53L0X sensors)
- Error reporting to ROS2 for operator awareness
- Automatic recovery when sensors come back online

#### Performance Violations
- Immediate detection when loop frequency drops
- Graduated response (warning → E-stop)
- Detailed timing reports for debugging
- Automatic recovery when performance improves

## Development Guidelines

### Module Development
1. Inherit from `Module` base class
2. Implement non-blocking `loop()` method (≤2ms execution)
3. Use state machines for complex operations
4. Implement proper safety monitoring in `IsUnsafe()`
5. Provide meaningful diagnostic messages

### Safety Considerations
1. All operations must be fail-safe by default
2. Implement proper bounds checking
3. Validate all external inputs
4. Design for graceful degradation
5. Test recovery scenarios thoroughly

### Performance Optimization
1. Minimize memory allocations in `loop()`
2. Use efficient data structures
3. Cache calculations when possible
4. Profile timing regularly
5. Optimize critical paths first

This architecture provides a robust foundation for real-time embedded control while maintaining flexibility for future expansion and modification.
