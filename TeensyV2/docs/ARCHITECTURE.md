# TeensyV2 System Architecture

**Table of Contents**
- [Overview](#overview)
- [Design Principles](#design-principles)
- [System Components](#system-components)
  - [Core Framework](#core-framework)
  - [Module System](#module-system)
  - [Communication](#communication)
  - [Safety System](#safety-system)
  - [Hardware Control](#hardware-control)
- [Board Configuration](#board-configuration)
- [Integration with ROS2](#integration-with-ros2)
- [Message Flow](#message-flow)

## Overview

TeensyV2 is a real-time embedded control system for the Sigyn house patroller robot, running on Teensy 4.1 microcontrollers. The system manages:

- **Motor control & odometry** via RoboClaw motor drivers
- **Distance sensing** using 8× VL53L0X time-of-flight sensors
- **Safety coordination** across multiple boards via hardware and software communication
- **Thermal & power monitoring** for system health
- **Data logging** to SD card for forensic analysis and performance statistics
- **High-speed USB communication** with ROS2 running on a host PC

### Key Design Characteristics

- **Real-time**: Main loop enforces board-specific minimums (Board 1: 50 Hz, Board 2: 20 Hz, Board 3: 10 Hz) and runs faster when work is light
- **No heap allocations**: Fixed-size buffers and static data structures for embedded safety
- **Modular**: Hardware devices and services registered via a unified Module system
- **Redundant safety**: Multiple independent safety monitors with inter-board communication
- **Efficient**: Compact JSON messages over USB eliminate MicroROS complexity

---

## Design Principles

### 1. Real-Time Performance First

The system prioritizes meeting strict timing requirements to ensure responsive motor control and safe operation:

- **Minimum loop frequency thresholds**: Board 1 → 50 Hz, Board 2 → 20 Hz, Board 3 → 10 Hz (from `config.h`)
- **Performance violations**: Tracked when loop frequency drops below threshold
- **Per-module timing budgets**:
  - Critical modules (RoboClawMonitor): ≤ 0.8–1.4 ms per iteration (observed average from logs)
  - Standard modules: ≤ 2.0 ms per iteration
  - Non-blocking I/O throughout to prevent stack-up

### 2. Modular Design

Every hardware device or service is a **Module**:

- Automatic registration via instantiation
- Unified setup/loop lifecycle
- Standardized isUnsafe() contract for safety integration
- Easy addition of new sensors or actuators without core changes

See [Module System](#module-system) below.

### 3. Safety-Critical Operation

Safety is **not** an afterthought—it is embedded in the architecture:

- Multiple independent safety monitoring systems (motors, temperature, battery, communication)
- Fail-safe defaults (motors stop when safety threshold violated)
- E-stop sources tracked individually; auto-recovery when conditions clear
- Inter-board safety communication via GPIO (in development) and message passing
- Comprehensive fault logging for forensic analysis

See [Safety System](#safety-system) below.

### 4. Efficient Communication

USB is used instead of MicroROS:

- **Why**: Simpler, faster, lower latency, easier to debug and customize for Teensy
- **Protocol**: Compact JSON messages with short keys (e.g., `"lf"` for `lidar_frame`)
- **Timestamps**: Generated on PC side (sigyn_to_sensor_v2) for ROS2 message timing
- **No flow control overhead**: USB handshake is hardware-managed

---

## System Components

### Core Framework

#### Module System

The **Module** base class provides a unified registration and lifecycle interface:

```cpp
class Module {
public:
  // Initialization: called once at startup
  static void setupAll();
  
  // Main loop: runs as fast as work allows; performance monitor enforces per-board minimums
  static void loopAll();
  
  // Safety: check if module has a fault
  virtual bool isUnsafe();
  
  // Recovery: clear recoverable faults
  virtual void resetSafetyFlags();

protected:
  // Must be implemented by subclasses
  virtual void setup() = 0;
  virtual void loop() = 0;
  virtual const char* name() = 0;
};
```

**How it works**:
1. Each module (RoboClawMonitor, VL53L0XMonitor, etc.) inherits from Module
2. When instantiated, the module self-registers in a global registry
3. `Module::setupAll()` calls `setup()` on all registered modules
4. `Module::loopAll()` calls `loop()` on all registered modules and checks `isUnsafe()`

**Example**: RoboClawMonitor initialization:
```cpp
// In board1_main.cpp
if (BOARD_HAS_MOTOR_CONTROL) {
  roboclaw_monitor = new RoboClawMonitor();
}

// In setup():
Module::setupAll();  // Calls RoboClawMonitor::setup()

// In loop():
Module::loopAll();   // Calls RoboClawMonitor::loop() and checks isUnsafe()
```

#### SerialManager

**Purpose**: Unified interface for USB communication with ROS2 host

- **Single responsibility**: Send/receive structured messages via USB
- **Non-blocking**: No waiting for serial I/O in the hot loop
- **Message parsing**: Handles incoming commands and dispatches to modules
- **Queue management**: Buffers prevent message loss under burst conditions

**Key methods**:
```cpp
void sendMessage(const char* message_type, const char* data);
void processIncomingMessage(const String& message);
```

**Example message**:
```json
IMU2:{"id":0,"qx":-0.8290,"qy":-0.5583,"qz":0.0005,"qw":0.0334,"gx":-0.00327,"gy":0.00109,"gz":0.00000,"ax":-0.04000,"ay":0.00000,"az":0.08000,"calib":48,"status":5,"error":0,"timestamp":14521269}
```

#### PerformanceMonitor

**Purpose**: Detect and report timing violations

- **Measures**: Loop frequency, per-module execution time
- **Tracks**: Min/max/average time and violation count per module
- **Reports**: Module performance statistics and violations
- **Integrates with Safety**: If loop frequency drops consistently, can trigger E-stop

**Key metrics** (from actual logs):
```
Loop frequency: 297.6 Hz (Board 1 minimum threshold 50 Hz; running faster due to light workload)
RoboClawMonitor: min 0.80ms, max 4.77ms, avg 1.37ms
VL53L0XMonitor: min 2.08ms, max 3.77ms, avg 2.44ms
TemperatureMonitor: min 0.00ms, max 1.94ms, avg 0.08ms
Performance violations: 1543 (likely due to burst sensor reads)
```

---

### Module System

Each hardware device or service is a Module. Key modules include:

#### RoboClawMonitor

**Hardware**: RoboClaw motor driver (USB interface)

**Responsibilities**:
- Communicate with RoboClaw via serial over UART
- Execute motor speed commands from ROS2
- Read encoder data for odometry calculation
- Detect motor faults (overcurrent, runaway)
- Report RoboClaw device state

**Architecture**: Uses dual state machines to avoid blocking

```cpp
class RoboClawMonitor : public Module {
public:
  void setMotorSpeeds(int32_t m1_qpps, int32_t m2_qpps);
  
private:
  // Connection state machine
  enum class ConnectionState { 
    DISCONNECTED, CONNECTING, CONNECTED, ERROR_RECOVERY, FAILED 
  };
  ConnectionState connection_state_;
  
  // Reading state machine - cycles through sensor reads
  enum class ReadingState {
    READ_ENCODER_M1, READ_SPEED_M1, READ_ENCODER_M2, READ_SPEED_M2,
    READ_CURRENTS, READ_VOLTAGES, READ_ERROR_STATUS, COMPLETE
  };
  ReadingState reading_state_;
  
  // Each loop() call advances one of these states, taking only ~1-2ms
  void loop() override;
};
```

**Faults detected**:
- Motor overcurrent (> 20A limit per motor, configurable default)
- Motor runaway (wheel encoders not consistent with expectation)
- Serial communication failure
- RoboClaw error flags

#### VL53L0XMonitor

**Hardware**: 8× VL53L0X time-of-flight distance sensors

**Responsibilities**:
- Multiplex sensor reads (only one sensor per loop iteration)
- Report distances, confidence, and obstacle detection
- Detect sensor degradation
- Trigger safety if obstacle too close

**Message format** (from actual logs):
```json
{
  "total_sensors": 8,
  "active_sensors": 8,
  "min_distance": 165,
  "max_distance": 1735,
  "distances": [
    {"id": 0, "mm": 265, "raw": 271, "age_us": 2908, "degraded": false},
    ...
  ]
}
```

#### SDLogger

**Hardware**: MicroSD card slot

**Responsibilities**:
- Log all diagnostic messages
- Log performance statistics
- Log fault events
- Provide statistics (buffer usage, write rate)

**Why it matters**: When something fails, remove the card and analyze the detailed logs locally.

**Statistics** (from actual logs):
```json
{
  "buffer_usage": "0 bytes (0%)",
  "write_rate": "10565.00 B/s",
  "total_writes": 239
}
```

#### TemperatureMonitor

**Hardware**: 2× NTC thermistors (left and right motor temperatures)

**Responsibilities**:
- Read analog temperature sensors
- Track min/max temperatures
- Detect overtemperature condition

**Message format** (from actual logs):
```json
{
  "total_sensors": 8,
  "active_sensors": 2,
  "temperatures": [25.4,24.9,null,null,null,null,null,null],
  "avg_temp": 25.1,
  "max_temp": 25.4,
  "min_temp": 24.9,
  "hottest_sensor": 0,
  "system_warning": false,
  "system_critical": false,
  "rate_hz": 19.9,
  "readings": 120,
  "errors": 0
}

```

#### BatteryMonitor

**Hardware**: Battery and power supply voltage and current via INA226 sensors

**Responsibilities**:
- Report battery and power supply voltage and current draw
- Detect out of range power conditions
- Integrate power consumption data

---

### Communication

#### USB + Virtual Serial Port

- **Transport**: USB 2.0 high-speed (Teensy acts as virtual serial port on PC)
- **Format**: Newline-delimited JSON
- **Timestamp source**: PC side (sigyn_to_sensor_v2) applies ROS2 timestamps
- **Baudrate**: Virtual (USB), fast handshake

#### Message Categories

**1. Sensor Data** (published by Teensy, consumed by ROS2):
- VL53L0X: Distance sensors → costmap, safety
- ROBOCLAW: Motor status, currents, encoders → odometry
- TEMPERATURE: Motor temps → thermal warnings
- BATTERY: Voltage/current → low-battery detection
- IMU: Accelerometers, gyroscopes.

**2. Commands** (sent by ROS2, processed by Teensy):
- Motor speed setpoints: **cmd_vel** message
- E-stop trigger: `ESTOP:reason`

**3. Diagnostics** (published for logging/monitoring):
- Fault events: `FAULT:module:reason`
- Performance reports: Loop frequency, timing violations
- Safety events: E-stop triggers, source tracking

#### ROS2 Integration via sigyn_to_sensor_v2

The PC-side **sigyn_to_sensor_v2** node:
- Reads USB serial messages from Teensy
- Converts to ROS2 messages with proper frame_ids and timestamps
- Publishes (partial list):
  - `/sigyn/teensy_bridge/range/*` (VL53L0X)
  - `/sigyn/teensy_bridge/diagnostics` (various)
  - `/sigyn/teensy_bridge/battery/status` (Battery, power supplies)
  - `/sigyn/teensy_bridge/safety/*` (Safety events)
- Subscribes to (partial list)
  - `/cmd_vel` (for  motor control)

**Timestamp handling**: Timestamps are generated on PC side (not Teensy) because:
- ROS2 clock on PC is authoritative
- Avoids clock sync complexity
- Acceptable for non-safety-critical perception
- Simplifies Teensy code

---

### Safety System

#### SafetyCoordinator

- **Role**: Central fault aggregator; other modules call `activateFault()` / `deactivateFault()`.
- **Scope**: Present on any board, not hard-coded to Board 1.
- **Fault tracking**: Up to 16 faults, keyed by source string, each with a severity.
- **Severities** (from `FaultSeverity`): `NORMAL`, `WARNING`, `DEGRADED`, `EMERGENCY_STOP`, `SYSTEM_SHUTDOWN`.
- **E-stop behavior**: When a fault is raised with `EMERGENCY_STOP` or `SYSTEM_SHUTDOWN`, `active_estop_count_` increments and the RoboClaw E-stop line is asserted if `CONTROLS_ROBOCLAW_ESTOP_PIN` is compiled in. Clearing the last E-stop fault releases the line.
- **Reporting**: Once per second, publishes `FAULT` messages over `SerialManager` with active fault data. No inter-board serial/GPIO signaling is implemented yet.
- **Commands**: `setEstopCommand("trigger=true")` raises an E-stop fault; `setEstopCommand("reset=true")` calls `Module::resetAllSafetyFlags()` to clear all faults.
- **Recovery**: SafetyCoordinator itself does not auto-clear; faults clear only when callers invoke `deactivateFault()` or a global reset is requested.

---

### Hardware Control

#### Motor Control via RoboClaw

RoboClawMonitor executes motor commands via a state machine to avoid blocking and to meet performance needs:

1. **Receive command**: ***cmd_vel*** message
2. **Parse**: Extract linear and rotational velocities
3. **Convert**: RPM → QPPS (quadrature pulses per second) using motor config
4. **Execute**: Issue SetMotor command to RoboClaw
5. **Verify**: Read back motor status to confirm

Each step is non-blocking; the state machine progresses over multiple loop() calls.

#### Sensor Multiplexing

VL53L0X has only one I2C data line, but 8 sensors:

- **Multiplexer**: TCA9548A I2C switch
- **Scheduling**: Each loop() iteration reads one sensor
- **Cycle time**: 8 loop iterations = ~94 ms to read all sensors
- **Age tracking**: Each measurement is timestamped; consumer can see age_us

---

## Board Configuration

### Compile-Time Constants

File: `common/core/config.h`

Controls which modules are enabled per board:

```cpp
#define BOARD_ID 1                    // Board 1, 2, or 3
#define ENABLE_PERFORMANCE true       // Performance monitoring
#define ENABLE_SAFETY true            // Safety coordination
#define ENABLE_VL53L0X true           // Distance sensors
#define ENABLE_TEMPERATURE true       // Temperature monitoring
#define ENABLE_BATTERY true           // Battery monitoring
#define ENABLE_SD_LOGGING true        // SD card logging
#define BOARD_HAS_MOTOR_CONTROL true  // Motor control (Board 1 only)
```

### Board 1

- **Motor control**: RoboClawMonitor (RoboClaw motor drivers)
- **Distance sensors**: VL53L0XMonitor (8× VL53L0X)
- **Temperature**: TemperatureMonitor (left & right motor temps)
- **Performance**: Standard performance monitoring
- **Safety**: Primary safety monitoring and control
- **Logging**: SDLogger (logs to card)

### Board 2

- **Battery**: BatteryMonitor (INA226 x5 monitoring 36V, 24V, 12V, 5V, 3.3V)
- **IMU**: BNO055Monitor (2x BNO055)
- **Performance**: Standard performance monitoring
- **Safety**: Secondary safety monitoring
- **Logging**: SDLogger (logs to card)

### Board 3

- **Motors**: StepperMotor modules (Elevator, Extender)
- **Functions**: Gripper control via servos/steppers
- **Performance**: Standard performance monitoring
- **Safety**: Secondary safety monitoring
- **Logging**: SDLogger (logs to card)

---

## Integration with ROS2

### Message Flow

```
ROS2 (nav2, behavior tree, etc.)
  ↓
sigyn_to_sensor_v2 (on PC)
  ↓ (USB serial)
TeensyV2 (board1_main.cpp)
  ↓ (Module::loopAll())
  ├─ RoboClawMonitor → Motor commands, Odometry
  ├─ VL53L0XMonitor → Distance, Obstacle detection
  ├─ TemperatureMonitor → Thermal data
  ├─ SafetyCoordinator → E-stop logic
  └─ SDLogger → Persistent logging
  ↓ (USB serial)
sigyn_to_sensor_v2 (on PC)
  ↓
ROS2 Topics & Services
```

### Recommended Topics

**Subscribed by ROS2**:
- `/scan` (from ldlidar and other sensors)
- `/tf` (robot pose from odometry)
- `/odom` (wheel odometry)

**Published by sigyn_to_sensor_v2**:
- `/sigyn/teensy_bridge/range/*` (VL53L0X distances)
- `/sigyn/teensy_bridge/motor/status` (motor currents, speeds)
- `/sigyn/teensy_bridge/battery/status` (voltage, current)
- `/sigyn/teensy_bridge/safety/events` (fault events)

### Fault Integration with Behavior Trees

ROS2 behavior trees can monitor:

- **Sensor data**: Low battery → trigger charging behavior
- **Fault events**: Obstacle detected close → retreat behavior
- **Safety status**: E-stop active → safe hold state

---

## Performance Characteristics

**Actual measurements from logs for board1**:

| Metric | Value | Notes |
|--------|-------|-------|
| Loop frequency | 297.6 Hz | Configured minimum 50 Hz (Board 1); running faster due to fast I2C reads |
| RoboClawMonitor time | 0.80–4.77 ms (avg 1.37 ms) | Motor driver communication |
| VL53L0XMonitor time | 2.08–3.77 ms (avg 2.44 ms) | Sensor I2C multiplexing |
| TemperatureMonitor time | 0.00–1.94 ms (avg 0.08 ms) | Analog ADC reads |
| SDLogger write rate | 10565 B/s | To MicroSD card |
| Message sample rate | VL53L0X ~1 Hz, ROBOCLAW ~1–2 Hz | USB serial output |

---

## Testing & Dependency Injection

The system supports unit testing via interface-based dependency injection:

- Core modules define abstract interfaces (e.g., `MotorDriverInterface`)
- Implementations can be swapped (real RoboClaw vs. mock for testing)
- Test suite runs with compile-time enabled/disabled modules

See [Testing.md](./Testing.md) for details.

---

## Related Documentation

- [Message_Formats.md](./Message_Formats.md) — Detailed protocol & examples
- [Safety_System.md](./Safety_System.md) — In-depth safety design
- [Module_Reference.md](./Module_Reference.md) — Per-module fault codes & parameters
- [PLATFORMIO_SETUP.md](../PLATFORMIO_SETUP.md) — Build & upload instructions
- [Testing.md](./Testing.md) — Unit test framework & examples
