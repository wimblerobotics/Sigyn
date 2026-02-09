# TeensyV2 Module Reference

**Table of Contents**
- [Introduction](#introduction)
- [RoboClawMonitor](#robobclawmonitor)
- [StepperMotor](#steppermotor)
- [VL53L0XMonitor](#vl53l0xmonitor)
- [TemperatureMonitor](#temperaturemonitor)
- [BatteryMonitor](#batterymonitor)
- [PerformanceMonitor](#performancemonitor)
- [SDLogger](#sdlogger)
- [SafetyCoordinator](#safetycoordinator)

---

## Introduction

This document describes each module's **purpose, configuration, fault codes, and typical behavior**. Modules are designed to be independent; each can be compiled in or out via `config.h` flags.

**Note**: This document describes the current state of the codebase. Module implementations may evolve; verify against source code when making critical decisions.

---

## RoboClawMonitor

### Purpose
Interface to RoboClaw 2x15a v4.3.6 motor driver via USB serial. Reads odometry, current, voltage, and applies motor commands.

### Location
- Header: `modules/roboclaw/roboclaw_monitor.h`
- Implementation: `modules/roboclaw/roboclaw_monitor.cpp`

### Compilation
```cpp
#define BOARD_HAS_MOTOR_CONTROL 1  // In config.h to enable
```

### Loop Frequency
- Encoders + odometry: ~67 Hz (15 ms cadence)
- Safety checks: 10 Hz (100 ms cadence)
- System status: ~3 Hz (333 ms cadence)
- Diagnostics: ~1 Hz
- Command rate limiting: ≥15 ms between motor commands; force resend every 100 ms

### Key Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| USB Port | `Serial7` on Teensy (virtual USB to host) | See `roboclaw_monitor.cpp` |
| Baud Rate | 230400 | `RoboClawConfig::baud_rate` default |
| Motor Overcurrent Threshold | 100.0 A (default, configurable) | `max_current_m1/m2` in `RoboClawConfig` |
| Overcurrent Recovery | Manual reset | Overcurrent is latched; host must clear via reset command |
| Query Timeout | 100 ms | `timeout_us = 100000` in `RoboClawConfig` |
| Max speed | 10,000 QPPS | `max_speed_qpps` default |

### Motor Speed Encoding

**From ROS2 to RoboClaw**:

1. ROS2 sends: `TWIST:linear_x:0.5,angular_z:1.0`
2. RoboClawMonitor parses: `vx = 0.5 m/s`, `wz = 1.0 rad/s`
3. Converts Velocity → QPPS (Quadrature Pulses Per Second):
   ```cpp
   // Internally:
   // 1. Convert Twist -> Left/Right RPM (Differential Drive Kinematics)
   // 2. Convert RPM -> QPPS using encoder resolution and gear ratio
   ```
4. Sends to RoboClaw: `Drive M1 at calculated QPPS, M2 at calculated QPPS`
5. RoboClaw's PID controller servo to target speed

**Encoder Feedback**:
- RoboClaw continuously reads quadrature encoders
- Returns cumulative encoder count + current speed (measured, not commanded) in QPPS
- RoboClawMonitor publishes `ODOM1` messages with calculated pose and velocity

### Fault Codes

**Motor Overcurrent (latched)**:
- **Threshold**: > 100 A on either motor (configurable)
- **Recovery**: Manual reset required (host must call reset); watchdog does not auto-clear
- **Log**: `CRITICAL … Motor X overcurrent` (from `checkSafetyConditions`)

**Motor Runaway Detection**:
- **Threshold**: Speed > 5000 QPPS (configurable; ~0.5 m/s) while commanded speed is 0.
- **Condition**: Firmware detects uncommanded movement when robot should be stationary.
- **Action**: Triggers immediate E-stop to prevent uncontrolled motion.
- **Log**: `CRITICAL … Motor X runaway detected` (from `detectMotorRunaway`)

**RoboClaw Communication Failure**:
- **Threshold**: serial calls exceed 100 ms timeout and repeated failures
- **Recovery**: Automatic when communication resumes; after 3 consecutive failures an E-stop fault is raised
- **Log**: `Communication error detected` (diagnostic) and fault raised via SafetyCoordinator

**RoboClaw Internal Errors**:
The firmware actively monitors the RoboClaw's internal error register (via `ReadError`). If any of the following bits are set, a fatal E-stop is triggered via SafetyCoordinator:
- **0x0001**: `E_STOP` (RoboClaw E-stop active)
- **0x0002**: `TEMPERATURE_ERROR` (Over temperature)
- **0x0004**: `TEMPERATURE2_ERROR`
- **0x0008**: `MAIN_BATTERY_HIGH_ERROR` (Over voltage > 34V)
- **0x0010**: `LOGIC_VOLTAGE_HIGH_ERROR` (> 5.5V)
- **0x0020**: `LOGIC_VOLTAGE_LOW_ERROR` (< 4.5V)
- **0x0040**: `M1_DRIVER_FAULT_ERROR` (MOSFET failure/Driver fault)
- **0x0080**: `M2_DRIVER_FAULT_ERROR`
- **0x0100**: `M1_SPEED_ERROR` (Cannot maintain speed)
- **0x0200**: `M2_SPEED_ERROR`
- **0x0400**: `M1_POSITION_ERROR` (Position deviation too high)
- **0x0800**: `M2_POSITION_ERROR`
- **0x1000**: `M1_CURRENT_ERROR` (Hardware overcurrent)
- **0x2000**: `M2_CURRENT_ERROR`

**Warnings (Non-Fatal)**:
- `M1/M2_OVERCURRENT_WARNING`
- `MAIN_VOLTAGE_HIGH/LOW_WARNING`
- `TEMPERATURE_WARNING`
- `S4/S5_SIGNAL_TRIGGERED`

### Actual Logs

**Idle state**:
```json
ROBOCLAW1:{"LogicVoltage":0.0,"MainVoltage":24.1,"Encoder_Left":0,"Encoder_Right":0,"LeftMotorCurrent":0.000,"RightMotorCurrent":0.000,"LeftMotorSpeed":0,"RightMotorSpeed":0,"Error":"0","ErrorDecoded":"No errors"}
```

**Moving forward (50 RPM)**:
```json
ROBOCLAW1:{"LogicVoltage":0.0,"MainVoltage":24.1,"Encoder_Left":102,"Encoder_Right":98,"LeftMotorCurrent":1.200,"RightMotorCurrent":1.150,"LeftMotorSpeed":50,"RightMotorSpeed":50,"Error":"0","ErrorDecoded":"No errors"}
```

**Obstacle impact**:
```json
ROBOCLAW1:{"LeftMotorCurrent":6.200,"RightMotorCurrent":5.800,"Error":"0"}
DIAG1:{"level":"ERROR","module":"RoboClawMonitor","message":"OVERCURRENT: left=6.2A, right=5.8A"}
```

---

## StepperMotor

### Purpose
Controls two stepper motors (elevator and extender) on Board 3 for gripper positioning. Provides position-based control alongside the existing velocity-based TWIST commands. Manages homing sequences and position feedback.

### Location
- Header: `modules/motors/stepper_motor.h`
- Implementation: `modules/motors/stepper_motor.cpp`
- Dependencies: `Motor` class (hardware abstraction)

### Compilation
```cpp
#define BOARD_ID 3  // Board 3 (elevator/gripper)
// StepperMotor module is always enabled on Board 3
```

### Loop Frequency
- Position updates: Every loop iteration (~100+ Hz)
- Status messages: **10 Hz auto-publishing** (100ms intervals)
- Command processing: Non-blocking state machine
- Homing: Non-blocking with continuous status updates

### Key Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Elevator Max Travel | 0.8999 m | Hardware-measured maximum (hardcoded) |
| Extender Max Travel | 0.3418 m | Hardware-measured maximum (hardcoded) |
| Home Position | 0.0 m (both) | Lower limit for elevator, retracted for extender |
| Position Units | meters (m) | Absolute position from home |
| Homing Sequence | Extender first, then elevator | Safety: retract before lowering |
| Status Publishing Rate | 10 Hz (100ms) | Automatic, continuous updates |
| Position Self-Correction | Enabled | Auto-corrects at limit switches |

### Command Types

**Position Control (`STEPPOS`)**:
- Format: `STEPPOS:elevator:0.5,extender:0.2`
- Moves motors to absolute positions in meters
- Positions clamped to hardware limits (0.0 to max_travel)
- Non-blocking execution with state machine
- Returns `STEPPERSTAT3` on completion

**Homing (`STEPHOME`)**:
- Format: `STEPHOME:`
- Initiates **non-blocking** two-stage homing sequence:
  1. Retract extender to lower limit (0.0m)
  2. Lower elevator to lower limit (0.0m)
- Uses hardware limit switches for home detection
- Establishes zero reference for position tracking
- **Non-blocking**: Status continues to publish at 10 Hz during homing
- **Safety**: Retracts gripper before lowering to avoid collisions

**Status Query (`STEPSTATUS`)**:
- Format: `STEPSTATUS:`
- Immediately returns `STEPPERSTAT3` message
- **Note**: Status is auto-published at 10 Hz; explicit queries rarely needed
- Use for synchronization or immediate position verification

### Velocity Control (Legacy)

The TWIST command interface is also supported for time-based movements:
- Format: `TWIST:linear_x:0.001,angular_z:0.001`
- `linear_x`: Elevator velocity (0.001m increments per command)
- `angular_z`: Extender velocity (0.001m increments per command)
- Legacy interface maintained for compatibility
- Position control (STEPPOS) recommended for new implementations

### Status Message Format
Auto-published at 10 Hz (every 100ms)

```json
STEPPERSTAT3:{
  "elev_pos": 0.4500,
  "ext_pos": 0.1200,
  "elev_lim": "none",
  "ext_lim": "none",
  "elev_max": 0.8999,
  "ext_max": 0.3418
}
```

**Publishing Behavior**:
- Automatic transmission at 10 Hz during all operations
- No manual query required for continuous monitoring
- Continues during homing, movement, and idle states
```

**Fields**:
- `elev_pos` (float, m): Current elevator position
- `ext_pos` (float, m): Current extender position
- `elev_lim` (string): Limit switch state: `"none"`, `"upper"`, `"lower"`, `"both"`
- `ext_lim` (string): Limit switch state: `"none"`, `"upper"`, `"lower"`, `"both"`
- `elev_max` (float, m): Maximum elevator travel (0.8999m)
- `ext_max` (float, m): Maximum extender travel (0.3418m)

### Hardware Limits

**Measured Travel Ranges** (determined via manual testing):
- **Elevator**: 0.0m (lower limit) to 0.8999m (upper limit)
- **Position auto-correction**: When limit switch detected, position immediately corrected to limit value
- **Extender**: 0.0m (retracted) to 0.3418m (fully extended)

**Limit Switch Protection**:
- Hardware limit switches at both ends of travel
- Firmware checks limit states before allowing movement
- Position commands clamped to measured max values
- Emergency stop on unexpected limit activation during motion

### Safety Features

1. **Hardware Limits**: Physical limit switches prevent overtravel
2. **Software Clamping**: Target positions validated against measured maxima
3. **Homing Sequence**: Retracts exte10 Hz status publishing via STEPPERSTAT3
5. **Non-blocking Operation**: State machine allows concurrent safety monitoring
6. **Position Self-Correction**: Auto-corrects position tracking when hitting limit switches
7. **Homing During Operation**: Non-blocking homing allows status monitoring throughout
5. **Non-blocking Operation**: State machine allows concurrent safety monitoring

### Fault Codes

**Position Command Out of Range**:
- **Condition**: `STEPPOS` target exceeds hardware limits
- **Action**: Command rejected, diagnostic message sent
- **Recovery**: Send valid position command within range
- **Log**: `ERROR: Invalid position command: elevator=X, extender=Y`

**Limit Switch Activated During Motion**:
- **Condition**: Unexpected limit switch activation
- **Action**: Motor stops immediately
- **Recovery**: Home motors or move away from limit
- **Log**: `WARNING: Limit switch activated: [motor] [upper/lower]`

**Homing Failure**:
- **Condition**: Limit switch not detected within expected travel
- **Action**: Homing sequence aborts, position marked invalid
- **Recovery**: Manual intervention, check hardware
- **Log**: `CRITICAL: Homing failed for [elevator/extender]`

### Example Sequences

**Initialization**:
```
STEPHOME:                               # Initiate homing
STEPPERSTAT3:{"elev_pos":0.000,"ext_pos":0.000,"elev_lim":"lower","ext_lim":"lower",...}
```

**Position Movement**:
```
STEPPOS:elevator:0.5,extender:0.2       # Move to mid-range
STEPPERSTAT3:{"elev_pos":0.500,"ext_pos":0.200,"elev_lim":"none","ext_lim":"none",...}
```

**Status Query**:
```
STEPSTATUS:                             # Query current state
STEPPERSTAT3:{"elev_pos":0.450,"ext_pos":0.120,...}
```

### Integration Notes

- **ROS2 Bridge**: sigyn_to_sensor_v2/teensy_bridge handles serial communication
- **Message Parsing**: message_parser.cpp needs updates for STEPPERSTAT3
- **Behavior Trees**: Create nodes using STEPPOS for position-based gripper control
- **Motion Planning**: Query max_travel values via STEPSTATUS before planning movements

---

## VL53L0XMonitor

### Purpose
Read 8× VL53L0X time-of-flight distance sensors via I2C multiplexer (TCA9548A). Detects obstacles and monitors sensor degradation.

### Location
- Header: `modules/sensors/vl53l0x_monitor.h`
- Implementation: `modules/sensors/vl53l0x_monitor.cpp`
- Driver: `modules/sensors/vl53l0x_driver.h`

### Compilation
```cpp
#define BOARD_HAS_DISTANCE_SENSORS 1  // In config.h to enable
```

### Loop Frequency
- Target: 30–100 Hz (reads one sensor per iteration, rotates through all 8)
- Actual: ~4 Hz per sensor (every ~250 ms per sensor), but all 8 sampled in parallel via multiplexer
- Per-module timing: 2.1–3.8 ms, average 2.44 ms

### Sensor Layout

**Board 1** has 8 VL53L0X sensors accessible via I2C multiplexer (TCA9548A):

| Sensor ID | Notes |
|-----------|-------|
| 0–7 | 8 total sensors; specific mounting positions not documented in code |

**Note**: The physical mounting positions (direction, orientation) of each sensor are not defined in the firmware. Refer to hardware documentation or robot CAD model for sensor layout details.

### Key Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| I2C Bus | Wire (Teensy standard) | TCA9548A multiplexer on pins SDA/SCL |
| VL53L0X Timeout | 1000 ms | Measurement timeout per sensor |
| Obstacle Threshold | 200 mm | Default; triggers `obstacles=true` |
| Degraded Threshold | All 8 sensors | Only triggers fault if ALL degraded |
| Measurement Timing | ~33 ms per sensor | 8 sensors × 33 ms = ~264 ms full cycle |

### Degradation Detection

VL53L0X internally provides **signal strength** and **ambient light**. When either is poor:

- `degraded = true`: Low confidence in this measurement
- **Causes**: Dirty lens, laser pointing at reflective surface, insufficient ambient light
- **Action**: RoboClawMonitor may reduce motor speed in degraded mode (safety consideration)

### Fault Codes

**All Sensors Degraded**:
- **Threshold**: All 8 sensors report `degraded=true` simultaneously
- **Recovery**: Automatic when ≥1 sensor recovers to good signal
- **Log**: `"DEGRADATION: all_8_sensors_degraded"`
- **Reason**: Indicates robot is in area with poor optical properties; blind navigation mode

**Single Sensor Init Failure**:
- **Threshold**: VL53L0X does not respond to initialization
- **Recovery**: Manual (Teensy reboot likely needed)
- **Log**: `"sensor_init_failed: id=3"`
- **Reason**: I2C communication error or dead sensor

### Actual Logs

**All sensors good**:
```json
VL53L0X1:{"total_sensors":8,"active_sensors":8,"min_distance":165,"max_distance":1735,"obstacles":true,"distances":[{"id":0,"mm":265,"raw":271,"age_us":2908,"degraded":false},{"id":1,"mm":1031,"raw":1043,"age_us":12772,"degraded":false},...]}
```

**Sensor 3 degraded (dirty)**:
```json
VL53L0X1:{"distances":[...,{"id":3,"mm":191,"raw":194,"age_us":2702,"degraded":true},...]}
```

**Initialization log**:
```json
DIAG1:{"level":"INFO","module":"VL53L0XMonitor","message":"sensor=0,init=success,test_dist=264"}
DIAG1:{"level":"INFO","module":"VL53L0XMonitor","message":"sensor=1,init=success,test_dist=512"}
...
```

---

## TemperatureMonitor

### Purpose
Monitor motor winding temperatures via NTC thermistors. Detect overheating and enforce thermal shutdown.

### Location
- Header: `modules/sensors/temperature_monitor.h`
- Implementation: `modules/sensors/temperature_monitor.cpp`

### Compilation
```cpp
#define BOARD_HAS_TEMPERATURE_SENSOR 1  // In config.h to enable
```

### Loop Frequency
- Read interval: 100 ms per sensor (`read_interval_ms` default)
- Status reporting: 1 Hz (`status_report_interval_ms`)

### Sensor Details

**Sensors**: Analog sensors configured via `TemperatureSensorConfig` (names, pins, thresholds per board)

**Thresholds (defaults)**:
- Warning: 70°C per sensor
- Critical: 85°C per sensor
- System warning: 65°C; system critical: 80°C

### Thermal Characteristics

Temperature rise/cooling depend on hardware and environment; firmware does not encode expected curves.
- From 70°C to 50°C: ~10–15 minutes

### Fault Codes

**Motor Thermal Critical**:
- **Threshold**: Configurable; defaults 85°C per sensor or 80°C system
- **Recovery**: Clears when sensors drop below configured thresholds
- **Log**: Reported via `TemperatureMonitor` and SafetyCoordinator with descriptions

**Sensor Communication Error**:
- **Threshold**: ADC read fails or invalid resistance
- **Recovery**: Manual (Teensy reboot)
- **Log**: `"sensor_read_failed: id=0"`
- **Common Causes**: Loose thermistor wire, disconnected sensor

### Actual Logs

**Idle (room temperature)**:
```json
TEMPERATURE1:{"total_sensors":8,"active_sensors":2,"temperatures":[25.4,24.9,null,null,null,null,null,null],"avg_temp":25.1,"max_temp":25.4,"min_temp":24.9,"hottest_sensor":0,"system_warning":false,"system_critical":false}
```

**After sustained motion**:
```json
TEMPERATURE1:{"temperatures":[65.2,64.1],"avg_temp":64.6,"max_temp":65.2,"system_warning":true,"system_critical":false}
```

**Critical thermal shutdown**:
```json
TEMPERATURE1:{"temperatures":[81.5,80.2],"max_temp":81.5,"system_critical":true}
DIAG1:{"level":"ERROR","module":"TemperatureMonitor","message":"CRITICAL: left=81.5C, right=80.2C"}
```

---

## BatteryMonitor

### Purpose
Monitor the 36V (10s LIPO) main battery and various DC-DC regulated voltage rails. Detects low battery and brownout conditions on key power lines.

### Location
- Header: `modules/battery/battery_monitor.h`
- Implementation: `modules/battery/battery_monitor.cpp`

### Compilation
```cpp
#define BOARD_HAS_BATTERY_MONITOR 1  // In config.h to enable
```

### Loop Frequency
- Update period: 100 ms per config (default)
- Reports: status once per second

### Voltage Measurement

**Monitoring Types**:
1. **LIPO Battery (Sensor 0)**:
   - **Source**: 10s LiPo Battery (~36V nominal)
   - **Method**: 
     - Calculates actual battery voltage using a configured voltage divider multiplier (`voltage_multiplier` = 2.375).
     - Reads true current via INA226 sensing resistor.
   - **Purpose**: Main power source monitoring (Critical for system safety).

2. **DC-DC Converters (Sensors 1–4)**:
   - **Sources**: 5V, 12V, 24V, 3.3V Rails
   - **Method**: Direct voltage monitoring (Multiplier = 1.0).
   - **Purpose**: Verifies health of internal power regulation rails.

**Sensors**: INA226 accessed via I2C multiplexer.

### Key Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Low Threshold | 34.0 V (LIPO) | Battery ~55% discharged, warning state |
| Critical Threshold | 32.0 V (LIPO) | Battery ~45% discharged, triggers E-stop |
| Hysteresis | 2.0 V | Fault clears at 34 V |
| Measurement Rate | 1 Hz | Every 1000 ms |

### Typical Battery Life

**LiPo 6S (22.2V nominal, 20V minimum)**:

| Threshold | Voltage | Behavior |
|-----------|---------|----------|
| Warning | 34.0 V (default) | Report warning, continue running |
| Critical | 32.0 V (default) | Raise `FaultSeverity::EMERGENCY_STOP` |
| High current | 20.0 A (default) | Also triggers critical fault |

### Fault Codes

**Battery Critical**:
- **Threshold**: Main voltage < 32.0 V
- **Recovery**: Automatic when voltage rises to ≥ 34.0 V (requires recharge)
- **Log**: `"CRITICAL: voltage=31.8V"`
- **Impact**: Motors disabled until battery recharged

**Voltage Measurement Failure**:
- **Threshold**: Unable to read battery voltage
- **Recovery**: Manual (check voltage measurement circuit)
- **Log**: `"read_failed: battery_voltage_invalid"`
- **Common Causes**: Disconnected sensor, ADC failure

### Actual Logs

**Normal operation**:
```json
{"battery_voltage": 40.2, "state": "good"}
```

**Battery critical**:
```json
{"battery_voltage": 31.8, "state": "critical"}
DIAG1:{"level":"ERROR","module":"BatteryMonitor","message":"CRITICAL: voltage=31.8V"}
```

---

## PerformanceMonitor

### Purpose
Track execution time of each module and detect performance violations (timeouts, frequency drops).

### Location
- Header: `modules/performance/performance_monitor.h`
- Implementation: `modules/performance/performance_monitor.cpp`

### Compilation
Always enabled (part of core framework)

### Metrics Tracked

**Per-Module**:
- Min execution time (ms)
- Max execution time (ms)
- Average execution time (ms)
- Last execution time (ms)
- Iteration count
- Timeout violations

**Overall Loop**:
- Actual frequency (Hz) from Module::getCurrentLoopFrequency()
- Minimum frequency threshold (per board)
- Frequency violations

### Key Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Max module time | From `BOARD_MAX_MODULE_TIME_MS` (Board1 2 ms, Board2 3 ms, Board3 5 ms defaults) | Set in `config.h` |
| Min loop frequency | From `BOARD_MIN_LOOP_FREQUENCY_HZ` (Board1 50 Hz, Board2 20 Hz, Board3 10 Hz defaults) | Set in `config.h` |
| Frequency violation tolerance | `max_frequency_violations` (default 5) | Consecutive violations before flagging unsafe |
| Module violation tolerance | `max_violation_count` (default 8) | Consecutive module overruns before flagging unsafe |
| Stats interval | 1000 ms | `stats_report_interval_ms` |

### Actual Performance

From logs (typical config):

```json
PERF1:{
  "freq":297.6,
  "tfreq":30.0,
  "mods":[
    {"n":"SDLogger","min":0.00,"max":0.04,"avg":0.00,"cnt":1544},
    {"n":"RoboClawMonitor","min":0.80,"max":4.77,"avg":1.37,"cnt":1543},
    {"n":"VL53L0XMonitor","min":2.08,"max":3.77,"avg":2.44,"cnt":1543},
    {"n":"TemperatureMonitor","min":0.00,"max":1.94,"avg":0.08,"cnt":1543}
  ]
}
```

**Analysis**:
- Minimum loop threshold comes from `config.h` (Board1 50 Hz). The example shows the loop running faster (≈298 Hz) because work per cycle is light.
- Module execution times stay below the configured `BOARD_MAX_MODULE_TIME_MS` values in the example run.

### Fault Handling

- The module tracks violations but `isUnsafe()` currently returns `false`; it does not assert E-stop by itself.
- Violations are counted for diagnostics (`violations_`), and warnings/critical messages are sent when thresholds are exceeded (see `performance_monitor.cpp`).

### Actual Logs

**Normal boot sequence**:
```json
DIAG1:{"level":"INFO","module":"PerformanceMonitor","message":"Loop frequency: 297.6 Hz"}
PERF1:{"freq":297.6,"tfreq":30.0,"mviol":0,"fviol":0,"mods":[...]}
```

**Module timeout (hypothetical)**:
```json
DIAG1:{"level":"ERROR","module":"PerformanceMonitor","message":"VIOLATION: RoboClawMonitor exceeded 100ms (actual=142ms)"}
PERF1:{"mviol":1,"mods":[{"n":"RoboClawMonitor","viol":true}]}
```

---

## SDLogger

### Purpose
Log diagnostic data and performance statistics to MicroSD card for post-mission analysis.

### Location
- Header: `modules/storage/sd_logger.h`
- Implementation: `modules/storage/sd_logger.cpp`

### Compilation
```cpp
#define BOARD_HAS_SD_CARD 1  // In config.h to enable
```

### Loop Frequency
- Target: 1–10 Hz (asynchronous writes)
- Actual: ~1 Hz (batches every 1000 ms)
- Per-module timing: 0–0.04 ms (fast write buffer)

### File Format

**Log File**: `LOG00246.TXT` (on MicroSD, numbered sequentially)

**Format**: Newline-delimited JSON (same as serial output)

```
DIAG1:{"level":"INIT","module":"Module","message":"starting_module_setup","timestamp":9302}
DIAG1:{"level":"INFO","module":"SDLogger","message":"Initializing SD card","timestamp":9302}
...
ROBOCLAW1:{"MainVoltage":24.1,"LeftMotorCurrent":2.5,...}
VL53L0X1:{...}
TEMPERATURE1:{...}
PERF1:{...}
```

**Write Rate** (typical): ~10 KB/s (measured in logs as "10565 B/s")

**Buffer Size**: 8 KB per write batch; files auto-flushed every ~1 second

### Statistics Computed

At shutdown or when requested:

```json
DIAG1:{"level":"INFO","module":"SDLogger","message":"Statistics: 1544 total_messages, 10565 bytes/sec, uptime=146sec"}
```

### Fault Codes

**SD Card Init Failed**:
- **Threshold**: SD card not detected or not initialized
- **Recovery**: Manual (check SD card connection, reboot Teensy)
- **Log**: `"SD_INIT_FAILED: sd_begin() returned false"`
- **Common Causes**: Loose SD card, corrupted filesystem

**File Open Failed**:
- **Threshold**: Cannot create new log file
- **Recovery**: Manual (check filesystem, format if needed)
- **Log**: `"FILE_OPEN_FAILED: LOG00247.TXT"`
- **Common Causes**: SD card full, corrupted FAT table

**Write Buffer Full**:
- **Threshold**: More data arriving than buffer can hold
- **Recovery**: Automatic (data is dropped, message logged)
- **Log**: `"BUFFER_OVERFLOW: dropped_12_messages"`
- **Reason**: Should not happen in normal operation (indicates system overload)

### How to Retrieve Logs

1. **Power off robot** and Teensy
2. **Locate MicroSD card slot** on Board 1 (Teensy 4.1)
3. **Remove SD card** carefully
4. **Insert into PC card reader**
5. **Open `LOG00246.TXT`** with text editor or Python script
6. **Parse JSON** for diagnostics:
   ```bash
   grep '"level":"ERROR"' LOG00246.TXT  # Find all errors
   ```

---

## SafetyCoordinator

### Purpose
Central fault aggregator that modules call to raise or clear faults with severities.

### Location
- Header: `modules/safety/safety_coordinator.h`
- Implementation: `modules/safety/safety_coordinator.cpp`

### Compilation
Built as part of the module set; enabled when `ENABLE_SAFETY` is true.

### Fault Model
- Stores up to 16 faults, keyed by source string.
- Each fault has a `FaultSeverity`: `NORMAL`, `WARNING`, `DEGRADED`, `EMERGENCY_STOP`, `SYSTEM_SHUTDOWN`.
- E-stop behavior: faults raised with `EMERGENCY_STOP`/`SYSTEM_SHUTDOWN` increment `active_estop_count_`; when the count transitions from 0→1 the RoboClaw E-stop line is asserted (if `CONTROLS_ROBOCLAW_ESTOP_PIN` is compiled in). Clearing the last such fault releases the line.
- Reporting: once per second sends `FAULT` JSON messages over `SerialManager` for each active fault (or `active_fault:false` when none).
- Commands: `setEstopCommand("trigger=true")` raises an E-stop fault; `setEstopCommand("reset=true")` calls `Module::resetAllSafetyFlags()`.
- Recovery: faults are cleared only by callers (`deactivateFault`) or by a global reset; no automatic clearing inside the coordinator.

### Sources (as implemented)
- RoboClawMonitor: overcurrent (latched), runaway, fatal RoboClaw error bits, RoboClaw over-temperature.
- BatteryMonitor: critical low voltage (<32 V default), critical high current (>20 A default), warning low voltage (<34 V default).
- Software commands: `setEstopCommand("trigger=true")` from host.
- Other modules can call `activateFault` directly as needed.

---

## Related Documentation

- [ARCHITECTURE.md](./ARCHITECTURE.md) — System design overview
- [Safety_System.md](./Safety_System.md) — Fault handling & recovery
- [Message_Formats.md](./Message_Formats.md) — Message types and examples
