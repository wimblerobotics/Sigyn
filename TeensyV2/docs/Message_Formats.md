# TeensyV2 Message Formats

**Table of Contents**
- [Overview](#overview)
- [Format Convention](#format-convention)
- [Sensor Messages](#sensor-messages)
  - [BATT: Battery Status](#batt-battery-status)
  - [PERF: Performance Metrics](#perf-performance-metrics)
  - [IMU: Inertial Measurement Unit](#imu-inertial-measurement-unit)
  - [VL53L0X: Distance Sensor Array](#vl53l0x-distance-sensor-array)
  - [ROBOCLAW: Motor Driver Status](#roboclaw-motor-driver-status)
  - [TEMPERATURE: Temperature Monitor](#temperature-temperature-monitor)
  - [ODOM: Odometry / Pose](#odom-odometry--pose)
  - [FAULT: Active Faults](#fault-active-faults)
- [Command Messages](#command-messages)
  - [TWIST: Velocity Command](#twist-velocity-command)
  - [STEPPOS: Stepper Position Command](#steppos-stepper-position-command)
  - [STEPHOME: Stepper Homing Command](#stephome-stepper-homing-command)
  - [STEPSTATUS: Stepper Status Query](#stepstatus-stepper-status-query)
  - [ESTOP: Emergency Stop](#estop-emergency-stop)
- [Diagnostic Messages](#diagnostic-messages)
- [Examples from Live System](#examples-from-live-system)

## Overview

All messages between TeensyV2 boards and the ROS2 host are transmitted as **newline-delimited JSON** over USB serial. This document specifies the exact format and meaning of each message type.

### Design Rationale

- **Compact keys**: Minimizes bandwidth (e.g., `"v"` instead of `"voltage"`)
- **Fixed arrays**: Predictable size for embedded systems
- **JSON-first**: Easy parsing in Python/C++
- **Extensible**: New fields can be added without breaking parsers
- **Human-readable**: Can debug directly from serial monitor

---

## Format Convention

### Message Prefix

Each line begins with a **message type prefix**, followed by a colon:

```
TYPE<ID>:{"key": value, ...}
```

Where `<ID>` corresponds to the board ID (e.g., `1` for Main/Nav, `2` for Power/Sensor, `3` for Elevator).

**Common Prefixes**:
- `BATT2` — Battery status (Board 2)
- `IMU2` — IMU data (Board 2)
- `PERF1`, `PERF2` — Performance stats
- `VL53L0X1` — Distance sensors (Board 1)
- `ROBOCLAW1` — Motor status (Board 1)
- `STEPPERSTAT3` — Stepper motor status (Board 3)
- `ODOM1` — Odometry (Board 1)
- `DIAG1`, `DIAG2` — Diagnostics
- `FAULT1`, `FAULT2` — Fault alerts
- `TWIST` — Velocity command (input to Teensy)
- `STEPPOS` — Stepper position command (input to Teensy, Board 3)
- `STEPHOME` — Stepper homing command (input to Teensy, Board 3)
- `STEPSTATUS` — Stepper status query (input to Teensy, Board 3)
- `ESTOP` — Emergency stop command (input to Teensy)

### JSON Key Conventions

- **Short keys** for compact transmission
- **Units specified in key name** when not obvious (e.g., `mm` for millimeters)
- **Null values** permitted for inactive/uninitialized sensors

---

## Sensor Messages

### BATT: Battery Status

**Frequency**: ~1 Hz (per battery rail)

**Message format**:
```json
BATT2:{
  "idx": 0,
  "V": 42.07,
  "A": 0.00,
  "charge": 1.0,
  "state": "NORMAL",
  "location": "36VLIPO"
}
```

**Fields**:
- `idx` (int): Battery/Rail index
- `V` (float, V): Voltage
- `A` (float, A): Current (positive = discharge)
- `charge` (float): State of charge (0.0 - 1.0, where 1.0 implies 100% charged)
- `state` (string): `NORMAL`, `CHARGING`, `DISCHARGING`, `CRITICAL`
- `location` (string): Rail identifier (e.g., "36VLIPO", "5VDCDC")

**Example (Board 2)**:
```json
BATT2:{"idx":0,"V":42.07,"A":0.00,"charge":1,"state":"NORMAL","location":"36VLIPO"}
BATT2:{"idx":1,"V":0.00,"A":0.00,"charge":0,"state":"DEGRADED","location":"5VDCDC"}
```

---

### PERF: Performance Metrics

**Frequency**: ~0.2 Hz (Every 5 seconds)

**Message format**:
```json
PERF2:{
  "freq": 500000.0,
  "tfreq": 20.0,
  "mviol": 1013709,
  "fviol": 131,
  "mods": [
    {"n": "SDLogger", "min": 0.0, "max": 71.89, "avg": 0.0, "last": 0.03, "cnt": 3003, "viol": false},
    ...
  ]
}
```

**Fields**:
- `freq` (float, Hz): Actual loop frequency (should be ~85 Hz for board1)
- `tfreq` (float, Hz): Target loop frequency
- `mviol` (int): Total module timing violations
- `fviol` (int): Frequency violations (loop too slow)
- `violdet`: Details of last violation
  - `cmod` (int): Module index with violation (255 = frequency)
  - `cfreq` (float): Loop frequency at violation time
  - `lastms` (int): Timestamp of last violation
- `mods` (array of module objects):
  - `n` (string): Module name
  - `min`/`max`/`avg`/`last` (float, ms): Execution time stats
  - `cnt` (int): Execution count since last reporting period
  - `viol` (bool): If currently violating timing constraints

**Example**:
```json
PERF1:{"freq":297.6,"tfreq":30.0,"mviol":1543,"fviol":0,"violdet":{"cmod":255,"cfreq":0,"lastms":16630},"mods":[{"n":"SDLogger","min":0.00,"max":0.04,"avg":0.00,"last":0.03,"cnt":1544,"viol":false},{"n":"RoboClawMonitor","min":0.80,"max":4.77,"avg":1.37,"last":0.81,"cnt":1543,"viol":false}]}
```

---

### IMU: Inertial Measurement Unit

**Frequency**: High (depends on sensor, e.g. 100Hz+)

**Message format**:
```json
IMU2:{
  "id": 0,
  "qx": -0.8290, "qy": -0.5583, "qz": 0.0005, "qw": 0.0334,
  "gx": -0.00109, "gy": -0.00218, "gz": 0.00109,
  "ax": -0.03000, "ay": 0.00000, "az": 0.04000,
  "calib": 48,
  "status": 5,
  "error": 0,
  "timestamp": 14520645
}
```

**Fields**:
- `id` (int): Sensor ID (0=Primary, 1=Secondary)
- `qx`,`qy`,`qz`,`qw` (float): Orientation quaternion
- `gx`,`gy`,`gz` (float, rad/s): Angular velocity
- `ax`,`ay`,`az` (float, m/s²): Linear acceleration
- `calib` (int): Calibration status (BNO055 specific)
- `status` (int): System status
- `error` (int): Error flags
- `timestamp` (int, ms): Timestamp

**Example**:
```json
IMU2:{"id":0,"qx":-0.8290,"qy":-0.5583,"qz":0.0005,"qw":0.0334,"gx":-0.00109,"gy":-0.00218,"gz":0.00109,"ax":-0.03000,"ay":0.00000,"az":0.04000,"calib":48,"status":5,"error":0,"timestamp":14520645}
```

---

### VL53L0X: Distance Sensor Array

**Frequency**: ~1 Hz

**Message format**:
```json
VL53L0X1:{
  "total_sensors": 8,
  "active_sensors": 8,
  "min_distance": 165,
  "max_distance": 1735,
  "obstacles": true,
  "distances": [
    {"id": 0, "mm": 265, "raw": 271, "age_us": 2908, "degraded": false},
    ...
  ]
}
```

**Fields**:
- `total_sensors` (int): Total number of sensors configured (fixed; 8× VL53L0X sensors).
- `active_sensors` (int): Number of sensors currently active.
- `min_distance` (int, mm): Minimum distance detected across all sensors.
- `max_distance` (int, mm): Maximum distance detected across all sensors.
- `obstacles` (bool): True if any sensor detects an object within the safety threshold.
- `distances` (array): List of individual sensor readings.
  - `id` (int): Sensor index (0-7).
  - `mm` (int): Filtered distance in millimeters.
  - `raw` (int): Raw sensor reading.
  - `age_us` (int): Time in microseconds since last measurement (staleness).
  - `degraded` (bool): True if the reading quality is suspect (e.g. low signal).

**Typical ranges**:
- Sensors measure 30 mm to 1800+ mm
- Closer objects return higher confidence
- Far obstacles may have low confidence (high degraded flag)

**Example from logs**:
```json
VL53L0X1:{"total_sensors":8,"active_sensors":8,"min_distance":172,"max_distance":1676,"obstacles":true,"distances":[{"id":0,"mm":265,"raw":265,"age_us":2702,"degraded":false},...]}
```

---

### ROBOCLAW: Motor Driver Status

**Frequency**: ~1 Hz

**Message format**:
```json
ROBOCLAW1:{
  "LogicVoltage": 0.0,
  "MainVoltage": 24.1,
  "Encoder_Left": 0,
  "Encoder_Right": 0,
  "LeftMotorCurrent": 0.0,
  "RightMotorCurrent": 0.0,
  "LeftMotorSpeed": 0,
  "RightMotorSpeed": 0,
  "Error": "0",
  "ErrorDecoded": "No errors"
}
```

**Fields**:
- `LogicVoltage` (float, V): 5V logic supply voltage (USB logic power).
- `MainVoltage` (float, V): Main motor rail voltage (24V).
- `Encoder_Left`, `Encoder_Right` (int): Absolute encoder counts.
- `LeftMotorCurrent`, `RightMotorCurrent` (float, A): Motor current draw (peak ~20A).
- `LeftMotorSpeed`, `RightMotorSpeed` (int, QPPS): Current speed in Quad Pulses Per Second.
- `Error` (string): Hexadecimal error code mask.
- `ErrorDecoded` (string): Human-readable error string.

**Example from logs**:
```json
ROBOCLAW1:{"LogicVoltage":0.0,"MainVoltage":24.1,"Encoder_Left":0,"Encoder_Right":0,"LeftMotorCurrent":0.000,"RightMotorCurrent":0.000,"LeftMotorSpeed":0,"RightMotorSpeed":0,"Error":"0","ErrorDecoded":"No errors"}
```

---

### STEPPERSTAT3: Stepper Motor Status

**Frequency**: On request (via STEPSTATUS command) or ~1 Hz during motion

**Message format**:
```json
STEPPERSTAT3:{
  "elev_pos": 0.1234,
  "ext_pos": 0.0567,
  "elev_lim": "none",
  "ext_lim": "upper",
  "elev_max": 0.8999,
  "ext_max": 0.3418
}
```

**Fields**:
- `elev_pos` (float, m): Current elevator position (0.0 = bottom, positive = up)
- `ext_pos` (float, m): Current extender position (0.0 = retracted, positive = extended)
- `elev_lim` (string): Elevator limit switch state (`"none"`, `"upper"`, `"lower"`, `"both"`)
- `ext_lim` (string): Extender limit switch state (`"none"`, `"upper"`, `"lower"`, `"both"`)
- `elev_max` (float, m): Maximum elevator travel (hardware-measured limit: 0.8999m)
- `ext_max` (float, m): Maximum extender travel (hardware-measured limit: 0.3418m)

**Example from logs**:
```json
STEPPERSTAT3:{"elev_pos":0.4500,"ext_pos":0.1200,"elev_lim":"none","ext_lim":"none","elev_max":0.8999,"ext_max":0.3418}
```

**Notes**:
- Position values are absolute: 0.0 = home position (lower limit for elevator, retracted for extender)
- Limit switch states indicate hardware limit detection for safety
- Max values are hardcoded from physical measurements and used for bounds checking
- Board 3 sends this message in response to STEPSTATUS or autonomously during movements

---

### TEMPERATURE: Temperature Monitor

**Frequency**: ~1 Hz (when enabled)

**Message format**:
```json
TEMPERATURE1:{
  "total_sensors": 8,
  "active_sensors": 2,
  "temperatures": [
    25.4, 25.0, null, null, null, null, null, null
  ],
  "avg_temp": 25.2,
  "max_temp": 25.4,
  "min_temp": 25.0,
  "hottest_sensor": 0,
  "system_warning": false,
  "system_critical": false,
  "rate_hz": 19.9,
  "readings": 140,
  "errors": 0
}
```

**Fields**:
- `total_sensors` (int): Total possible sensors (e.g. 8).
- `active_sensors` (int): Count of responding sensors.
- `temperatures` (array): List of temperature readings in Celsius. `null` for inactive measurement points.
- `avg_temp` (float, °C): Average temperature of all active sensors.
- `max_temp` (float, °C): Leading (highest) temperature reading.
- `min_temp` (float, °C): Lowest temperature reading.
- `hottest_sensor` (int): Index of the sensor reporting `max_temp`.
- `system_warning` (bool): True if thermal warning threshold exceeded (e.g., >60°C).
- `system_critical` (bool): True if thermal critical threshold exceeded (e.g., >80°C).
- `rate_hz` (float): Current sampling rate in Hz.
- `readings` (int): Total measurements since startup.
- `errors` (int): Communication or sensor errors count.

**Example from logs**:
```json
TEMPERATURE1:{"total_sensors":8,"active_sensors":2,"temperatures":[25.4,24.9,null,null,null,null,null,null],"avg_temp":25.1,"max_temp":25.4,"min_temp":24.9,"hottest_sensor":0,"system_warning":false,"system_critical":false,"rate_hz":19.9,"readings":120,"errors":0}
```

---

### ODOM: Odometry / Pose

**Frequency**: ~1–2 Hz

**Message format**:
```json
ODOM1:{
  "px": 0.0, "py": 0.0,
  "ox": 0.0, "oy": 0.0, "oz": 0.0, "ow": 1.0,
  "vx": 0.0, "vy": 0.0, "wz": 0.0
}
```

**Fields**:
- `px`, `py` (float, m): Robot position in ***odom*** frame (accumulated odometry).
- `ox`, `oy`, `oz`, `ow` (float): Robot orientation quaternion.
- `vx`, `vy` (float, m/s): Linear velocity (vx = forward, vy = strafe).
- `wz` (float, rad/s): Angular velocity (yaw rate).

**Note**: These are accumulated on the Teensy so large values are possible. ROS2 nav system integrates this with other sensors (LIDAR, IMU) for map-level localization.

**Example**:
```json
ODOM1:{"px":0.000,"py":0.000,"ox":0.000,"oy":0.000,"oz":0.000,"ow":1.000,"vx":0.000,"vy":0.000,"wz":0.000}
```

---

### FAULT: Active Faults

**Frequency**: ~1 Hz

**Message format**:
```json
FAULT2:{
  "active_fault": "true",
  "source": "BatteryMonitor_Battery1",
  "severity": "DEGRADED",
  "description": "Sensor initialization failed",
  "timestamp": 6909
}
```

**Fields**:
- `source`: Configuration/Module name generating fault.
- `severity`: `INFO`, `WARN`, `DEGRADED`, `CRITICAL`.
- `description`: Human readable description.
- `active_fault` (bool or string): True if any E-stop source is active.

**Example**:
```json
FAULT2:{"active_fault":"true","source":"BatteryMonitor_Battery1","severity":"DEGRADED","description":"Sensor initialization failed for device 1 (5VDCDC)","timestamp":6909}
```

---

## Command Messages

Messages sent **from ROS2 to Teensy**.

### TWIST: Velocity Command

**Sent by**: ROS2 ros2_control / Navigation Stack (via sigyn_to_sensor_v2)
**Processed by**: RoboClawMonitor

**Format**:
```
TWIST:linear_x:0.5,angular_z:-0.1
```

**Parsing**:
- `linear_x` (float, m/s): Forward velocity (positive = forward)
- `angular_z` (float, rad/s): Angular velocity (positive = turn left)

**Example commands**:
```
TWIST:linear_x:0.5,angular_z:0.0    # Forward at 0.5 m/s
TWIST:linear_x:0.0,angular_z:1.5    # Rotate left at 1.5 rad/s
TWIST:linear_x:0.0,angular_z:0.0    # Stop
```

**How it's handled**:
1. SerialManager receives `TWIST` prefix
2. RoboClawMonitor parses `linear_x` and `angular_z`
3. Teensy calculates differential drive kinematics (converting m/s & rad/s → Left/Right RPM)
4. Issues velocity commands to RoboClaw motor controller

---

### STEPPOS: Stepper Position Command

**Sent by**: ROS2 behavior tree or manual control (via sigyn_to_sensor_v2)
**Processed by**: StepperMotor module (Board 3)

**Format**:
```
STEPPOS:elevator:0.5,extender:0.2
```

**Parsing**:
- `elevator` (float, m): Target elevator position (0.0 to 0.8999m)
- `extender` (float, m): Target extender position (0.0 to 0.3418m)

**Example commands**:
```
STEPPOS:elevator:0.5,extender:0.0      # Raise elevator to 50cm, retract extender
STEPPOS:elevator:0.8999,extender:0.3418  # Move to maximum travel positions
STEPPOS:elevator:0.0,extender:0.0      # Return to home position
```

**How it's handled**:
1. SerialManager receives `STEPPOS` prefix
2. StepperMotor module parses elevator and extender target positions
3. Validates positions against hardware limits (0.0-0.8999m elevator, 0.0-0.3418m extender)
4. Commands stepper motors to move to target positions
5. Returns `STEPPERSTAT3` message when motion completes

**Safety Notes**:
- Positions are clamped to measured hardware limits
- Limit switches provide hardware overtravel protection
- Invalid positions are rejected with diagnostic error message

---

### STEPHOME: Stepper Homing Command

**Sent by**: ROS2 initialization or manual control
**Processed by**: StepperMotor module (Board 3)

**Format**:
```
STEPHOME:
```

**Example**:
```
STEPHOME:    # Initiate homing sequence for both steppers
```

**How it's handled**:
1. SerialManager receives `STEPHOME` prefix
2. StepperMotor module initiates homing sequence:
   - **Step 1**: Retract extender to lower limit (safety: avoid collisions)
   - **Step 2**: Lower elevator to lower limit
3. Sets current position to 0.0 for both motors
4. Returns `STEPPERSTAT3` message when homing completes

**Safety Notes**:
- Homing sequence retracts extender **before** lowering elevator to prevent collisions
- Uses hardware limit switches to detect home position
- Should be performed after power-on and before position commands
- Establishes zero reference for all subsequent position commands

---

### STEPSTATUS: Stepper Status Query

**Sent by**: ROS2 monitoring or diagnostics
**Processed by**: StepperMotor module (Board 3)

**Format**:
```
STEPSTATUS:
```

**Example**:
```
STEPSTATUS:    # Request current stepper status
```

**Response**:
Board 3 immediately sends a `STEPPERSTAT3` message with current positions, limit states, and maximum travel values.

**Use cases**:
- Query current positions before planning movements
- Verify homing completed successfully
- Monitor limit switch states for diagnostics
- Retrieve hardware travel limits for motion planning

---

### ESTOP: Emergency Stop

**Sent by**: ROS2 safety monitor or behavior tree
**Processed by**: SafetyCoordinator

**Format**:
```
ESTOP:trigger=true
ESTOP:reset=true
```

**Example**:
```
ESTOP:trigger=true    # Trigger software E-stop
ESTOP:reset=true      # Reset/Clear E-stop
```

**Effect**:
- **trigger=true**:
  - Immediately stops motors (hardware & software safeing).
  - Activates `EMERGENCY_STOP` fault state.
  - Ignores new velocity commands.
- **reset=true**:
  - Calls `Module::resetAllSafetyFlags()`.
  - Clears active faults if conditions have normalized.
  - Returns system to `NORMAL` state if no physical E-stop is pressed.

---

## Diagnostic Messages

### DIAG: Log Messages

**Frequency**: Variable (initialization, errors, warnings)

**Message format**:
```json
DIAG2:{
  "level": "DEBUG",
  "module": "SDLogger",
  "message": "Performance stats: Buffer usage: 0...",
  "timestamp": 14521486
}
```

**Fields**:
- `level` (enum): `DEBUG`, `INFO`, `WARN`, `ERROR`, `INIT`
- `module` (string): Name of originating module
- `message` (string): Human-readable message
- `timestamp` (int, ms): Milliseconds since Teensy startup

**Example**:
```json
DIAG2:{"level":"DEBUG","module":"SDLogger","message":"SDLogger: Performance stats: Buffer usage: 0 bytes (0%), Write rate: 14226.00 B/s, Total writes: 482428","timestamp":14521486}
```

---

## Examples from Live System

### Startup Sequence

When a Teensy boots, diagnostic messages log module initialization:

```json
DIAG1:{"level":"INIT","module":"Module","message":"starting_module_setup","timestamp":9302}
DIAG1:{"level":"INFO","module":"SDLogger","message":"Initializing SD card","timestamp":9302}
DIAG1:{"level":"DEBUG","module":"SDLogger","message":"SD card begin() succeeded","timestamp":9313}
DIAG1:{"level":"INFO","module":"SDLogger","message":"Created log file: LOG00246.TXT","timestamp":9367}
DIAG1:{"level":"INIT","module":"Module","message":"module_initialized:SDLogger","timestamp":9367}
DIAG1:{"level":"INIT","module":"Module","message":"module_initialized:PerformanceMonitor","timestamp":9367}
DIAG1:{"level":"INIT","module":"Module","message":"module_initialized:SafetyCoordinator","timestamp":9367}
DIAG1:{"level":"INFO","module":"RoboClawMonitor","message":"Starting initialization","timestamp":9367}
DIAG1:{"level":"INFO","module":"RoboClawMonitor","message":"Version check passed: USB Roboclaw 2x15a v4.3.6","timestamp":9368}
DIAG1:{"level":"INFO","module":"RoboClawMonitor","message":"Initialization successful","timestamp":9376}
DIAG1:{"level":"INIT","module":"Module","message":"module_initialized:RoboClawMonitor","timestamp":9377}
DIAG1:{"level":"INFO","module":"VL53L0XMonitor","message":"Starting VL53L0X monitor initialization","timestamp":9577}
DIAG1:{"level":"INFO","module":"VL53L0XMonitor","message":"sensor=0,init=success,test_dist=264","timestamp":9666}
...
```

### Normal Operation (Idle)

When stopped, periodic messages are published:

```json
ROBOCLAW1:{"LogicVoltage":0.0,"MainVoltage":24.1,"Encoder_Left":0,"Encoder_Right":0,"LeftMotorCurrent":0.000,"RightMotorCurrent":0.000,"LeftMotorSpeed":0,"RightMotorSpeed":0,"Error":"0","ErrorDecoded":"No errors"}
VL53L0X1:{"total_sensors":8,"active_sensors":8,"min_distance":165,"max_distance":1735,"obstacles":true,"distances":[{"id":0,"mm":265,"raw":271,"age_us":2908,"degraded":false},...]}
TEMPERATURE1:{"total_sensors":8,"active_sensors":2,"temperatures":[25.4,24.9,null,...],"avg_temp":25.1,"max_temp":25.4,"min_temp":24.9,...}
ODOM1:{"px":0.000,"py":0.000,"ox":0.000,"oy":0.000,"oz":0.000,"ow":1.000,"vx":0.000,"vy":0.000,"wz":0.000}
PERF1:{"freq":297.6,"tfreq":30.0,"mviol":1543,...}
FAULT1:{"active_fault":"false"}
```

### Motor Command (Forward Motion)

ROS2 sends velocity command:
```
TWIST:linear_x:0.5,angular_z:0.0
```

Teensy executes, and encoder counts update:
```json
ROBOCLAW1:{"Encoder_Left":45,"Encoder_Right":43,"LeftMotorSpeed":100,"RightMotorSpeed":100,"LeftMotorCurrent":2.500,"RightMotorCurrent":2.450,...}
ODOM1:{"px":0.034,"py":0.000,"ox":0.000,"oy":0.000,"oz":0.000,"ow":1.000,"vx":0.15,"vy":0.000,"wz":0.000}
```

---

## Related Documentation

- [ARCHITECTURE.md](./ARCHITECTURE.md) — System design overview
- [Module_Reference.md](./Module_Reference.md) — Per-module parameter details
- [Safety_System.md](./Safety_System.md) — E-stop & fault handling
