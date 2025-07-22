# TeensyV2 Message Protocol Specification

## Overview

The TeensyV2 system uses a structured text-based message protocol for communication between the embedded Teensy boards and the ROS2 system. This protocol is designed for efficiency, reliability, and easy parsing while maintaining human readability for debugging.

## Message Format

The TeensyV2 system uses two message formats depending on the message type:

### Standard Key-Value Format
```
TYPE:key1:val1,key2:val2,key3:val3,...
```

### JSON Format (for complex data)
```
TYPE:<json_object>
```

### Components

- **TYPE**: Message type identifier (4-8 characters, uppercase)
- **:** (colon): Separator between type and data
- **key:val**: Key-value pairs containing message data (note: colon separator)
- **,** (comma): Separator between key-value pairs
- **<json_object>**: Valid JSON object for complex structured data

### Constraints

- Maximum message length: 1024 characters (increased for JSON support)
- Key names: alphanumeric + underscore, max 16 characters  
- Values: printable ASCII characters, max 32 characters per value
- JSON values: valid JSON syntax with proper escaping
- No spaces around separators (for efficiency)
- Case-sensitive keys and values

## Message Types

### BATT - Battery Status Messages

Battery monitoring data from INA226 sensors and analog voltage monitoring.

**Format:**
```
BATT:idx:<sensor_id>,V:<voltage>,A:<current>,charge:<percentage>,state:<state>,location:<location>
```

**Parameters:**
- `idx`: Sensor/battery identifier (0-255)
- `V`: Voltage in volts (float, 2 decimal places)
- `A`: Current in amperes (float, 2 decimal places, positive=discharge)
- `charge`: Charge percentage (float, 0.0-1.0)
- `state`: Battery state (UNKNOWN, CHARGING, DISCHARGING, CRITICAL, WARNING, NORMAL)
- `location`: Battery physical location/name (string, from battery configuration)

**Examples:**
```
BATT:idx:0,V:42.51,A:1.15,charge:1.00,state:NORMAL,location:36VLIPO
BATT:idx:1,V:5.02,A:0.85,charge:0.92,state:NORMAL,location:5VDCDC
BATT:idx:2,V:23.80,A:1.20,charge:0.78,state:NORMAL,location:24VDCDC
BATT:idx:3,V:3.25,A:0.35,charge:0.88,state:NORMAL,location:3.3VDCDC
BATT:idx:4,V:12.10,A:2.10,charge:0.65,state:WARNING,location:12VDCDC
```

### PERF - Performance Monitoring Messages

Real-time performance metrics from the embedded system control loop.

**Format:**
```
PERF:<json_object>
```

**JSON Structure:**
```json
{
  "freq": <frequency>,
  "target_freq": <target_frequency>,
  "mod_viol": <module_violations>,
  "freq_viol": <frequency_violations>,
  "violation_details": {
    "consecutive_mod": <consecutive_module_violations>,
    "consecutive_freq": <consecutive_frequency_violations>,
    "last_viol_ms": <last_violation_timestamp>
  },
  "modules": [
    {
      "name": "<module_name>",
      "min": <min_execution_time_ms>,
      "max": <max_execution_time_ms>,
      "avg": <average_execution_time_ms>,
      "last": <last_execution_time_ms>,
      "count": <execution_count>,
      "violation": <is_violating_timing>
    }
  ]
}
```

**Parameters:**
- `freq`: Current loop frequency in Hz (float, 1 decimal place)
- `target_freq`: Target loop frequency in Hz (float, 1 decimal place)
- `mod_viol`: Total module timing violations since reset (integer)
- `freq_viol`: Total frequency violations since reset (integer)
- `violation_details`: Present only when safety violations are active
- `modules`: Array of per-module performance statistics

**Examples:**
```
PERF:{"freq":1996.0, "target_freq":80.0, "mod_viol":0, "freq_viol":0, "modules":[{"name":"PerformanceMonitor","min":0.00,"max":0.05,"avg":0.00,"last":0.00,"count":9425,"violation":false},{"name":"BatteryMonitor","min":0.00,"max":1.59,"avg":0.01,"last":0.00,"count":9425,"violation":false}]}

PERF:{"freq":67.5,"target_freq":80.0,"mod_viol":2,"freq_viol":1,"violation_details":{"consecutive_mod":0,"consecutive_freq":1,"last_viol_ms":12345},"modules":[{"name":"PerformanceMonitor","min":0.10,"max":0.25,"avg":0.16,"last":0.15,"count":890,"violation":false},{"name":"BatteryMonitor","min":0.40,"max":3.45,"avg":0.68,"last":3.45,"count":890,"violation":true}]}
```

### SAFETY - Safety Status Messages

Overall safety system status and active conditions.

**Format:**
```
SAFETY:state=<state>,hw_estop=<hw_status>,inter_board=<board_status>,active_conditions=<has_conditions>,sources=<source_list>
```

**Parameters:**
- `state`: Safety state (NORMAL, WARNING, DEGRADED, ESTOP, SHUTDOWN)
- `hw_estop`: Hardware E-stop status (true/false)
- `inter_board`: Inter-board safety signal status (true/false)
- `active_conditions`: Any active E-stop conditions (true/false)
- `sources`: List of active E-stop sources (SOURCE1+SOURCE2+..., or empty if none)

**Examples:**
```
SAFETY:state=NORMAL,hw_estop=false,inter_board=false,active_conditions=false
SAFETY:state=ESTOP,hw_estop=true,inter_board=false,active_conditions=true,sources=HARDWARE
SAFETY:state=ESTOP,hw_estop=false,inter_board=true,active_conditions=true,sources=BATTERY_VOLTAGE+PERFORMANCE
```

### ESTOP - Emergency Stop Event Messages

Notifications when E-stop conditions are triggered or cleared.

**Format:**
```
ESTOP:active=<status>,source=<source>,reason=<description>,value=<trigger_value>,manual_reset=<manual>,time=<timestamp>
```

**Parameters:**
- `active`: E-stop activation status (true/false)
- `source`: E-stop source identifier (HARDWARE, SOFTWARE, BATTERY_VOLTAGE, BATTERY_CURRENT, MOTOR_CURRENT, MOTOR_RUNAWAY, PERFORMANCE, SENSOR, INTER_BOARD)
- `reason`: Human-readable description of the condition
- `value`: Trigger value if applicable (float, omitted if not applicable)
- `manual_reset`: Requires manual reset (true/false)
- `time`: Activation timestamp in milliseconds since boot (integer)

**Examples:**
```
ESTOP:active=true,source=BATTERY_VOLTAGE,reason=Critical low voltage,value=31.25,manual_reset=false,time=125430
ESTOP:active=false,source=BATTERY_VOLTAGE,reason=Voltage recovered,manual_reset=false,time=127890
ESTOP:active=true,source=HARDWARE,reason=E-stop button pressed,manual_reset=true,time=98765
ESTOP:active=true,source=PERFORMANCE,reason=Critical timing violation,value=5.23,manual_reset=false,time=156789
```

### DIAG - Diagnostic Messages

Diagnostic information, warnings, and error reports.

**Format:**
```
DIAG:level=<level>,module=<module>,msg=<message>,details=<details>,time=<timestamp>
```

**Parameters:**
- `level`: Diagnostic level (INFO, WARN, ERROR)
- `module`: Source module name (string, max 16 characters)
- `msg`: Primary diagnostic message (string, max 64 characters)
- `details`: Additional details (string, max 128 characters, optional)
- `time`: Timestamp in milliseconds since boot (integer)

**Examples:**
```
DIAG:level=INFO,module=BatteryMonitor,msg=INA226 initialized,time=1234
DIAG:level=WARN,module=PerformanceMonitor,msg=Loop frequency below target,details=freq=72.5Hz target=85.0Hz,time=45678
DIAG:level=ERROR,module=SafetyCoordinator,msg=Hardware E-stop pin read failed,details=pin=2 error=timeout,time=67890
```

### CONFIG - Configuration Messages

Configuration updates and responses between ROS2 and embedded system.

**Format (Command):**
```
CONFIG:cmd=<command>,module=<target>,param=<parameter>,value=<new_value>
```

**Format (Response):**
```
CONFIG:status=<status>,module=<target>,param=<parameter>,value=<current_value>,error=<error_msg>
```

**Command Parameters:**
- `cmd`: Command type (SET, GET, SAVE, RESET)
- `module`: Target module name (string)
- `param`: Parameter name (string)
- `value`: New parameter value (string, for SET commands)

**Response Parameters:**
- `status`: Operation status (OK, ERROR, INVALID, UNSUPPORTED)
- `module`: Target module name (string)
- `param`: Parameter name (string)
- `value`: Current parameter value (string)
- `error`: Error description if status is ERROR (optional)

**Examples:**
```
// Commands (ROS2 → Teensy)
CONFIG:cmd=SET,module=BatteryMonitor,param=critical_low_voltage,value=30.0
CONFIG:cmd=GET,module=PerformanceMonitor,param=target_frequency
CONFIG:cmd=SAVE,module=ALL

// Responses (Teensy → ROS2)
CONFIG:status=OK,module=BatteryMonitor,param=critical_low_voltage,value=30.0
CONFIG:status=OK,module=PerformanceMonitor,param=target_frequency,value=85.0
CONFIG:status=ERROR,module=InvalidModule,param=unknown,error=Module not found
```

### IMU - Inertial Measurement Unit Messages

Orientation data from BNO055 9-DOF IMU sensors for robot navigation.

**Format:**
```
IMU:id:<sensor_id>,qx:<quat_x>,qy:<quat_y>,qz:<quat_z>,qw:<quat_w>
```

**Parameters:**
- `id`: Sensor identifier (0-1 for dual sensor setup)
- `qx`, `qy`, `qz`, `qw`: Quaternion components (normalized, float)

**Key Features:**
- **Dual Sensor Support**: Two BNO055 sensors on I2C multiplexer channels 0 and 1
- **Fusion Data**: Quaternion represents fused orientation from gyro/accel/magnetometer
- **Real-time Performance**: Optimized for <1ms execution time per sensor
- **ROS2 Integration**: Direct compatibility with robot_localization EKF filter

**Examples:**
```
IMU:id:0,qx:0.1234,qy:-0.5678,qz:0.2468,qw:0.7890
IMU:id:1,qx:0.0987,qy:-0.6543,qz:0.3210,qw:0.6789
```

**Notes:**
- Only quaternion data is transmitted for efficiency (gyro/accel removed)
- Status and calibration data removed to improve performance
- Message rate: ~20Hz per sensor (40Hz total for dual setup)
- Coordinate frame: X-forward, Y-left, Z-up (ROS REP-103 compatible)

## Communication Protocol

### Serial Configuration
- **Baud Rate**: 921600 bps
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: None

### Message Transmission
- Each message terminated with newline character (`\n`)
- Maximum transmission rate: 100 messages/second per board
- No acknowledgment required for status messages
- Configuration commands should wait for response

### Error Handling
- Invalid messages are silently discarded
- Malformed key-value pairs are ignored
- Unknown message types generate DIAG error messages
- Communication timeouts trigger reconnection attempts

### Timing Requirements
- Status messages (BATT, PERF, SAFETY): 1-10 Hz typical
- Event messages (ESTOP, DIAG): Immediate when triggered
- Configuration messages: As needed, wait for response
- Maximum acceptable latency: 100ms for safety-critical messages

## ROS2 Integration

### Topic Mapping

#### Published Topics (Teensy → ROS2)
- `BATT` messages → `/battery_state` (sensor_msgs/BatteryState)
- `PERF` messages → `/performance_stats` (custom message)
- `SAFETY` messages → `/safety_status` (diagnostic_msgs/DiagnosticArray)
- `ESTOP` messages → `/estop_status` (custom message)
- `DIAG` messages → `/diagnostics` (diagnostic_msgs/DiagnosticArray)
- `IMU` messages → `/imu/sensor0`, `/imu/sensor1` (sensor_msgs/Imu)

#### Subscribed Topics (ROS2 → Teensy)
- `/commands/config` → `CONFIG` messages
- `/commands/estop` → `ESTOP` trigger commands
- `/parameters/*` → `CONFIG` parameter updates

### Parameter Integration
- ROS2 parameters automatically sync with embedded system
- Parameter changes trigger `CONFIG` messages
- Embedded system responds with current values
- Parameter validation on both sides

## ROS2 Parser Requirements

### Dual Format Support

The ROS2 message parser must handle both message formats:

1. **Key-Value Format**: `BATT:idx:0,V:40.61,A:1.10,charge:0.86,state:NORMAL`
2. **JSON Format**: `PERF:{"freq":1996.0, "target_freq":80.0, ...}`

### Parser Implementation

```cpp
// Pseudo-code for dual format parser
bool parseMessage(const std::string& message) {
    auto colon_pos = message.find(':');
    if (colon_pos == std::string::npos) return false;
    
    std::string type = message.substr(0, colon_pos);
    std::string data = message.substr(colon_pos + 1);
    
    if (type == "PERF") {
        // Handle JSON format
        return parseJsonMessage(type, data);
    } else {
        // Handle key-value format  
        return parseKeyValueMessage(type, data);
    }
}
```

### Configuration Options

The message validation can be controlled via ROS2 parameters:

- `enable_message_validation`: Enable/disable strict validation (default: true)
- `enable_json_parsing`: Enable/disable JSON format support (default: true)
- `treat_perf_as_string`: Treat PERF JSON as string data (default: false)

### Error Handling

- Invalid JSON in PERF messages should be logged but not cause node failure
- Malformed key-value pairs should be skipped with warnings
- Unknown message types should generate diagnostic messages
- Parser should be robust against malformed input

This protocol provides a robust, efficient, and extensible communication framework for the TeensyV2 system while maintaining simplicity and debuggability.
