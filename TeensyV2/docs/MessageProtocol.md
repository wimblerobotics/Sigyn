# TeensyV2 Message Protocol Specification

## Overview

The TeensyV2 system uses a structured text-based message protocol for communication between the embedded Teensy boards and the ROS2 system. This protocol is designed for efficiency, reliability, and easy parsing while maintaining human readability for debugging.

## Message Format

All messages follow a consistent structure:

```
TYPE:key1=val1,key2=val2,key3=val3,...
```

### Components

- **TYPE**: Message type identifier (4-8 characters, uppercase)
- **:** (colon): Separator between type and data
- **key=val**: Key-value pairs containing message data
- **,** (comma): Separator between key-value pairs

### Constraints

- Maximum message length: 512 characters
- Key names: alphanumeric + underscore, max 16 characters
- Values: printable ASCII characters, max 32 characters per value
- No spaces around separators (for efficiency)
- Case-sensitive keys and values

## Message Types

### BATT - Battery Status Messages

Battery monitoring data from INA226 sensors and analog voltage monitoring.

**Format:**
```
BATT:idx=<sensor_id>,V=<voltage>,A=<current>,charge=<percentage>,state=<state>
```

**Parameters:**
- `idx`: Sensor/battery identifier (0-255)
- `V`: Voltage in volts (float, 2 decimal places)
- `A`: Current in amperes (float, 2 decimal places, positive=discharge)
- `charge`: Charge percentage (float, 0.0-1.0)
- `state`: Battery state (UNKNOWN, CHARGING, DISCHARGING, CRITICAL, WARNING, NORMAL)

**Examples:**
```
BATT:idx=0,V=42.51,A=1.15,charge=1.00,state=NORMAL
BATT:idx=0,V=32.10,A=0.12,charge=0.15,state=CRITICAL
BATT:idx=0,V=41.50,A=-2.34,charge=0.95,state=CHARGING
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
PERF:{"freq":85.2,"target_freq":80.0,"mod_viol":0,"freq_viol":0,"modules":[{"name":"PerformanceMonitor","min":0.12,"max":0.18,"avg":0.15,"last":0.14,"count":1245,"violation":false},{"name":"BatteryMonitor","min":0.35,"max":0.92,"avg":0.48,"last":0.45,"count":1245,"violation":false}]}

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
- `BATT` messages → `/sigyn/battery/status` (sensor_msgs/BatteryState)
- `PERF` messages → `/sigyn/performance/stats` (custom message)
- `SAFETY` messages → `/sigyn/safety/status` (diagnostic_msgs/DiagnosticArray)
- `ESTOP` messages → `/sigyn/safety/estop_events` (custom message)
- `DIAG` messages → `/sigyn/diagnostics` (diagnostic_msgs/DiagnosticArray)

#### Subscribed Topics (ROS2 → Teensy)
- `/sigyn/commands/config` → `CONFIG` messages
- `/sigyn/commands/estop` → `ESTOP` trigger commands
- `/sigyn/parameters/*` → `CONFIG` parameter updates

### Parameter Integration
- ROS2 parameters automatically sync with embedded system
- Parameter changes trigger `CONFIG` messages
- Embedded system responds with current values
- Parameter validation on both sides

## Message Examples by Scenario

### Normal Operation
```
BATT:id=0,v=38.45,c=2.134,p=82.02,pct=0.72,state=DISCHARGING,sensors=INA226
PERF:freq=85.1,exec=1.67,viol=0,modules=4,avg_freq=84.9,max_exec=1.89
SAFETY:state=NORMAL,hw_estop=false,inter_board=false,active_conditions=false
```

### Battery Critical Condition
```
BATT:id=0,v=31.89,c=1.567,p=49.98,pct=0.12,state=CRITICAL,sensors=INA226
ESTOP:active=true,source=BATTERY_VOLTAGE,reason=Critical low voltage,value=31.89,manual_reset=false,time=234567
SAFETY:state=ESTOP,hw_estop=false,inter_board=false,active_conditions=true,sources=BATTERY_VOLTAGE
```

### Performance Issues
```
PERF:freq=68.2,exec=4.23,viol=3,modules=5,avg_freq=75.4,max_exec=6.78
DIAG:level=WARN,module=PerformanceMonitor,msg=Performance degradation detected,details=freq_low=3 time_high=2,time=345678
ESTOP:active=true,source=PERFORMANCE,reason=Critical timing violation,value=6.78,manual_reset=false,time=345679
```

### Configuration Update
```
// ROS2 sends configuration command
CONFIG:cmd=SET,module=BatteryMonitor,param=critical_low_voltage,value=29.0

// Teensy responds with confirmation
CONFIG:status=OK,module=BatteryMonitor,param=critical_low_voltage,value=29.0

// Updated battery monitoring with new threshold
BATT:id=0,v=30.12,c=0.895,p=26.96,pct=0.08,state=WARNING,sensors=INA226
```

This protocol provides a robust, efficient, and extensible communication framework for the TeensyV2 system while maintaining simplicity and debuggability.
