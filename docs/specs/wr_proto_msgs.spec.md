# wr_proto_msgs — Shared Serial Protocol Library Specification

**Document status:** Draft v0.1  
**Last updated:** 2026-03-03  
**Scope:** The `wr_proto_msgs` C++ library — shared between the `wr_ros_teensy` ROS
2 package and the `wr_teensy_boards` firmware — defining all message types, wire
formats, serialization, parsing, and package versioning for the Sigyn PC-to-Teensy
serial protocol  
**See also:**
- `docs/specs/wr_ros_teensy.spec.md` — PC-side usage
- `docs/specs/wr_teensy_boards.spec.md` — Teensy-side usage

---

## Table of Contents

1. [Purpose and Design Constraints](#1-purpose-and-design-constraints)
2. [Package Versioning](#2-package-versioning)
3. [Wire Format](#3-wire-format)
4. [Message Type Catalog](#4-message-type-catalog)
5. [ParsedMessage and MessageParser](#5-parsedmessage-and-messageparserparser)
6. [CommandFactory (PC → Teensy)](#6-commandfactory-pc--teensy)
7. [ResponseFactory (Teensy → PC)](#7-responsefactory-teensy--pc)
8. [Fault Model in Messages](#8-fault-model-in-messages)
9. [Build System Integration](#9-build-system-integration)
10. [Test Suite Requirements](#10-test-suite-requirements)
11. [Open Questions](#11-open-questions)

---

## 1. Purpose and Design Constraints

### 1.1 Purpose

`wr_proto_msgs` is the single authoritative definition of every message exchanged
between the Sigyn PC (running ROS 2) and the Teensy boards over USB serial. Both
sides are compiled from the same source, so format drift between them is impossible.

### 1.2 Key design constraints

| Constraint                              | Reason                                                  |
| --------------------------------------- | ------------------------------------------------------- |
| **No ROS 2 dependencies**               | Must compile on Teensy (Arduino/PlatformIO)             |
| **No Arduino/PlatformIO dependencies**  | Must compile in a Linux/ROS 2 build                     |
| **No heap allocation in embedded path** | Teensy firmware forbids `new`/`malloc`                  |
| **No `std::string` in embedded path**   | Use fixed-size `char[]` buffers                         |
| **No `std::vector` in embedded path**   | Fixed-size arrays only                                  |
| **C++17 standard**                      | Both GCC for Linux and GCC-arm for Teensy support C++17 |
| **Single header set**                   | The same headers compile cleanly on both targets        |
| **JSON payload**                        | Human-readable; debuggable with a serial monitor        |
| **No external JSON library dependency** | Use a bundled single-header JSON parser                 |

### 1.3 Platform abstraction

The package handles the two environments with preprocessor guards:

```cpp
#ifdef ARDUINO
  // Teensy-specific includes and type aliases
  #include <Arduino.h>
  using WrString = FixedString<256>;
#else
  // Linux/ROS 2 side
  #include <string>
  using WrString = std::string;
#endif
```

The embedded code paths never allocate; they write into pre-allocated buffers that
are passed in by the caller. All `Format*()` methods take a `char* buf, size_t len`
output parameter pair on the embedded side. The PC side uses `std::string` return
values.

---

## 2. Package Versioning

### 2.1 Version string

The package version is a semantic version string embedded as a compile-time
constant in the library header:

```cpp
// In wr_proto_msgs/version.h
#define WR_PROTO_MSGS_VERSION "1.0.0"
#define WR_PROTO_MSGS_COMPILED_AT __TIMESTAMP__
```

Both `wr_ros_teensy` and `wr_teensy_boards` include this header. At the protocol
handshake, each side sends its `WR_PROTO_MSGS_VERSION` and `WR_PROTO_MSGS_COMPILED_AT`
(the`__TIMESTAMP__` of the compile that produced the binary). The peer verifies
that the version strings match before allowing any data messages.

### 2.2 Version compatibility rules

- Versions must match exactly (major, minor, and patch) for protocol negotiation
  to succeed. There is no "compatible range" interpretation.
- If versions do not match, the robot enters safe mode (see `wr_ros_teensy.spec.md`
  Section 6.1). No motion commands are issued.
- The package version is incremented whenever any message format changes — new
  fields, removed fields, changed field semantics, or changed key names.

### 2.3 Compilation timestamp logging

Both sides log their `WR_PROTO_MSGS_COMPILED_AT` value at startup and in the
protocol handshake. This appears in:
- PC safety event log (JSON header entry at node startup).
- Teensy SD log (one log line at startup: `STARTUP:{"pkg_ver":"1.0.0","pkg_comp":"...",
  "fw_comp":"..."}`).
- The `PROTO_ANNOUNCE` and `PROTO_ACK` messages (see Section 4.1).

When examining post-event logs, the compilation timestamp identifies exactly which
build was running — even if the version string has not changed since the last build.

---

## 3. Wire Format

### 3.1 Line format

Each message is a single line:

```
<TYPE><BOARD_ID>:<JSON_PAYLOAD>\n
```

- `<TYPE>`: a short uppercase ASCII message type token (see catalog in Section 4).
- `<BOARD_ID>`: a single digit (1, 2, or 3) identifying the source (Teensy) or
  destination (PC command) board.
- `:` separator.
- `<JSON_PAYLOAD>`: a valid JSON object, no embedded newlines.
- `\n` line terminator.

Examples:
```
TEMP1:{"inst":0,"v":42.1,"t":12345}
ODOM1:{"lx":0.502,"az":-0.012,"px":1.204,"py":0.301,"ph":0.031,"t":98765}
FLT1:{"fid":"VL53L0X_RING3","inst":4,"sev":3,"rha":true,"rsn":"front_left_fwd @ 0.18m","t":12345}
```

### 3.2 Key length conventions

| Category                      | Key length     | Examples                                |
| ----------------------------- | -------------- | --------------------------------------- |
| Frequent sensor data (≥ 1 Hz) | ≤ 3 characters | `v`, `lx`, `az`, `px`, `t`, `inst`      |
| Moderate-frequency status     | ≤ 6 characters | `state`, `mode`, `pos`, `spd`           |
| Event / fault messages (rare) | Unrestricted   | `fault_id`, `description`, `cleared_by` |
| Protocol handshake            | Unrestricted   | `pkg_ver`, `proto_comp`, `fw_comp`      |

**Rationale:** A message like `TEMP1:{"inst":0,"v":42.1,"t":12345}` is 40 bytes.
At 921600 baud and 8 sensors at 1 Hz, this is < 0.1% of bandwidth. Fully spelled-out
keys like `"temperature_celsius"` would be 5× larger for no benefit at that rate.
At 50 Hz for odometry, short keys matter more.

### 3.3 Required fields in every message

Every message JSON payload includes:
- `t`: Teensy `millis()` timestamp at the time the message was composed (uint32).
  For PC-originated messages, `t` is omitted (PC time is handled separately via
  the `PC_TIME` message).

For sensor messages, `inst` (instance number, uint8) is always present so the
receiver can look up the sensor name from the name table.

### 3.4 Interrupted stream recovery

The parser must recover gracefully from partial messages:
- If a line does not end with `\n` within `kMessageTimeoutMs` (configurable,
  default 100 ms), discard all buffered bytes for that message and start fresh.
- If a line does not begin with a recognized TYPE prefix after `:`, discard and
  log a malformed-message event.
- If a line's JSON does not parse, discard and increment the malformed counter.
- After recovery, the next well-formed complete line must parse correctly.

These recovery paths are tested in the unit test suite (Section 10.3).

---

## 4. Message Type Catalog

### 4.1 Protocol handshake messages

#### PROTO_ANNOUNCE (Teensy → PC, on connect)
```
PROTO<N>:{"pkg_ver":"1.0.0","proto_comp":"2026-03-01T14:22:00Z",
           "board_id":N,"fw_comp":"2026-03-01T15:00:00Z"}
```
| Field        | Type   | Description                                     |
| ------------ | ------ | ----------------------------------------------- |
| `pkg_ver`    | string | `WR_PROTO_MSGS_VERSION` compiled into firmware  |
| `proto_comp` | string | `WR_PROTO_MSGS_COMPILED_AT` from firmware build |
| `board_id`   | uint8  | Board number (1, 2, or 3)                       |
| `fw_comp`    | string | Firmware binary compilation timestamp           |

#### PROTO_REQ (PC → Teensy, solicits announcement)
```
PROTO_REQ<N>:{}
```

#### PROTO_ACK (PC → Teensy, acknowledges matching version)
```
PROTO_ACK<N>:{"pkg_ver":"1.0.0","pc_comp":"2026-03-01T14:22:00Z"}
```

### 4.2 Heartbeat messages

#### HEARTBEAT (Teensy → PC, 2 Hz)
```
HB<N>:{"t":12345}
```

#### HEARTBEAT_PC (PC → all Teensy boards, 2 Hz)
```
HB_PC<N>:{"ts_lo":2147483648}
```
`ts_lo` is the lower 32 bits of the ROS nanosecond timestamp. The Teensy uses
this to loosely correlate PC time with its own `millis()` for log annotations.

### 4.3 Time synchronization

#### PC_TIME (PC → Teensy, once per connection)
```
PC_TIME<N>:{"sec":1772000000,"nsec":123456789}
```
Fields: `sec` (Unix epoch seconds), `nsec` (additional nanoseconds). Sent once
on first connection after PROTO_ACK. The Teensy:
1. Records the offset between `millis()` and the provided wall time.
2. Uses this offset to annotate all subsequent SD log entries with estimated
   wall-clock time.
3. If supported by the SD library, sets the modification time of the current log
   file to the provided wall time (done once only).

### 4.4 Odometry (Teensy → PC, 50 Hz)

Each odometry message is a single update:
```
ODOM1:{"lx":0.502,"az":-0.012,"px":1.204,"py":0.301,"ph":0.031,"t":98765}
```
| Field                   | Key  | Type   | Unit  |
| ----------------------- | ---- | ------ | ----- |
| Linear velocity X       | `lx` | float  | m/s   |
| Angular velocity Z      | `az` | float  | rad/s |
| Position X (integrated) | `px` | float  | m     |
| Position Y (integrated) | `py` | float  | m     |
| Heading (integrated)    | `ph` | float  | rad   |
| Teensy millis           | `t`  | uint32 | ms    |

### 4.5 IMU (Teensy → PC, 100 Hz)

```
IMU2:{"inst":0,"qw":0.9990,"qx":0.0012,"qy":0.0031,"qz":0.0009,
      "gx":-0.002,"gy":0.001,"gz":0.000,"ax":0.01,"ay":-0.02,"az":9.81,"t":12345}
```
| Field                  | Key                 | Type   | Unit  |
| ---------------------- | ------------------- | ------ | ----- |
| Instance               | `inst`              | uint8  | —     |
| Orientation quaternion | `qw`,`qx`,`qy`,`qz` | float  | —     |
| Angular velocity       | `gx`,`gy`,`gz`      | float  | rad/s |
| Linear acceleration    | `ax`,`ay`,`az`      | float  | m/s²  |
| Teensy millis          | `t`                 | uint32 | ms    |

### 4.6 Temperature (Teensy → PC, 1 Hz per sensor)

One message per sensor instance:
```
TEMP1:{"inst":0,"v":42.1,"t":12345}
TEMP1:{"inst":1,"v":44.8,"t":12346}
TEMP1:{"inst":2,"v":38.2,"t":12347}
```
| Field         | Key    | Type   | Unit |
| ------------- | ------ | ------ | ---- |
| Instance      | `inst` | uint8  | —    |
| Temperature   | `v`    | float  | °C   |
| Teensy millis | `t`    | uint32 | ms   |

### 4.7 Battery (Teensy → PC, 1 Hz)

```
BATT2:{"inst":0,"v":24.51,"c":3.22,"soc":0.82,"t":12345}
```
| Field           | Key    | Type   | Unit                       |
| --------------- | ------ | ------ | -------------------------- |
| Instance        | `inst` | uint8  | —                          |
| Voltage         | `v`    | float  | V                          |
| Current         | `c`    | float  | A (positive = discharging) |
| State of charge | `soc`  | float  | 0.0–1.0                    |
| Teensy millis   | `t`    | uint32 | ms                         |

### 4.8 Power rail (Teensy → PC, 1 Hz per rail)

```
RAIL2:{"inst":0,"v":5.02,"c":0.82,"t":12345}
RAIL2:{"inst":1,"v":12.01,"c":1.54,"t":12346}
```
Same structure as BATT; `inst` identifies the rail.

### 4.9 Proximity / Time-of-Flight (Teensy → PC, per-sensor rate)

One message per sensor per measurement cycle:
```
PROX1:{"inst":0,"d":0.45,"t":12345}
```
| Field         | Key    | Type   | Unit |
| ------------- | ------ | ------ | ---- |
| Instance      | `inst` | uint8  | —    |
| Distance      | `d`    | float  | m    |
| Teensy millis | `t`    | uint32 | ms   |

### 4.10 RoboClaw status (Teensy → PC, 1 Hz)

```
RCLW1:{"la":100,"ra":100,"li":2.31,"ri":2.15,"lt":42,"rt":44,
        "err":0,"t":12345}
```
| Field         | Key   | Type   | Description            |
| ------------- | ----- | ------ | ---------------------- |
| Left speed    | `la`  | int16  | Encoder counts/s (raw) |
| Right speed   | `ra`  | int16  | Encoder counts/s (raw) |
| Left current  | `li`  | float  | Amps                   |
| Right current | `ri`  | float  | Amps                   |
| Left temp     | `lt`  | float  | °C                     |
| Right temp    | `rt`  | float  | °C                     |
| Error flags   | `err` | uint16 | RoboClaw error bitmask |
| Teensy millis | `t`   | uint32 | ms                     |

### 4.11 Stepper status (Teensy → PC, on change)

```
STPR3:{"inst":0,"pos":1200,"spd":0,"homed":true,"t":12345}
```
| Field         | Key     | Type   | Description                 |
| ------------- | ------- | ------ | --------------------------- |
| Instance      | `inst`  | uint8  | 0=elevator, 1=gripper       |
| Position      | `pos`   | int32  | Steps from home             |
| Speed         | `spd`   | int16  | Steps/s (0 = stopped)       |
| Homed         | `homed` | bool   | True if home position known |
| Teensy millis | `t`     | uint32 | ms                          |

### 4.12 Performance report (Teensy → PC, 0.2 Hz)

```
PERF1:{"hz":150,"max_us":6542,"avg_us":6201,"t":12345}
```
| Field             | Key      | Type   | Description                 |
| ----------------- | -------- | ------ | --------------------------- |
| Loop rate         | `hz`     | uint16 | Main loop Hz (last window)  |
| Max loop time     | `max_us` | uint32 | Worst case loop duration µs |
| Average loop time | `avg_us` | uint32 | Average loop duration µs    |
| Teensy millis     | `t`      | uint32 | ms                          |

### 4.13 Diagnostics (Teensy → PC, 1 Hz or on event)

```
DIAG1:{"mod":"VL53L0X","inst":3,"sev":"WARN",
        "msg":"sensor read timeout x3","t":12345}
```
Verbose keys allowed; this is a human-readable status message.

### 4.14 Fault events (Teensy → PC, on event)

#### FAULT activation
```
FLT1:{"fid":"VL53L0X_RING3","inst":4,"sev":3,"ugc":0,
       "ac":false,"rha":true,"ltch":false,
       "rsn":"front_left_fwd @ 0.18m, threshold 0.30m","t":12345}
```
| Field                 | Key    | Type   | Description                                      |
| --------------------- | ------ | ------ | ------------------------------------------------ |
| Fault ID              | `fid`  | string | Stable fault class identifier                    |
| Instance              | `inst` | uint8  | Sensor/module instance                           |
| Severity              | `sev`  | uint8  | 0=WARNING 1=DEGRADED 2=EMERGENCY_STOP 3=SHUTDOWN |
| Urgency class         | `ugc`  | uint8  | See safety_system.spec.md                        |
| Auto-clear            | `ac`   | bool   | True if fault clears without human               |
| Requires human ack    | `rha`  | bool   |                                                  |
| Latching              | `ltch` | bool   |                                                  |
| Human-readable reason | `rsn`  | string | Verbose; includes measurement values             |
| Teensy millis         | `t`    | uint32 | ms                                               |

#### FAULT_CLEAR (Teensy → PC, fault self-resolved or board confirmed clear)
```
FLT_CLR1:{"fid":"VL53L0X_RING3","inst":4,"t":23456}
```

#### FAULT_CLEAR_CMD (PC → Teensy, human-authorized clear)
```
FLT_CLR_CMD1:{"fid":"VL53L0X_RING3","inst":4,"by":"wim","why":"obstacle_removed"}
```
(`t` omitted on PC-originated messages.)

#### FAULT_ENUM_REQ (PC → Teensy)
```
FLT_ENM_REQ1:{}
```

#### FAULT_ENUM (Teensy → PC, response to FAULT_ENUM_REQ)
```
FLT_ENM1:{"faults":[
  {"fid":"VL53L0X_RING3","inst":4,"sev":2,"active":true,"rsn":"..."},
  ...
],"t":12345}
```

### 4.15 Motion commands (PC → Teensy)

#### TWIST
```
TWIST1:{"lx":0.50,"az":-0.10}
```
The Teensy applies this as the current motor velocity setpoint.

#### ESTOP_SET
```
ESTOP_SET1:{"src":"PC_SAFETY","rsn":"ProximityRing3_board1_inst4"}
```

#### ESTOP_CLEAR
```
ESTOP_CLR1:{"rsn":"all_faults_cleared"}
```

### 4.16 Configuration commands (PC → Teensy)

```
CFG_SET1:{"key":"wheel_diam_m","val":"0.2150"}
CFG_GET1:{"key":"wheel_diam_m"}
CFG_RST1:{}
```

Configuration acknowledgment (Teensy → PC):
```
CFG_ACK1:{"key":"wheel_diam_m","val":"0.2150","ok":true,"t":12345}
```

### 4.17 Parameter dump (Teensy → PC)

On connect (after PROTO_ACK) and in response to `PRM_REQ`:
```
PRM1:{"params":[
  {"k":"wheel_diam_m","v":"0.2150"},
  {"k":"ring2_dist_m","v":"0.300"},
  {"k":"max_motor_temp_c","v":"75.0"}
],"t":12345}
```
The parameter dump is split into multiple messages if needed to stay within the
line buffer size limit (`kMaxLineBytes`, target 512 bytes on Teensy).

### 4.19 Behavior tree action name (PC → Teensy)

```
BT_ACT1:{"name":"NavigateToPose","rslt":"SUCCESS"}
```
`rslt` is `"SUCCESS"`, `"FAILURE"`, or `"RUNNING"` (first tick only if sent on
first tick; see OQ-STT-6).

---

## 5. ParsedMessage and MessageParser

### 5.1 ParsedMessage struct

```cpp
namespace wr_proto_msgs {

enum class Severity : uint8_t {
  WARNING = 0, DEGRADED = 1, EMERGENCY_STOP = 2, SHUTDOWN = 3
};

struct ParsedMessage {
  // Common header — always present after successful parse
  bool        valid;          // false if parsing failed
  char        type[16];       // e.g., "TEMP", "FLT", "ODOM"
  uint8_t     board_id;       // 1, 2, or 3
  uint32_t    teensy_millis;  // "t" field; 0 for PC-originated

  // Error info — only set when !valid
  char        parse_error[128];

  // Payload union — only the relevant variant is populated
  // The type string indicates which variant to access.
  union {
    struct OdomPayload    { float lx, az, px, py, ph; } odom;
    struct TempPayload    { uint8_t inst; float value; } temp;
    struct BattPayload    { uint8_t inst; float voltage, current, soc; } batt;
    struct RailPayload    { uint8_t inst; float voltage, current; } rail;
    struct ProxPayload    { uint8_t inst; float distance; } prox;
    struct ImuPayload     { uint8_t inst;
                            float qw, qx, qy, qz;
                            float gx, gy, gz;
                            float ax, ay, az; } imu;
    struct FaultPayload   { char fid[32]; uint8_t inst;
                            Severity severity; uint8_t urgency;
                            bool auto_clear, rha, latching;
                            char reason[128]; } fault;
    struct FaultClrPayload{ char fid[32]; uint8_t inst; } fault_clr;
    struct ProtoPayload   { char pkg_ver[16]; char proto_comp[32];
                            char fw_comp[32]; } proto;
    struct HbPayload      { uint32_t ts_lo; } hb;
    struct PerfPayload    { uint16_t hz; uint32_t max_us, avg_us; } perf;
    struct DiagPayload    { char module[16]; uint8_t inst;
                            uint8_t sev; char msg[192]; } diag;
    struct CfgAckPayload  { char key[32]; char val[64]; bool ok; } cfg_ack;
    struct StepperPayload { uint8_t inst; int32_t pos;
                            int16_t speed; bool homed; } stepper;
    // ... additional payload types
  } payload;
};

} // namespace wr_proto_msgs
```

The union avoids heap allocation. The caller checks `msg.type` to know which
payload variant is valid.

### 5.2 MessageParser class

```cpp
namespace wr_proto_msgs {

class MessageParser {
 public:
  /// Parse a null-terminated line (without the trailing newline).
  /// Completely stateless; may be called from any thread.
  /// On the PC side, pass the line string directly.
  /// On the Teensy side, pass a fixed-size line buffer.
  static ParsedMessage parse(const char* line, size_t len);

  /// Convenience overload for PC side only (std::string available)
#ifndef ARDUINO
  static ParsedMessage parse(const std::string& line);
#endif
};

} // namespace wr_proto_msgs
```

**Parser contract:**
- Returns `ParsedMessage.valid = true` on success; otherwise `valid = false` with
  `parse_error` set.
- Allocates nothing. The returned `ParsedMessage` is stack-allocated.
- Handles: type prefix parsing, board ID extraction, JSON field extraction, field
  range validation.
- Detects malformed JSON, missing required fields, out-of-range field values.
- Is stateless: safe to call from multiple threads simultaneously.

---

## 6. CommandFactory (PC → Teensy)

```cpp
namespace wr_proto_msgs {

class CommandFactory {
 public:
  // --- Motion ---
  static std::string twist(float linear_x, float angular_z);
  static std::string estopSet(uint8_t board_id,
                               const std::string& source,
                               const std::string& reason);
  static std::string estopClear(uint8_t board_id, const std::string& reason);

  // --- Protocol ---
  static std::string protoReq(uint8_t board_id);
  static std::string protoAck(uint8_t board_id);

  // --- Heartbeat ---
  static std::string pcHeartbeat(uint8_t board_id, uint32_t ts_lo);

  // --- Time sync ---
  static std::string pcTime(uint8_t board_id,
                             int64_t unix_sec, uint32_t nsec);

  // --- Configuration ---
  static std::string configSet(uint8_t board_id,
                                const std::string& key,
                                const std::string& value);
  static std::string configGet(uint8_t board_id, const std::string& key);
  static std::string configReset(uint8_t board_id);

  // --- Fault management ---
  static std::string faultClearCmd(uint8_t board_id,
                                    const std::string& fault_id,
                                    uint8_t instance,
                                    const std::string& cleared_by,
                                    const std::string& reason);
  static std::string faultEnumReq(uint8_t board_id);

  // --- SD card ---
  static std::string sdListReq(uint8_t board_id);
  static std::string sdDeleteReq(uint8_t board_id,
                                  const std::string& filename);

  // --- Parameter dump ---
  static std::string paramDumpReq(uint8_t board_id);

  // --- Behavior tree ---
  static std::string btAction(uint8_t board_id,
                               const std::string& node_name,
                               const std::string& result);
};

} // namespace wr_proto_msgs
```

All methods are `static`. The class has no state. All methods return `std::string`
on the PC side; on the Teensy side, variants that accept `char* buf, size_t len`
output parameters are provided.

Round-trip test requirement: the output of every `CommandFactory` method must be
parseable by `MessageParser::parse()` and round-trip cleanly.

---

## 7. ResponseFactory (Teensy → PC)

On the Teensy side, the firmware uses a `ResponseFactory` to construct outbound
messages. This class mirrors `CommandFactory` but builds Teensy-originated messages.
It uses fixed-size char buffers, no heap allocation.

```cpp
namespace wr_proto_msgs {

class ResponseFactory {
 public:
  // All methods write into caller-provided buffer and return bytes written.
  static size_t protoAnnounce(char* buf, size_t len,
                               uint8_t board_id, uint32_t millis_now);

  static size_t heartbeat(char* buf, size_t len,
                           uint8_t board_id, uint32_t millis_now);

  static size_t odom(char* buf, size_t len, uint8_t board_id,
                      float lx, float az, float px, float py, float ph,
                      uint32_t millis_now);

  static size_t temperature(char* buf, size_t len,
                              uint8_t board_id, uint8_t instance,
                              float celsius, uint32_t millis_now);

  static size_t proximity(char* buf, size_t len,
                           uint8_t board_id, uint8_t instance,
                           float distance_m, uint32_t millis_now);

  static size_t imu(char* buf, size_t len, uint8_t board_id, uint8_t instance,
                     float qw, float qx, float qy, float qz,
                     float gx, float gy, float gz,
                     float ax, float ay, float az,
                     uint32_t millis_now);

  static size_t battery(char* buf, size_t len, uint8_t board_id,
                         uint8_t instance, float voltage, float current,
                         float soc, uint32_t millis_now);

  static size_t powerRail(char* buf, size_t len, uint8_t board_id,
                           uint8_t instance, float voltage, float current,
                           uint32_t millis_now);

  static size_t fault(char* buf, size_t len, uint8_t board_id,
                       const char* fault_id, uint8_t instance,
                       uint8_t severity, uint8_t urgency,
                       bool auto_clear, bool rha, bool latching,
                       const char* reason, uint32_t millis_now);

  static size_t faultClear(char* buf, size_t len, uint8_t board_id,
                             const char* fault_id, uint8_t instance,
                             uint32_t millis_now);

  static size_t performance(char* buf, size_t len, uint8_t board_id,
                              uint16_t hz, uint32_t max_us, uint32_t avg_us,
                              uint32_t millis_now);

  static size_t diagMessage(char* buf, size_t len, uint8_t board_id,
                              const char* module, uint8_t instance,
                              uint8_t severity, const char* msg,
                              uint32_t millis_now);

  static size_t configAck(char* buf, size_t len, uint8_t board_id,
                           const char* key, const char* val, bool ok,
                           uint32_t millis_now);

  static size_t stepperStatus(char* buf, size_t len, uint8_t board_id,
                                uint8_t instance, int32_t pos, int16_t speed,
                                bool homed, uint32_t millis_now);
};

} // namespace wr_proto_msgs
```

---

## 8. Fault Model in Messages

### 8.1 Fault IDs

Fault IDs are short stable strings defined in `wr_proto_msgs/fault_ids.h`. They
are compile-time constants used on both sides:

```cpp
// wr_proto_msgs/fault_ids.h
namespace wr_proto_msgs {
namespace FaultId {
  constexpr const char* VL53L0X_RING3      = "VL53L0X_RING3";
  constexpr const char* VL53L0X_RING2      = "VL53L0X_RING2";
  constexpr const char* MOTOR_OVERTEMP     = "MOTOR_OVERTEMP";
  constexpr const char* BATTERY_CRITICAL   = "BATT_CRIT";
  constexpr const char* BATTERY_LOW        = "BATT_LOW";
  constexpr const char* SPIN_IN_PLACE      = "SPIN_PLACE";
  constexpr const char* PC_LOST            = "PC_LOST";
  constexpr const char* BOARD_LOST         = "BOARD_LOST";
  constexpr const char* PROTO_VERSION_MISMATCH = "PROTO_VER_MISMATCH";
  constexpr const char* ROBOCLAW_ERROR     = "RCLW_ERR";
  // ... extended as new fault types are defined
}
} // namespace wr_proto_msgs
```

Using constants on both sides ensures that a fault ID string from a Teensy message
can be matched without string manipulation on the PC side.

### 8.2 Severity mapping

```cpp
namespace wr_proto_msgs {
enum class Severity : uint8_t {
  WARNING        = 0,  // Informational; robot still fully operational
  DEGRADED       = 1,  // Reduced capability; velocity limited
  EMERGENCY_STOP = 2,  // Motion stopped; human notification required
  SHUTDOWN       = 3   // System must be powered down
};
} // namespace wr_proto_msgs
```

---

## 9. Build System Integration

### 9.1 PC side (ROS 2 / CMake)

`wr_proto_msgs` is a separate ament_cmake package providing a CMake interface
library. Downstream packages include it with:

```cmake
find_package(wr_proto_msgs REQUIRED)
target_link_libraries(my_target wr_proto_msgs::wr_proto_msgs)
```

The library is header-only for most paths; a small static library provides the
`parse()` and `format()` implementations using the bundled JSON engine.

### 9.2 Teensy side (PlatformIO)

The package is included as a PlatformIO library dependency. The library path is
added to `platformio.ini`:

```ini
lib_deps =
    file://../../wr_proto_msgs
```

On the embedded side, the library is compiled with `-DARDUINO` defined, which
activates the fixed-buffer paths and suppresses `std::string`.

### 9.3 JSON engine

The bundled, single-header JSON engine is [ArduinoJson](https://arduinojson.org/)
which:
- Compiles on both Linux and Teensy.
- Supports both dynamic (PC side) and static/stack-allocated (Teensy side) modes.
- Is MIT licensed.
- Has no system dependencies.

`StaticJsonDocument<N>` is used on the Teensy side (N = 512 bytes default).
`DynamicJsonDocument` is used on the PC side. The choice is made via the `ARDUINO`
preprocessor flag.

---

## 10. Test Suite Requirements

### 10.1 Test framework

The `wr_proto_msgs` test suite uses Google Test, runs on Linux (no hardware), and is
part of the ROS 2 `ament_cmake_gtest` infrastructure. It must also be runnable as a
standalone CMake project to allow testing outside a ROS 2 workspace.

### 10.2 Coverage targets

All of the following must have passing tests before the package version is considered
stable:

- `MessageParser::parse()`: round-trip for every message type in Section 4.
- `CommandFactory`: every method produces output that parses correctly with
  `MessageParser`.
- `ResponseFactory`: every method produces output that parses correctly.
- Version extraction from `PROTO_ANNOUNCE`.

### 10.3 Fault-tolerance tests (critical)

These test the parser's robustness under real-world USB serial failure conditions:

| Test case                                         | Expected behavior                                               |
| ------------------------------------------------- | --------------------------------------------------------------- |
| Partial line followed by timeout                  | First partial is discarded; next complete line parses correctly |
| Two complete lines concatenated (missing newline) | First line parses; remainder treated as new partial             |
| Valid prefix + corrupt JSON                       | `valid=false`; malformed counter incremented                    |
| Extra unknown JSON fields                         | Parsed successfully (unknown fields ignored)                    |
| Missing required field (`t` absent)               | `valid=false`; parse_error set                                  |
| Field value out of range (e.g., `soc: 1.5`)       | `valid=false`; specific error set                               |
| Empty line                                        | `valid=false`; discarded                                        |
| Line with only type prefix, no `:` or JSON        | `valid=false`                                                   |
| Line too long (> `kMaxLineBytes`)                 | Discarded with error; parser resets                             |
| Board ID `0` or `>3`                              | `valid=false`                                                   |
| Correct data after a series of corrupt lines      | Parser recovers; correct data parses                            |

### 10.4 Bandwidth budget test

A test that generates 1000 messages of each frequent type (TEMP, PROX, IMU, ODOM,
BATT) and verifies the average byte count per message is within the budget defined
in Section 3.2. This is a regression test — a change that significantly increases
message size for frequent messages should fail this test.

### 10.5 Cross-compilation smoke test

A build smoke test verifies that the library compiles without error with the Teensy
GCC arm cross-compiler (`arm-none-eabi-g++ -DARDUINO`). This runs as part of CI
via PlatformIO's `native` environment.

---

## 11. Open Questions (Resolved)

| ID      | Question                                                | Resolution                                                                                                                          |
| ------- | ------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| OQ-PM-1 | `ParsedMessage` tagged union vs `std::variant` on PC?   | **C++17 `std::variant`** is used cleanly on both platforms, maintaining strict type safety over `memcpy`.                           |
| OQ-PM-2 | ArduinoJson vendored vs PlatformIO dependency?          | **Vendored v7.4.3**. Guaranteeing exactly identical builds for test suite on ROS vs. Teensy.                                        |
| OQ-PM-3 | FAULT_ENUM as individual vs array?                      | **Individual `FLT_ENM_ITEM` + `FLT_ENM_DONE`**. Keeps JSON payload size strictly bounded and parsing loop generic.                  |
| OQ-PM-4 | `RoboClawStatus` single vs split (`RCLW_L` / `RCLW_R`)? | **Combined** historically, but can be evaluated as we refactor logic in `wr_ros_teensy`. *(Pending final analysis of motor module)* |
| OQ-PM-5 | `PARAM_DUMP` large stream handling?                     | Like faults, can use a `_DONE` marker if required.                                                                                  |
