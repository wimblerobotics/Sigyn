# wr_ros_teensy Package Specification

**Document status:** Draft v0.2  
**Last updated:** 2026-03-03  
**Scope:** The `wr_ros_teensy` ROS 2 package — the PC-side communication bridge,
safety coordinator, and ROS 2 interface layer for all Teensy boards  
**Supersedes:** `sigyn_to_teensy.spec.md` v0.1 (2026-03-02)  
**See also:**
- `docs/specs/sigyn.spec.md` (overall system specification)
- `docs/specs/safety_system.spec.md` (safety behavior specification)
- `docs/specs/wr_teensy_boards.spec.md` (embedded firmware specification)
- `docs/specs/wr_proto_msgs.spec.md` (shared serial protocol message class)
- `docs/specs/human_interaction.spec.md` (human notification and override)

---

## Table of Contents

1. [Purpose and Motivation](#1-purpose-and-motivation)
2. [Scope and Boundaries](#2-scope-and-boundaries)
3. [Architecture Overview](#3-architecture-overview)
4. [Component Design](#4-component-design)
5. [Threading Model](#5-threading-model)
6. [Message Protocol Handling](#6-message-protocol-handling)
7. [Safety Coordinator](#7-safety-coordinator)
8. [ROS 2 Interfaces](#8-ros-2-interfaces)
9. [Sensor Naming](#9-sensor-naming)
10. [Parameter Management](#10-parameter-management)
11. [Human Notification Integration](#11-human-notification-integration)
12. [Behavior Tree Integration](#12-behavior-tree-integration)
13. [PC-Side Safety Computations](#13-pc-side-safety-computations)
14. [Logging and Audit Trail](#14-logging-and-audit-trail)
15. [Testing Requirements](#15-testing-requirements)
16. [Implementation Notes](#16-implementation-notes)

---

## 1. Purpose and Motivation

`wr_ros_teensy` is the authoritative boundary between the real-time embedded world
of the Teensy boards and the deliberative world of ROS 2. It has three distinct
roles that must not be conflated:

1. **Serial bridge:** Translate bytes on USB serial ports into ROS 2 messages and
   back. Purely mechanical transformation; contains no policy.

2. **Safety coordinator:** Maintain the authoritative PC-side safety state, enforce
   the authority chain defined in `safety_system.spec.md` Section 6, and act as the
   single point through which safety commands flow from the deliberative layer to
   the Teensy boards.

3. **ROS 2 interface layer:** Publish well-typed, properly timestamped sensor data
   to topics that navigation, localization, behavior trees, and diagnostics consume.

**The critical design invariant:** The serial bridge component must never make a
safety decision. The safety coordinator must never parse serial bytes. Each component
has one job.

### 1.1 Relationship to other packages in this workspace

This workspace (`sigyn_control`) contains three related packages:

| Package            | Content                                             | Depends on      |
| ------------------ | --------------------------------------------------- | --------------- |
| `wr_ros_teensy`    | This package: PC-side ROS 2 node                    | `wr_proto_msgs` |
| `wr_teensy_boards` | Teensy firmware (PlatformIO, C++)                   | `wr_proto_msgs` |
| `wr_proto_msgs`    | Shared serial protocol message class (portable C++) | nothing         |

`wr_proto_msgs` is the shared contract between the two sides. It contains no
ROS 2 dependencies, no Arduino dependencies — pure C++17 that compiles on both.

### 1.2 Simulation mode

When running in simulation (`use_sim_time: true`), this package operates without
real-time thread priorities, without real serial ports, and without physical Teensy
boards. No fault is activated for absent boards in simulation mode. All timing
parameters (heartbeat, reconnect, velocity limits) use configured defaults but have
no safety implications in simulation.

### 1.3 Compilation time logging

Both sides of the system — `wr_ros_teensy` on the PC and firmware on each Teensy
board — shall log, at startup and in connection handshake messages, the compilation
timestamp of the `wr_proto_msgs` package they were built against. This appears as a
field in the protocol announcement and is written to both the PC safety event log
and the Teensy SD log. This makes it easy to correlate post-event log files with the
exact protocol version in use at the time.

---

## 2. Scope and Boundaries

### 2.1 What this package owns

- Serial port management for all three Teensy boards.
- Parsing and validation of all inbound Teensy messages (via `wr_proto_msgs`).
- Serialization and transmission of all outbound commands (via `wr_proto_msgs`).
- PC-side safety state machine (aggregating Teensy faults + PC-computed faults).
- Publication of all Teensy-sourced sensor data to ROS 2 topics.
- Subscription to and forwarding of `cmd_vel` to Board 1.
- Heartbeat monitoring for all Teensy boards and sending PC heartbeats to all boards.
- Parameter synchronization between ROS 2 parameter server and Teensy `ConfigStore`.
- Human alert generation (tier 1–4 escalation) as defined in `human_interaction.spec.md`.
- Publishing overall system connectivity state (which boards are connected; see Section 8.1).

### 2.2 What this package does NOT own

- Navigation decisions (Nav2).
- Behavior tree nodes or their logic.
- Sensor drivers that do not live on a Teensy (LiDAR, OAK-D camera).
- SLAM or localization.
- ROS 2 bag file recording. Bag files are managed externally by the operator.
- Distance-to-charger computation (see Section 8.2 and 10.3).

The safety coordinator **receives** inputs from navigation and localization (current
pose, LiDAR-based proximity estimates) but does not make navigation decisions itself.
It influences navigation by publishing velocity constraints and fault events that the
behavior tree reacts to.

---

## 3. Architecture Overview

### 3.1 Component decomposition

```
wr_ros_teensy package
│
├── SerialBridge (per board: up to 3 instances)
│     Owns: fd, read thread, write thread, raw byte queues
│     Output: Queue of complete validated message strings
│     Input: Queue of command strings to transmit
│
├── MessageParser  [uses wr_proto_msgs]
│     Stateless; converts a message string → ParsedMessage struct
│     Has no I/O, no threading, no state
│     Fully unit-testable
│
├── TopicPublisher
│     Receives ParsedMessage structs from dispatcher
│     Converts to ROS 2 message types using sensor name table
│     Publishes to ROS 2 topics
│
├── SafetyCoordinator
│     Maintains FaultRegistry (authoritative PC-side fault state)
│     Computes aggregated safety level
│     Issues safety commands to Teensy boards via CommandQueue
│     Publishes safety state and fault events to ROS 2
│     Subscribes to: twist, LiDAR proximity hints, behavior tree state
│
├── MessageDispatcher
│     Routes ParsedMessages from parser to: TopicPublisher and SafetyCoordinator
│     Applies message priority ordering
│     Stateless except for per-board last-seen timestamps (for heartbeat)
│
├── [wr_proto_msgs::CommandFactory]   [in wr_proto_msgs library]
│     Constructs typed command structs and serializes to wire format strings
│     Also parses inbound messages to typed structs
│     Stateless; used by SafetyCoordinator and the cmd_vel path
│
└── TeensyBridgeNode (rclcpp_lifecycle::LifecycleNode)
      Owns: SerialBridge instances, MessageParser, MessageDispatcher,
            TopicPublisher, SafetyCoordinator
      Provides: constructors for all components with injected dependencies
      Entry point and lifecycle manager
      Lifecycle states: Unconfigured → Configured → Active → Inactive → Finalized
```

**Key change from prior design:** `CommandBuilder` (renamed `CommandFactory`) is
now part of `wr_proto_msgs`, not `wr_ros_teensy`. This ensures both sides have
exactly one definition of every message format — on the PC side it is called as a
library; on the Teensy side the same header is compiled into the firmware.

### 3.2 Lifecycle node states

All nodes in this package are `rclcpp_lifecycle::LifecycleNode` instances. The
state machine maps to Teensy connection events:

| Lifecycle state | Meaning                                                  |
| --------------- | -------------------------------------------------------- |
| `Unconfigured`  | Node created; no serial ports opened                     |
| `Configured`    | Serial port objects created; not yet connected to boards |
| `Active`        | At least Board 1 connected and protocol negotiated       |
| `Inactive`      | Deliberately deactivated (e.g., during firmware upload)  |

Board 2 and Board 3 becoming unavailable do not cause the node to leave `Active`
state; they produce fault events and degraded connectivity reports.

### 3.3 Node structure

A single composable node, `TeensyBridgeNode`, contains all components. A future
split into a separate `SafetyCoordinatorNode` is possible without changing the
component interfaces (see OQ-STT-1).

Node name: `teensy_bridge`. Namespace: `/sigyn`.

---

## 4. Component Design

### 4.1 SerialBridge

One `SerialBridge` instance per Teensy board. Responsibilities:

- Open the serial port at the configured device path and baud rate.
- Maintain a reader thread: continuously calls `read()`, accumulates bytes into a
  line buffer, and on each newline pushes the complete line to a
  `thread_safe_queue<std::string>`.
- Maintain a writer thread that drains a priority queue of outbound strings.
- Reconnect automatically when the port is lost (e.g., USB disconnect).
- Never block the ROS 2 node's main thread; all I/O is in the reader/writer threads.

**Board availability policy:**

- Board 1 is required (except in simulation mode). If Board 1 cannot be opened or
  does not complete protocol negotiation within `startup_timeout_ms`, the node
  activates a `CRITICAL` fault and remains in `Configured` state (not `Active`).
- Boards 2 and 3 are optional. If they are absent, a `WARNING` fault is activated
  and logged. The robot can operate in a degraded mode. Topics for those boards'
  sensors are still advertised; they are published with a staleness flag or simply
  not published until the board connects.

**Error handling:**

- Port cannot be opened at startup: log `CRITICAL`, activate "TeensyNotConnected"
  fault, attempt reconnect every `kReconnectIntervalMs`.
- Port lost during operation: activate the same fault, start reconnect loop.
- Reconnect succeeds: deactivate the fault, trigger protocol renegotiation.

**Interface:**

```cpp
class ISerialBridge {
 public:
  virtual ~ISerialBridge() = default;
  virtual bool send(std::string message, TxPriority priority) = 0;
  virtual std::optional<std::string> receive() = 0;
  virtual bool isConnected() const = 0;
  virtual uint8_t boardId() const = 0;
};
```

The mock `ISerialBridge` allows tests to inject pre-formed message strings and
capture transmitted commands without hardware.

### 4.2 MessageParser

A stateless function object that delegates all JSON parsing and field extraction to
the `wr_proto_msgs` library. It converts a raw message string to a `ParsedMessage`.

```cpp
// ParsedMessage is defined in wr_proto_msgs
// MessageParser is a thin wrapper that stamps received_at and adapts to ROS types.
class MessageParser {
 public:
  wr_proto_msgs::ParsedMessage parse(
      const std::string& raw, rclcpp::Time received_at) const;
};
```

Because the parser is stateless, it is tested directly with known-good and
known-bad input strings. No mock is needed.

### 4.3 MessageDispatcher

Runs in its own thread, draining the `RawMessageQueue` and dispatching to handlers.

```cpp
using MessageHandler = std::function<void(const wr_proto_msgs::ParsedMessage&)>;

class MessageDispatcher {
 public:
  void registerHandler(const std::string& msg_type, MessageHandler handler);
  void dispatch(const wr_proto_msgs::ParsedMessage& msg);
  void updateHeartbeat(uint8_t board_id, rclcpp::Time when);
  bool hasHeartbeatTimedOut(uint8_t board_id, rclcpp::Duration timeout) const;
};
```

Handlers are invoked synchronously in the dispatcher's thread. Handlers for
safety-critical message types (`FAULT`, `ESTOP`) must have bounded execution time
(< 5 ms). Other handlers must not block.

### 4.4 TopicPublisher

Thin translation layer: receives parsed messages, looks up the sensor common name
in the sensor name table (Section 9), and publishes typed ROS 2 messages.

Requirements:
- Each message type has exactly one handler method.
- No safety decisions.
- Timestamps from the injected ROS 2 clock; Teensy `millis()` preserved in header.
- If a board sends data for a sensor not in the name table, log a warning once and
  publish to a fallback topic `sigyn/unknown_sensor/<board>/<raw_id>`.

---

## 5. Threading Model

### 5.1 Thread inventory

| Thread name       | Owner             | Responsibility                                                 |
| ----------------- | ----------------- | -------------------------------------------------------------- |
| `ros_main`        | rclcpp executor   | Handles ROS 2 callbacks (subscriptions, timers, services)      |
| `serial_reader_N` | SerialBridge[N]   | Reads bytes from serial port N; pushes complete lines to queue |
| `serial_writer_N` | SerialBridge[N]   | Drains outbound priority queue; writes to serial port N        |
| `dispatcher`      | MessageDispatcher | Drains inbound message queues; calls handlers                  |

Total thread count: 1 + (2 × boards) + 1 = 8 threads (3 boards).

### 5.2 Thread safety requirements

- `RawMessageQueue` (serial_reader → dispatcher): lock-free single-producer
  single-consumer ring buffer.
- `TxPriorityQueue` (dispatcher/ROS callbacks → serial_writer): mutex-protected
  priority queue. Multiple producers possible.
- `FaultRegistry` (SafetyCoordinator): reads on dispatcher thread; writes on
  dispatcher thread and via ROS 2 service callbacks. Use `shared_mutex`.
- ROS 2 publishers: all calls to `publish()` happen on `ros_main` thread via
  post-to-executor. Publishers are NOT called from the dispatcher thread.

### 5.3 Latency path analysis

```
Teensy firmware detects fault
    → serial_reader_N receives bytes              (≤ 2 ms)
    → RawMessageQueue push                        (< 1 µs)
    → dispatcher processes message                (< 1 ms)
    → SafetyCoordinator::onFaultMessage()         (< 1 ms)
    → publisher->publish() on ros_main            (< 5 ms ros executor latency)
    
Total: < 10 ms (target: < 20 ms per safety_system.spec.md Section 11.4)
```

In **simulation mode**, the dispatcher thread runs at normal priority (not
`SCHED_FIFO`). The RT priority setting is a launch parameter that defaults to
`false` and must be explicitly enabled for hardware deployments.

### 5.4 cmd_vel forwarding latency

cmd_vel commands are P2 priority and must enter the transmit queue within 10 ms of
receipt. cmd_vel does not share a queue backlog with P3/P4 sensor reports.

---

## 6. Message Protocol Handling

### 6.1 Protocol versioning — package-level handshake

Protocol versioning is managed at the **package level**, not per-message type.
There is a single `wr_proto_msgs` package version that governs all message formats
simultaneously. When either side upgrades its message format, the package version
increments. The connection handshake verifies that both sides were compiled against
a compatible package version before any data exchange begins.

**Handshake sequence (per board):**

```
1. SerialBridge opens port; sends PROTO_REQ to Teensy.
2. Teensy responds with PROTO_ANNOUNCE:
       PROTO<N>:{"pkg_ver":"1.3.0","proto_compiled":"2026-03-01T14:22:00Z",
                 "board_id":N,"fw_compiled":"2026-03-01T15:00:00Z"}
3. wr_ros_teensy verifies pkg_ver against its own compiled-in expected version.
4a. Version matches:
       - Both sides log the compilation timestamps.
       - wr_ros_teensy sends PROTO_ACK<N>:{"pkg_ver":"1.3.0","pc_compiled":"2026-03-01T14:22:00Z"}
       - Board N enters normal operating mode.
4b. Version does NOT match:
       - wr_ros_teensy activates a CRITICAL fault ("ProtocolVersionMismatch", board_id=N).
       - The entire robot remains in safe mode: no motion commands are issued,
         all actuators remain stopped.
       - No per-message-type negotiation is attempted.
       - The mismatch, along with both version strings and both compilation
         timestamps, is logged at ERROR level on the PC and written to the
         Teensy SD log.
       - The operator must rebuild both sides from matching wr_proto_msgs source.
```

**Version mismatch behavior:**
- Board 1 version mismatch: full safe mode; robot cannot operate.
- Board 2 or 3 version mismatch: those boards are treated as disconnected; their
  sensors are unavailable; a WARNING fault is active.

**There is no graceful degradation for version mismatches.** Every message type
relies on the package version being compatible. Partial compatibility is not
defined.

### 6.2 Message type catalog

The complete set of message types is defined in `wr_proto_msgs`. They are listed
here for reference; the authoritative definition is in `wr_proto_msgs.spec.md`.

| Direction   | Message type      | Safety-critical | Rate                    |
| ----------- | ----------------- | --------------- | ----------------------- |
| Teensy → PC | `FAULT`           | Yes             | On event                |
| Teensy → PC | `FAULT_CLEAR`     | Yes             | On event                |
| Teensy → PC | `HEARTBEAT`       | Yes             | 2 Hz                    |
| Teensy → PC | `PROTO_ANNOUNCE`  | Yes             | On connect              |
| Teensy → PC | `ODOM`            | No              | 50 Hz                   |
| Teensy → PC | `IMU`             | No              | 100 Hz                  |
| Teensy → PC | `BATT`            | No              | 1 Hz                    |
| Teensy → PC | `POWER_RAIL`      | No              | 1 Hz                    |
| Teensy → PC | `TEMP`            | No              | 1 Hz                    |
| Teensy → PC | `PROXIMITY`       | No              | Per sensor              |
| Teensy → PC | `ROBOCLAW`        | No              | 1 Hz                    |
| Teensy → PC | `STEPPER_STAT`    | No              | On change               |
| Teensy → PC | `PERF`            | No              | 0.2 Hz                  |
| Teensy → PC | `DIAG`            | No              | 1 Hz                    |
| Teensy → PC | `PARAM_DUMP`      | No              | On connect + on request |
| Teensy → PC | `FILE_LIST`       | No              | On request              |
| Teensy → PC | `FAULT_ENUM`      | No              | On request              |
| PC → Teensy | `PROTO_REQ`       | Yes             | On connect              |
| PC → Teensy | `PROTO_ACK`       | Yes             | On connect              |
| PC → Teensy | `TWIST`           | No              | Per cmd_vel             |
| PC → Teensy | `ESTOP_SET`       | Yes             | On event                |
| PC → Teensy | `ESTOP_CLEAR`     | Yes             | On event                |
| PC → Teensy | `FAULT_CLEAR_CMD` | Yes             | On event                |
| PC → Teensy | `HEARTBEAT_PC`    | Yes             | 2 Hz (all boards)       |
| PC → Teensy | `CONFIG_SET`      | No              | On demand               |
| PC → Teensy | `CONFIG_GET`      | No              | On demand               |
| PC → Teensy | `CONFIG_RESET`    | No              | On demand               |
| PC → Teensy | `PARAM_DUMP_REQ`  | No              | On demand               |
| PC → Teensy | `SD_LIST_REQ`     | No              | On demand               |
| PC → Teensy | `SD_DELETE_REQ`   | No              | On demand               |
| PC → Teensy | `FAULT_ENUM_REQ`  | No              | On demand               |
| PC → Teensy | `PC_TIME`         | No              | Once per connection     |
| PC → Teensy | `BT_ACTION`       | No              | On BT result            |

### 6.3 Wire format guidelines

Message format is defined in full in `wr_proto_msgs.spec.md`. The guiding principles:

- **Frequent messages use short keys.** For messages sent ≥ 1 Hz (ODOM, IMU, TEMP,
  BATT, PROXIMITY, etc.), all JSON keys are ≤ 3 characters. A person can still
  decode a message with the protocol document in hand.
- **Error and event messages may use long descriptive strings.** FAULT, DIAG, and
  any message with human-readable description fields may use fully spelled-out keys
  and values. These are rare and their verbosity does not affect serial throughput.
- Each message begins with a type prefix and board ID: `TYPE<N>:`. Example:
  `TEMP1:{"n":"lft_mtr","v":42.1,"t":12345}` where `n` is the sensor name,
  `v` is the value, and `t` is Teensy `millis()`.
- The JSON payload follows on the same line. Each message is terminated by `\n`.

### 6.4 Message field validation

The `wr_proto_msgs` parser validates:
- Required fields are present.
- Field value ranges (negative voltage, physically impossible temperature, etc.).
- UTF-8 validity of string fields.

Validation failures:
- Increment a per-type, per-board malformed message counter.
- Log at `DEBUG` level.
- If malformed rate for a type exceeds `kMaxMalformedRate` (default 5%) over a
  10-second window, activate a `DEGRADED` fault.

---

## 7. Safety Coordinator

### 7.1 Role and authority

The PC-side safety coordinator is authority level 4 in the authority chain
(see `safety_system.spec.md` Section 6). It:

- Aggregates fault events arriving from Teensy boards and from PC-side detectors.
- Maintains the authoritative PC-side fault registry.
- Issues safety commands to Teensy boards (e-stop set/clear, velocity limits).
- Publishes the aggregated safety state to ROS 2.
- Enforces the authority chain: it cannot clear a fault that a Teensy board has
  determined is still active.
- Handles human fault-clearing requests: when a human indicates an underlying
  condition is resolved (e.g., a cord tangle has been cleared), the coordinator
  sends a `FAULT_CLEAR_CMD` to the appropriate board so the Teensy can remove
  that fault from its own registry. The fault remains in the PC fault registry
  as a historical record until the board confirms the clear.

### 7.2 Fault registry

```cpp
struct PcFaultEntry {
  std::string  fault_id;         // Unique, stable string: e.g., "VL53L0X_RING3"
  std::string  source_class;     // e.g., "BATTERY", "VL53L0X", "PC_PROXIMITY"
  uint8_t      board_id;         // 0 = PC-originated
  uint8_t      instance;         // e.g., sensor index; 0 if not applicable
  Severity     severity;
  UrgencyClass urgency;
  bool         auto_clear;
  bool         requires_human_ack;
  bool         latching;
  bool         human_override_active;
  std::string  override_reason;
  std::string  override_by;       // person nickname from people table
  rclcpp::Time activated_at;
  rclcpp::Time deactivated_at;    // Zero if still active
  // Set of all reasons this fault is currently active.
  // E-stop is cleared only when this set is empty.
  std::set<std::string> active_reasons;
};
```

The registry is a fixed-capacity map keyed on `(fault_id, board_id, instance)`.
When a Teensy sends `FAULT` activation, the coordinator inserts or updates the
entry and adds the reason to `active_reasons`. When a Teensy sends `FAULT_CLEAR`,
the coordinator removes that reason. Only when `active_reasons` is empty is the
fault considered deactivated.

**Example:** Three VL53L0X sensors simultaneously signal Ring 3. Three separate
`FAULT` messages arrive (one per sensor), each with a distinct reason/instance.
The fault entry has three entries in `active_reasons`. All three must send
`FAULT_CLEAR` before the fault entry deactivates and e-stop can be cleared.

### 7.3 Aggregated safety level

The aggregated safety level is the maximum severity across all active fault entries
that have not been overridden. This level:

- Is published to `/sigyn/safety/state` at ≥ 10 Hz.
- Governs which velocity limit is sent to Board 1.
- Controls the behavior tree state signal (Section 12).

### 7.4 Velocity limit enforcement

| Condition                 | Max linear velocity   | Max angular velocity     |
| ------------------------- | --------------------- | ------------------------ |
| NORMAL + NOMINAL          | `max_linear_mps`      | `max_angular_radps`      |
| Any DEGRADED fault active | `degraded_linear_mps` | `degraded_angular_radps` |
| Ring 2 occupancy          | `ring2_linear_mps`    | `ring2_angular_radps`    |
| Any EMERGENCY_STOP        | 0.0                   | 0.0                      |

Limits are published to `/sigyn/safety/velocity_limit` and sent to Board 1 as
`CONFIG_SET` ephemeral commands (non-persisted; Board 1 reverts to defaults on
reconnect, and the PC re-sends limits promptly upon any reconnect).

### 7.5 E-stop command flow

E-stop is the *result* of active faults, not a command. The PC does not "command
e-stop" directly — it commands the board to respond to individual faults.

**When a fault at EMERGENCY_STOP severity activates:**
1. The fault entry is added to the registry with the fault's reason.
2. The coordinator sends `ESTOP_SET` to Board 1 (P0 priority).
3. The fault event is published to `/sigyn/safety/fault_events`.
4. The authority chain audit log is updated.

**When a fault clears:**
1. A `FAULT_CLEAR_CMD` is sent to the appropriate board (P1 priority) identifying
   the specific fault (`fault_id`, `instance`, cleared reason).
2. The board removes that reason from its local fault registry and responds with
   `FAULT_CLEAR` confirmation.
3. The PC removes the reason from `active_reasons` in its own registry.
4. If `active_reasons` is now empty for any EMERGENCY_STOP-severity fault, the
   coordinator evaluates whether all EMERGENCY_STOP faults are cleared.
5. When ALL EMERGENCY_STOP-severity faults have empty `active_reasons`,
   `ESTOP_CLEAR` is sent to Board 1.
6. The PC waits for the Board 1 `FAULT_CLEAR` confirmation for its own fault
   entries, but does **not** wait for e-stop to be signaled cleared before
   returning from the service call. E-stop clearing is asynchronous.

**The PC must not assume a command was acted upon.** The only evidence that e-stop
has been cleared is a Board 1 message indicating normal motor operation.

### 7.6 Human override handling

When an override request arrives (via `/sigyn/safety/human_override` service):
1. Validate that the fault allows override (per-fault config).
2. Validate the reason provided against the permitted list for that fault.
3. Validate the person identity against the people table (Section 11.2).
4. Set `human_override_active = true`; record `override_by` (person nickname)
   and `override_reason`.
5. Log the override with timestamp, person nickname, and full reason string.
6. Re-evaluate aggregated severity (overridden faults do not contribute).
7. Do NOT delete the fault entry.
8. At the next detection cycle, if the underlying condition is still present,
   the override may expire (per-fault configuration).

The `request_clear` service also requires the caller to identify themselves (who)
and provide a reason string (why). These are logged in the audit trail.

---

## 8. ROS 2 Interfaces

### 8.1 Published topics

All topics use the `/sigyn/` namespace. Sensor-named topics use the common name
from the sensor name table (Section 9).

#### System state topics

| Topic                          | Type                              | QoS                        | Rate      | Description                                                     |
| ------------------------------ | --------------------------------- | -------------------------- | --------- | --------------------------------------------------------------- |
| `/sigyn/safety/state`          | `wr_interfaces/SafetyState`       | Reliable, latched, depth 1 | ≥ 10 Hz   | Aggregated safety level, fault count, velocity limits           |
| `/sigyn/safety/fault_events`   | `wr_interfaces/FaultEvent`        | Reliable, depth 100        | On event  | Individual fault activation/deactivation events                 |
| `/sigyn/safety/velocity_limit` | `geometry_msgs/Twist`             | Best effort, depth 1       | On change | Active velocity limits                                          |
| `/sigyn/connectivity`          | `wr_interfaces/ConnectivityState` | Reliable, latched, depth 1 | On change | Per-board connection: board_id, connected, pkg_ver, fw_compiled |

#### Navigation topics (from Board 1)

| Topic                    | Type                           | QoS                   | Rate  | Description            |
| ------------------------ | ------------------------------ | --------------------- | ----- | ---------------------- |
| `/sigyn/wheel_odom`      | `nav_msgs/Odometry`            | Best effort, depth 10 | 50 Hz | Wheel encoder odometry |
| `/sigyn/roboclaw/status` | `wr_interfaces/RoboClawStatus` | Best effort, depth 5  | 1 Hz  | Motor driver status    |

#### Proximity topics (from Board 1)

One topic per sensor, named by common name from sensor name table:

| Topic pattern                    | Type                | QoS                  | Rate       |
| -------------------------------- | ------------------- | -------------------- | ---------- |
| `/sigyn/proximity/<sensor_name>` | `sensor_msgs/Range` | Best effort, depth 5 | Per sensor |

Example: `/sigyn/proximity/front_left_forward`, `/sigyn/proximity/front_right_sideward`

#### IMU topics

One topic per IMU, named by common name from sensor name table:

| Topic pattern              | Type              | QoS                   | Rate   |
| -------------------------- | ----------------- | --------------------- | ------ |
| `/sigyn/imu/<sensor_name>` | `sensor_msgs/Imu` | Best effort, depth 10 | 100 Hz |

Example: `/sigyn/imu/main_body` (BNO055 on Board 2)

#### Power topics (from Board 2)

| Topic                      | Type                            | QoS               | Rate | Description                                        |
| -------------------------- | ------------------------------- | ----------------- | ---- | -------------------------------------------------- |
| `/sigyn/battery/status`    | `wr_interfaces/BatteryStatus`   | Reliable, depth 5 | 1 Hz | Voltage, current, SoC, charging state              |
| `/sigyn/power/<rail_name>` | `wr_interfaces/PowerRailStatus` | Reliable, depth 5 | 1 Hz | Individual DC-DC rail status, named by common name |

#### Temperature topics

One topic per sensor, named by common name:

| Topic pattern                      | Type                      | QoS                  | Rate |
| ---------------------------------- | ------------------------- | -------------------- | ---- |
| `/sigyn/temperature/<sensor_name>` | `sensor_msgs/Temperature` | Best effort, depth 5 | 1 Hz |

Example: `/sigyn/temperature/left_motor`, `/sigyn/temperature/right_motor`,
`/sigyn/temperature/roboclaw`

#### Elevator / gripper topics (from Board 3)

| Topic                    | Type                          | QoS                  | Rate      |
| ------------------------ | ----------------------------- | -------------------- | --------- |
| `/sigyn/elevator/status` | `wr_interfaces/StepperStatus` | Best effort, depth 5 | On change |
| `/sigyn/gripper/status`  | `wr_interfaces/StepperStatus` | Best effort, depth 5 | On change |

#### Diagnostics and performance

| Topic                       | Type                              | QoS                   | Rate   |
| --------------------------- | --------------------------------- | --------------------- | ------ |
| `/sigyn/teensy/performance` | `wr_interfaces/PerformanceReport` | Best effort, depth 5  | 0.2 Hz |
| `/sigyn/diagnostics`        | `diagnostic_msgs/DiagnosticArray` | Best effort, depth 10 | 1 Hz   |

#### Human notification

| Topic                       | Type                       | QoS                | Rate     |
| --------------------------- | -------------------------- | ------------------ | -------- |
| `/sigyn/safety/human_alert` | `wr_interfaces/HumanAlert` | Reliable, depth 10 | On event |

### 8.2 Subscribed topics

| Topic                                | Type                            | Description                                                                                                             |
| ------------------------------------ | ------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| `/cmd_vel`                           | `geometry_msgs/Twist`           | Velocity command from Nav2 or teleop                                                                                    |
| `/sigyn/navigation/charger_goal`     | `wr_interfaces/ChargerGoalInfo` | Distance (m) and estimated travel time (s) to the nearest charger; computed by the navigation package, not this package |
| `/sigyn/navigation/current_pose`     | `geometry_msgs/PoseStamped`     | Current robot pose from AMCL (for spin-in-place detection)                                                              |
| `/sigyn/behavior_tree/active_action` | `std_msgs/String`               | Currently active behavior tree action name                                                                              |

> **Note:** `charger_goal` replaces the old `goal_distance` topic. This package is
> NOT responsible for computing distance or travel time to the charger. That
> computation lives in the navigation package which knows the full map, costmap,
> and charger coordinates. The charger coordinates themselves are exposed as ROS 2
> parameters of the navigation node, not this package.

### 8.3 Services

| Service                         | Type                                  | Description                                                                                                                                                                                                                                          |
| ------------------------------- | ------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `/sigyn/safety/request_clear`   | `wr_interfaces/srv/RequestFaultClear` | Request clearance of a specific fault. Fields: `fault_id`, `board_id`, `instance`, `cleared_by` (person nickname), `clear_reason` (free string).                                                                                                     |
| `/sigyn/safety/human_override`  | `wr_interfaces/srv/HumanOverride`     | Submit a human override. Fields: `fault_id`, `board_id`, `instance`, `person_id` (nickname), `reason` (from permitted list), `notification_method` (see human_interaction.spec.md). Returns: success/failure, list of permitted reasons.             |
| `/sigyn/safety/fault_enumerate` | `wr_interfaces/srv/FaultEnumerate`    | Request the current complete fault registry from both PC side and from specified Teensy board(s). Useful to confirm the PC mirror is in sync with embedded state.                                                                                    |
| `/sigyn/config/set`             | `wr_interfaces/srv/SetTeensyConfig`   | Set a runtime parameter on a board. Sends `CONFIG_SET` and waits for acknowledgment.                                                                                                                                                                 |
| `/sigyn/config/get`             | `wr_interfaces/srv/GetTeensyConfig`   | Read a parameter from a board.                                                                                                                                                                                                                       |
| `/sigyn/config/reset`           | `wr_interfaces/srv/ResetTeensyConfig` | Reset all params on a board to SD defaults.                                                                                                                                                                                                          |
| `/sigyn/sd/list`                | `wr_interfaces/srv/SdList`            | List SD card files. Response: array of `{name, size_bytes, timestamp_iso}`.                                                                                                                                                                          |
| `/sigyn/sd/delete`              | `wr_interfaces/srv/SdDelete`          | Delete a named file from an SD card. Cannot delete the currently open log file.                                                                                                                                                                      |
| `/sigyn/sd/read_line`           | `wr_interfaces/srv/SdReadLine`        | Read one line from an SD file (for diagnostics; not for bulk transfer).                                                                                                                                                                              |
| `/sigyn/time_sync`              | `wr_interfaces/srv/PcTimeSync`        | Send the current ROS time to a board. The Teensy uses this to annotate log lines with an estimated wall-clock time. If possible, the Teensy also sets the modification time of the current SD log file (once per connection, on first time message). |

### 8.4 Actions

| Action                 | Type                               | Description                       |
| ---------------------- | ---------------------------------- | --------------------------------- |
| `/sigyn/elevator/move` | `wr_interfaces/action/StepperMove` | Move elevator to target position. |
| `/sigyn/gripper/move`  | `wr_interfaces/action/StepperMove` | Move gripper to target position.  |

> **Future scope note:** The gripper is currently controlled by a Raspberry Pi 5 via
> WiFi, outside this package. If that control path is eventually wrapped into this
> package's communication model, the package name may change to `wr_ros_to_remote`
> to reflect that it communicates with multiple embedded targets.

### 8.5 Custom message and service types

All custom types are defined in the `wr_interfaces` ROS 2 package (separate from
`wr_proto_msgs`). Key types:

```
wr_interfaces/SafetyState:
  uint8   aggregated_level        # 0=NORMAL 1=WARNING 2=DEGRADED 3=EMERGENCY_STOP 4=SHUTDOWN
  uint8   active_fault_count
  float32 max_linear_velocity
  float32 max_angular_velocity
  bool[]  board_connected         # indexed [0]=unused [1]=board1 [2]=board2 [3]=board3
  builtin_interfaces/Time stamp

wr_interfaces/FaultEvent:
  uint8   board_id                # 0=PC 1-3=Teensy board
  string  fault_id
  string  source_class
  uint8   instance
  uint8   severity
  bool    active
  bool    human_override_active
  string  description             # verbose only for fault/error messages
  builtin_interfaces/Time activated_at

wr_interfaces/ConnectivityState:
  BoardConnectionInfo[] boards    # one per known board
    uint8  board_id
    bool   connected
    string pkg_version
    string fw_compiled_at
    string pc_compiled_at
    builtin_interfaces/Time last_heartbeat

wr_interfaces/ChargerGoalInfo:
  float32 distance_m
  float32 estimated_travel_s
  geometry_msgs/PoseStamped charger_pose

wr_interfaces/HumanAlert:
  uint8   tier
  string  title
  string  body
  string  fault_id
  string[] permitted_reasons      # if human response expected
  string[] notify_people          # people nicknames to notify

wr_interfaces/srv/HumanOverride:
  # request
  string  fault_id
  uint8   board_id
  uint8   instance
  string  person_id               # nickname from people table
  string  reason                  # must be in permitted_reasons for this fault
  string  notification_method     # "email", "sms", "log_only"
  ---
  # response
  bool    success
  string  message
  string[] permitted_reasons      # populated if reason was invalid

wr_interfaces/srv/RequestFaultClear:
  string  fault_id
  uint8   board_id
  uint8   instance
  string  cleared_by              # person nickname or BT node name
  string  clear_reason
  ---
  bool    success
  string  message

wr_interfaces/srv/FaultEnumerate:
  uint8[] board_ids
  ---
  wr_interfaces/FaultEvent[] faults
```

---

## 9. Sensor Naming

### 9.1 Motivation

Numeric sensor identifiers (e.g., temperature sensor 0, VL53L0X sensor 3) are
meaningless in topics, logs, and alerts. When a fault says "VL53L0X sensor 4 has
Ring 3 occupancy," the operator cannot act without looking up a pinout diagram.
When a log says "temperature sensor 2 is 87°C," it is unclear which motor is
overheating.

All sensor references in topics and logs use a **common name** — a short,
human-readable string that describes the sensor's physical role and location.

### 9.2 Sensor name table

The sensor name table maps `(board_id, sensor_type, instance_id)` →
`common_name_string`. It is loaded from a JSON configuration file at node startup.

```json
{
  "sensor_names": [
    {"board": 1, "type": "VL53L0X", "instance": 0, "name": "front_left_fwd"},
    {"board": 1, "type": "VL53L0X", "instance": 1, "name": "front_left_side"},
    {"board": 1, "type": "VL53L0X", "instance": 2, "name": "front_right_fwd"},
    {"board": 1, "type": "VL53L0X", "instance": 3, "name": "front_right_side"},
    {"board": 1, "type": "VL53L0X", "instance": 4, "name": "rear_left_fwd"},
    {"board": 1, "type": "VL53L0X", "instance": 5, "name": "rear_left_side"},
    {"board": 1, "type": "VL53L0X", "instance": 6, "name": "rear_right_fwd"},
    {"board": 1, "type": "VL53L0X", "instance": 7, "name": "rear_right_side"},
    {"board": 1, "type": "TEMP",    "instance": 0, "name": "left_motor"},
    {"board": 1, "type": "TEMP",    "instance": 1, "name": "right_motor"},
    {"board": 1, "type": "TEMP",    "instance": 2, "name": "roboclaw"},
    {"board": 2, "type": "IMU",     "instance": 0, "name": "main_body"},
    {"board": 2, "type": "TEMP",    "instance": 0, "name": "battery"},
    {"board": 2, "type": "BATT",    "instance": 0, "name": "main_battery"},
    {"board": 2, "type": "POWER_RAIL", "instance": 0, "name": "5v_logic"},
    {"board": 2, "type": "POWER_RAIL", "instance": 1, "name": "12v_drive"},
    {"board": 3, "type": "STEPPER", "instance": 0, "name": "elevator"},
    {"board": 3, "type": "STEPPER", "instance": 1, "name": "gripper"}
  ]
}
```

The configuration file path is exposed as a ROS 2 parameter: `sensor_names_file`.

### 9.3 Unknown sensors

If a Teensy sends data for a `(board_id, sensor_type, instance_id)` combination not
in the table, the `TopicPublisher` logs a one-time warning and publishes to a
fallback topic `/sigyn/unknown/<board_id>/<sensor_type>/<instance_id>`. This allows
new sensors to be discovered without dropping data.

### 9.4 Each sensor reports independently

Each sensor message carries data for exactly one sensor instance. There are no
aggregate "all temperatures in one message" message types. This design choice:

- Removes the false implication that all readings in one message are simultaneous.
- Allows sensors on different polling rates to report at their natural rate.
- Simplifies per-sensor fault detection.
- Allows per-sensor common naming without any indexing gymnastics.

---

## 10. Parameter Management

### 10.1 ROS 2 parameters owned by this package

All parameters are declared on the lifecycle node. Teensy-resident parameters that
are forwarded from `PARAM_DUMP` messages are also declared as ROS 2 parameters at
runtime, with a `board<N>/` prefix. This means rqt or `ros2 param list` can show
all known parameters — PC-side and board-side — in one place.

#### PC-side parameters

| Parameter                        | Type   | Default                          | Description                                            |
| -------------------------------- | ------ | -------------------------------- | ------------------------------------------------------ |
| `board1_port`                    | string | `/dev/teensy_sensor`             | Board 1 serial port                                    |
| `board2_port`                    | string | `/dev/teensy_sensor2`            | Board 2 serial port                                    |
| `board3_port`                    | string | `/dev/teensy_gripper`            | Board 3 serial port                                    |
| `baud_rate`                      | int    | 921600                           | Serial baud rate (all boards)                          |
| `board1_required`                | bool   | `true`                           | Fault if Board 1 absent (false in sim)                 |
| `board2_required`                | bool   | `false`                          | Fault if Board 2 absent                                |
| `board3_required`                | bool   | `false`                          | Fault if Board 3 absent                                |
| `heartbeat_timeout_ms`           | int    | 500                              | Board heartbeat timeout                                |
| `pc_heartbeat_interval_ms`       | int    | 500                              | PC → all boards heartbeat rate                         |
| `startup_timeout_ms`             | int    | 10000                            | Wait for required boards at startup                    |
| `reconnect_interval_ms`          | int    | 1000                             | Retry interval for lost ports                          |
| `max_malformed_rate_pct`         | float  | 5.0                              | Max % malformed before DEGRADED fault                  |
| `estop_clear_confirm_timeout_ms` | int    | 500                              | Timeout for e-stop clear confirm                       |
| `sensor_names_file`              | string | `config/sensor_names.json`       | Sensor name table file                                 |
| `people_file`                    | string | `config/people.json`             | People/notification table file                         |
| `safety_log_file`                | string | `~/.ros/sigyn_safety_events.log` | Safety event log path                                  |
| `use_realtime_threads`           | bool   | `false`                          | Enable SCHED_FIFO on dispatcher thread (hardware only) |
| `use_sim_time`                   | bool   | (ROS default)                    | When true, disables RT priority and board requirements |

#### Board-mirrored parameters (declared at runtime from PARAM_DUMP)

Each parameter from a Teensy board is declared as `board<N>/<param_name>`. For
example, `board1/wheel_diameter_m`, `board1/ring2_distance_m`,
`board1/max_motor_temp_c`.

These parameters are read-mostly. Writing them via the ROS 2 parameter interface
triggers a `CONFIG_SET` command to the board. The board's acknowledgment is required
before the parameter server value is updated.

> **Design note:** Wheel calibration and similar physical constants that require
> periodic adjustment (via a calibration procedure) are exposed here. A user with
> rqt can update `board1/wheel_diameter_m` and it takes effect immediately without
> a recompile.

#### Future parameter persistence (open question)

The current design mirrors board parameters into the ROS 2 parameter server, but
changes that survive a board reboot require the new values to be committed to the
board's SD YAML file via `CONFIG_RESET` → `CONFIG_SET` → `CONFIG_RESET`. A future
improvement is a parameter database (seeded from JSON, updated by human overrides,
with a service to reset to JSON defaults). See OQ-STT-4.

### 10.2 Charger coordinates

The coordinates of the charging station(s) are owned by the navigation package, not
this package. Distance and travel time to the charger are computed by navigation and
published to `/sigyn/navigation/charger_goal` (Section 8.2). This package subscribes
to that topic and uses it to inform battery-related fault thresholds, but does not
recompute it.

---

## 11. Human Notification Integration

### 11.1 Overview

Human notification is detailed in `human_interaction.spec.md`. This section
describes only what `wr_ros_teensy` is responsible for:

- Publish `wr_interfaces/HumanAlert` events to `/sigyn/safety/human_alert`.
- The downstream notification node subscribes and handles delivery (email, SMS,
  audible, etc.). This package does not manage delivery.
- Provide the fault enumeration service so a human-facing UI can display all
  currently active faults.
- Accept human override requests via the `/sigyn/safety/human_override` service and
  process the fault-clearing logic.

### 11.2 People table

The people table (`config/people.json` — **excluded from git**, see Section 16.5)
associates person identifiers with notification contact information. Format:

```json
{
  "people": [
    {
      "nickname": "wim",
      "display_name": "William Rotenberg",
      "email": "EMAIL_HERE",
      "phone_e164": "PHONE_HERE"
    }
  ]
}
```

A version-controlled example file (`config/people.example.json`) ships with the
repo with placeholder values and instructions. Anyone who deploys this code must
copy the example file to `people.json` and fill in real values. The real file must
NEVER be committed to git (it is listed in `.gitignore`).

Passwords and authentication tokens are NEVER stored in or transmitted to the
Teensy side. All notification delivery happens exclusively on the PC side.

### 11.3 Alert tiers

| Severity        | Tier | Channels                                         |
| --------------- | ---- | ------------------------------------------------ |
| WARNING         | 4    | Append to periodic status report                 |
| DEGRADED        | 3    | App push notification                            |
| EMERGENCY_STOP  | 2    | Push notification + audible tone on PC           |
| SYSTEM_SHUTDOWN | 1    | All channels simultaneously; all people in table |

Full details: `human_interaction.spec.md`.

---

## 12. Behavior Tree Integration

### 12.1 What the safety coordinator communicates to the BT

| Signal                     | Mechanism                                                                          | Description                     |
| -------------------------- | ---------------------------------------------------------------------------------- | ------------------------------- |
| Safety level change        | `/sigyn/safety/state` topic                                                        | BT condition nodes subscribe    |
| Individual fault event     | `/sigyn/safety/fault_events` topic                                                 | BT can react to specific faults |
| Velocity limit             | `/sigyn/safety/velocity_limit` topic                                               | Consumed by twist multiplexer   |
| Return-to-charger advisory | Fault event: `source="BATTERY"`, `severity=WARNING`, description="ReturnToCharger" | BT plans accordingly            |

### 12.2 What the BT communicates to the safety coordinator

| Signal               | Mechanism                                                                                    | Description                                  |
| -------------------- | -------------------------------------------------------------------------------------------- | -------------------------------------------- |
| Active action name   | `/sigyn/behavior_tree/active_action`                                                         | Used for forensic logging                    |
| Allow close approach | `/sigyn/safety/request_clear` service, `cleared_by="BT"`, `clear_reason="BT_CLOSE_APPROACH"` | BT explicitly overrides Ring 2/3 for docking |

### 12.3 Velocity limit enforcement in the navigation stack

The twist multiplexer (`wr_twist_multiplexer`) subscribes to
`/sigyn/safety/velocity_limit`. How Nav2 speed limits interact with this (e.g.,
whether Nav2 SpeedLimit costmap plugin can be driven dynamically) is an open
question. See OQ-STT-11. This package publishes the limit topic regardless; the
consuming infrastructure is handled in its own packages and specs.

---

## 13. PC-Side Safety Computations

### 13.1 Spin-in-place detection

Subscribes to `/sigyn/wheel_odom` and `/sigyn/navigation/current_pose`. When
wheel odometry reports non-zero displacement but AMCL pose change over the same
interval is below threshold, activates a `DELIBERATIVE/CRITICAL` fault with
`requires_human_ack`.

Parameters: `spin_detect_odom_threshold_m` (0.1 m), `spin_detect_pose_threshold_m`
(0.05 m), `spin_detect_window_ms` (500 ms).

### 13.2 LiDAR-based ring computation supplement

The TOF sensors on Board 1 provide 8 point samples. The LiDAR provides a 360° scan.
The safety coordinator supplements the TOF ring computation by processing the LiDAR
scan in the direction of current motion. This runs at 10 Hz.

**Latency note:** LiDAR → PC → safety coordinator → Board 1 is too slow (> 100 ms)
for Ring 3 enforcement. LiDAR-based ring computation is supplementary and deliberative
only. Ring 3 enforcement remains Board 1's exclusive responsibility.

### 13.3 PC serial heartbeat monitoring

The safety coordinator sends `HEARTBEAT_PC` to **all connected boards** every
`pc_heartbeat_interval_ms`. Each board monitors receipt of PC heartbeats; if they
stop, the board enters a grace period then begins a soft stop.

PC heartbeats serve a dual purpose:
1. Confirm to the Teensy that the PC is alive.
2. Receipt confirmation: when the Teensy responds to a heartbeat, the PC uses
   the response to detect a board crash or restart, not just USB disconnection.
   A board that stops responding to heartbeats but whose USB descriptor is still
   visible has likely crashed and restarted; the PC initiates protocol
   renegotiation.

The heartbeat timer is a high-priority dedicated timer, not a general ROS 2 timer
that may be delayed by callback queue depth.

---

## 14. Logging and Audit Trail

### 14.1 Safety event log

- Location: `~/.ros/sigyn_safety_events.log` (configurable via parameter).
- Format: one JSON object per line — timestamp, event type, fault fields, operator
  identity, active BT action at the time of the event.
- The log includes the `wr_proto_msgs` package version and compilation timestamp
  as a header entry at node startup.
- No automatic log rotation. The log file grows until manually managed. A future
  feature may add rotation; for now log management is the operator's responsibility.
- At startup, the node may optionally compute an estimated remaining disk space and
  log a WARNING if below `kMinDiskSpaceWarningBytes` (configurable).

### 14.2 SD card log management

Board-side SD card logging is managed by the Teensy firmware specification
(`wr_teensy_boards.spec.md`). From the PC side:

- The `/sigyn/sd/list` service returns `{name, size_bytes, timestamp}` per file.
- The `/sigyn/sd/delete` service deletes a named file; the currently open log file
  cannot be deleted.
- A possible future maintenance node (not in scope for initial implementation) may
  periodically poll SD file lists and prune old files when disk space is low.

### 14.3 Behavior tree action logging

The Teensy firmware writes the BT action name to its SD log once each time a new
name is transmitted. The PC sends a `BT_ACTION` message to the Teensy when a BT
action completes with SUCCESS or FAILURE (not on each RUNNING tick). Optionally,
the first tick of a new action node name may also be sent; this is an open question
(OQ-STT-6).

The Teensy timestamps the received name and retains it in memory. It is not appended
to any other log line; it is logged as a standalone event with a Teensy `millis()`
timestamp. Any future use can evaluate its staleness independently.

---

## 15. Testing Requirements

### 15.1 Unit test principles

Every class in Section 4 must have unit tests that run without a serial port,
Teensy, or ROS 2 runtime. Tests inject mock implementations of all I/O interfaces
and mock `rclcpp` clock.

### 15.2 Component test matrix

| Component                       | Key behavioral tests                                                                                                                                                                                                                                                                                                                                          |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `MessageParser`                 | Valid messages; missing required field; version mismatch; malformed JSON; empty payload; boundary field values; interrupted stream resumed                                                                                                                                                                                                                    |
| `SerialBridge` mock             | Inject N messages; verify order; send with priority queuing; reconnect triggers renegotiation callback                                                                                                                                                                                                                                                        |
| `SafetyCoordinator`             | Fault activated; reason added to `active_reasons`; second fault same entry adds second reason; fault clears → reason removed; e-stop cleared only when all reasons empty; human override applied; aggregated severity; velocity limit selection; authority chain: PC cannot override Hardware/EMBEDDED faults; FAULT_CLEAR_CMD sent on individual fault clear |
| `MessageDispatcher`             | Heartbeat timeout detected; handler registered and called; unknown type is silent                                                                                                                                                                                                                                                                             |
| `wr_proto_msgs::CommandFactory` | Round-trip: output of each command method parses successfully                                                                                                                                                                                                                                                                                                 |
| Threading                       | P0 message not delayed by P3 backlog                                                                                                                                                                                                                                                                                                                          |

### 15.3 Integration tests

Mock `ISerialBridge` instances injected into `TeensyBridgeNode`. Scenarios:

- Board connects: protocol announcement verified; name table populated.
- Version mismatch: safe mode activated; no motion commands sent.
- Board disconnects mid-session: fault activated; reconnect attempted.
- Three independent faults → three entries in `active_reasons` → all three must
  clear before e-stop clears.
- Human override via service with valid person id and reason: override applied.
- Human override with invalid person: rejected.
- SD time sync: `PC_TIME` sent on connect; Teensy log timestamp response validated.
- Heartbeat stops → board detected as crashed → renegotiation initiated.
- cmd_vel above velocity limit: clamped.
- Sensor name table lookup: correct common name in published topic.
- Unknown sensor: fallback topic published; one-time warning logged.

---

## 16. Implementation Notes

### 16.1 This is a fresh implementation

This package is written from scratch. The existing `sigyn_to_sensor_v2` implementation
may inform design decisions but imposes no constraints. There is no migration path;
the old and new implementations run as separate packages. When the new implementation
is confirmed operational, `sigyn_to_sensor_v2` is retired.

### 16.2 Build system

`wr_ros_teensy` is a standard ROS 2 / ament_cmake package. It depends on:
- `wr_proto_msgs` (the shared protocol library — header-only or static library)
- `wr_interfaces` (ROS 2 custom message types)
- Standard ROS 2 packages: `rclcpp`, `rclcpp_lifecycle`, `nav_msgs`, `sensor_msgs`,
  `geometry_msgs`, `diagnostic_msgs`

### 16.3 Naming conventions

- Class names: `PascalCase`.
- Parameters: `snake_case_with_underscores`.
- Topics: `/sigyn/category/common_name_from_table`.
- All sensor-specific strings in topics and logs use the sensor name table.

### 16.4 No gripper scope presently

The gripper is out of scope for this implementation. It is handled by the RPi5 over
WiFi. The `StepperMove` action for `gripper` is included in the interface definition
as a placeholder for future integration.

### 16.5 Secrets and private data in git

The following files must NEVER appear in the git repository:
- `config/people.json` (real names, email addresses, phone numbers)
- Any file containing authentication tokens, passwords, or API keys

These files are listed in `.gitignore`. The repository ships `config/people.example.json`
with placeholder values and instructions.

---

