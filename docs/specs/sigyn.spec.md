# Sigyn Robot System Specification

**Document status:** Draft v0.2  
**Last updated:** 2026-03-02  
**Scope:** The Sigyn personal assistant robot — hardware, software, behaviors, and
development philosophy  
**Note:** This document is intentionally broad. It will be split into focused
sub-specifications as each area is developed further. Sub-spec documents belong in
`docs/specs/` alongside this file.

**Sub-specifications written so far:**
- `docs/specs/safety_system.spec.md` — Safety behavior across firmware and PC
- `docs/specs/wr_teensy_boards.spec.md` — Teensy 4.1 firmware architecture
- `docs/specs/wr_ros_teensy.spec.md` — PC-side serial bridge and safety coordinator

---

## Table of Contents

1. [Mission and Motivation](#1-mission-and-motivation)
2. [Design Philosophy](#2-design-philosophy)
3. [Hardware Platform](#3-hardware-platform)
4. [Software Architecture Overview](#4-software-architecture-overview)
5. [Communication Architecture](#5-communication-architecture)
6. [Operational Milestones](#6-operational-milestones)
7. [Repository Structure](#7-repository-structure)
8. [Cross-Cutting Requirements](#8-cross-cutting-requirements)
9. [Human Notification System](#9-human-notification-system)
10. [System-Wide Configuration Management](#10-system-wide-configuration-management)
11. [Clock Synchronization and Timestamps](#11-clock-synchronization-and-timestamps)
12. [Privacy and Data Handling](#12-privacy-and-data-handling)
13. [System-Wide Testing Strategy](#13-system-wide-testing-strategy)
14. [Outdoor Robot Design Portability](#14-outdoor-robot-design-portability)
15. [Work in Progress and Open Questions](#15-work-in-progress-and-open-questions)

---

## 1. Mission and Motivation

Sigyn is a personal assistant robot designed to help an aging person live
independently. The primary operator is the robot's owner, who needs a machine that
can be trusted to carry out life-critical tasks including:

- Delivering medicine and beverages on request.
- Patrolling the home and detecting hazards (intruders, fires, falls).
- Calling for help when the owner needs assistance.
- Detecting that the owner has fallen and has not gotten up.
- Performing routine tasks (fetch, carry, monitor) to reduce physical demands on
  the owner.

These are not future aspirations — they are the design criteria that must be kept
in view throughout every architecture and implementation decision. A robot that is
fragile, unpredictable, or difficult to maintain is not suitable for this purpose.

**Secondarily**, Sigyn is intended to be a project worthy of sharing. Clean code,
thorough documentation, specification-driven development, and good test coverage
matter both for the owner's own benefit and for the benefit of others who might
learn from or build on this work.

**The outdoor successor robot**, currently in planning, will apply lessons learned
from Sigyn to a yard-patrol robot. Architecture decisions should note which ones are
Sigyn-specific and which are intended to be portable to other platforms.

---

## 2. Design Philosophy

### 2.1 Specification-driven development

All significant features begin with a written specification before implementation
starts. The specification:

- Defines the requirements clearly enough that a test can be written against them.
- Notes architectural assumptions that must be revisited if the hardware changes.
- Lists open questions that must be answered before implementation.
- Is kept as the permanent design record, updated to reflect decisions made during
  implementation.

This document and the documents in `docs/specs/` are the foundation of that
record.

### 2.2 Testing as a first-class requirement

No safety-critical code is merged without a corresponding unit test. All hardware
interfaces use dependency injection so that tests can run without connected hardware.
The test suite must pass on every build, and every known test failure is a blocker.

### 2.3 Incremental progress with architectural integrity

Progress is demonstrated through periodic milestones (see Section 6). However,
milestone pressure does not justify bypassing design standards. When a shortcut is
taken to meet a milestone, it must be recorded in `TODO.md` with a priority marker.
Accumulated shortcuts are reviewed and addressed in dedicated refactoring sessions
between milestones.

### 2.4 Graceful degradation over hard stopping

Where it is safe to do so, the robot should continue functioning in a reduced
capacity rather than stopping entirely. Every component must define its own
"degraded" mode and communicate its state clearly. A robot that stops unexpectedly
is failing as a personal assistant even when it is behaving correctly from a safety
standpoint.

---

## 3. Hardware Platform

### 3.1 Computing units

| Unit | Role | OS / Runtime |
|---|---|---|
| sigyn7900a (AMD Ryzen 9 7900X, ASRock B650E-I Mini-ITX) | Main compute: ROS 2, navigation stack, behavior trees, safety coordinator, sensor aggregation | Ubuntu 24.04 LTS, ROS 2 Jazzy |
| Raspberry Pi 5 + AI Hat | End-effector camera: Yolo-based object detection for grasping | TBD, likely ROS 2 or standalone Python |
| Pi (TBD model) | Charging station April Tags detection and docking coordination | TBD |
| Jetson Nano Orin (planned) | Semantic segmentation, object detection, VLA model for the multi-DOF arm | JetPack, ROS 2 |
| Teensy 4.1 × 3 | Real-time embedded control; sensor I/O; motor control; safety | PlatformIO / Arduino-style C++17 |

**Authority for safety decisions:** Only `sigyn7900a` and the Teensy boards
currently participate in the safety system. The Pi and Jetson units are monitored
for connectivity (heartbeat) but do not yet issue or receive safety commands.
Future expansion of the safety system to these units is possible but not currently
specified.

### 3.2 Teensy board assignments

| Board | udev symlink | Primary responsibilities |
|---|---|---|
| Board 1 | `/dev/teensy_sensor` | RoboClaw motor control; wheel odometry; 8× VL53L0X proximity sensing; temperature monitoring; SD logging; **safety authority** |
| Board 2 | `/dev/teensy_sensor2` | 5× INA226 power monitoring (main battery + 4 DC-DC rails); 2× BNO055 IMU; temperature |
| Board 3 | `/dev/teensy_gripper` | Elevator stepper motor; extender stepper motor; limit switches |

Board 1 is the **embedded safety authority**. It holds the motor e-stop pin and the
inter-board fault GPIO outputs. All safety commands that result in motor action are
executed by Board 1.

### 3.3 Sensors and hardware

| Sensor / Component | Board | Qty | Notes |
|---|---|---|---|
| RoboClaw motor controller | Board 1 via UART (Serial7) | 1 | Drives two differential-drive wheels |
| VL53L0X time-of-flight sensors | Board 1 via I²C | 8 | Placed around robot perimeter; may be replaced with wider-FOV sensors |
| TMP36 analog temperature sensors | Boards 1 & 2 | Multiple | Motor and system temperatures |
| INA226 power monitors | Board 2 via I²C | 5 | Main battery + 4 DC-DC rails |
| BNO055 IMU | Board 2 via I²C | 2 | Tilt detection, angular rate; dual for redundancy |
| Stepper motors (elevator, extender) | Board 3 | 2 | Gripper mechanism |
| LiDAR (LD series) | sigyn7900a via USB | 2 | Obstacle avoidance and localization |
| OAK-D camera | sigyn7900a via USB | 1 | Object detection, person detection |
| End-effector camera (gripper Pi) | Raspberry Pi 5 | 1 | Grasping assistance |
| Parallel gripper | Raspberry Pi 5 via GPIO | 1 | Current gripper (to be replaced by multi-DOF arm) |
| Mechanical e-stop button | Battery circuit | 1 | Physical battery disconnect; no software visibility |
| Motor power relay (24 V) | Board 1, GPIO 31 | 1 | RoboClaw power cycling |
| Main battery relay | Board 1, GPIO 32 | 1 | Emergency system shutdown |
| Piezo buzzer | Board 1 (planned) | 1 | Tier 1 human notification |

### 3.4 Hardware planned additions

- Battery temperature sensor (next battery replacement).
- Solid-state sensing of physical e-stop button state.
- Solid-state relay for programmable main battery disconnect.
- Charging station with April Tags markers and contact pads.
- Jetson Nano Orin (camera-based perception).
- Multi-DOF robotic arm (to replace parallel gripper).

---

## 4. Software Architecture Overview

### 4.1 Software layers

```
Behavioral layer:   sigyn_behavior_trees ───────────────────────────┐
                    sigyn_house_patroller                            │
                                                                      ▼
Navigation layer:   Nav2 (move_base2) ──────── twist_multiplexer ───►cmd_vel
                                                                      │
PC safety layer:    sigyn_to_teensy (safety coordinator within)      │
                         ▲  (serial)  ▼                              │
Embedded layer:     Board 1 ─(Serial5/GPIO)─ Board 2                 │
                    Board 1 ─(Serial3/GPIO)─ Board 3                 │
                    Board 1: RoboClaw ◄──────────────────────────────┘
```

### 4.2 Key ROS 2 packages

| Package | Responsibility |
|---|---|
| `sigyn_to_teensy` | Serial bridge + safety coordination between Teensy boards and ROS 2; protocol adapter + authoritative fault state on the PC side. **Note:** Previously called `sigyn_to_sensor_v2` in the Sigyn monorepo. Being rewritten and moved to its own repository. |
| `sigyn_behavior_trees` | Behavior tree plugins and launch configurations |
| `sigyn_interfaces` | Custom ROS 2 message, service, and action definitions |
| `sigyn_description` | URDF models and Gazebo simulation worlds |
| `sigyn_bringup` | System launch files |
| `wr_ldlidar` | LiDAR driver (forked, customized) |
| `wr_twist_multiplexer` | Velocity command multiplexer (forked, customized); must support a safety velocity limit input from `sigyn_to_teensy` |

### 4.3 Deployment

Sigyn uses an in-house deployment tool (`Sigyn2`) that manages building,
uploading, and monitoring software across all computing units. All ROS 2
packages must be buildable with `colcon build` from the workspace root.
Teensy firmware is built and uploaded via PlatformIO from the
`sigyn_teensy_boards` repository.

---

## 5. Communication Architecture

### 5.1 Teensy-to-PC serial

Each Teensy board communicates with `sigyn7900a` over a dedicated USB serial
connection at 115200 baud. The protocol uses newline-delimited JSON with short
keys. Message types are versioned (see `docs/specs/safety_system.spec.md`
Section 11 and the forthcoming `docs/specs/message_protocol.spec.md`).

All messages carry a board ID prefix (e.g., `BATT2:{ ... }` from Board 2).
Safety fault messages have the highest transmission priority.

### 5.2 Inter-board communication

Boards communicate with each other over:

- **UART (Serial5):** Board 1 ↔ Board 2, 1 Mbit/s; used for periodic status and
  fault frame exchange.
- **GPIO (dedicated pins):** Board 1 monitors active-HIGH fault signals from Boards
  2 and 3. Board 1 drives a shared GPIO output signal readable by all boards to
  indicate an emergency stop condition.

> **Constraint:** The GPIO inter-board lines provide < 1 ms latency for critical
> fault signals. UART provides richer but slower (< 50 ms) status data.

### 5.3 ROS 2 topic structure

All Sigyn ROS 2 topics must follow the namespace `/sigyn/`. The current partial
topic list will be formalized in `docs/specs/message_protocol.spec.md`. Key
safety-related topics:

| Topic | Direction | Description |
|---|---|---|
| `/sigyn/safety/state` | Published | Authoritative current safety state; latched, reliable QoS |
| `/sigyn/safety/fault_events` | Published | Stream of fault activations and deactivations |
| `/sigyn/safety/human_alert` | Published | Human notification trigger with all required fields |
| `/sigyn/safety/request_clear` | Service | Request fault clearance (PC may request; Teensy decides) |
| `/sigyn/safety/human_override` | Service | Human override submission with authentication |
| `/cmd_vel` | Subscribed (Board 1 via bridge) | Velocity commands from navigation or teleoperation |

---

## 6. Operational Milestones

Milestones are listed in order. Each must be achieved while maintaining all
prior milestone capabilities.

| ID | Milestone | Status |
|---|---|---|
| M1 | Robot navigates a predetermined path in the house without collision | Complete |
| M2 | Fetch a can of beer on request (behavior tree drives full task) | Complete |
| M3 | Return to charging station autonomously when battery is low | In progress |
| M4 | Safety system redesign: complete spec, architecture, and test infrastructure | In progress (this document) |
| M5 | Run house patrol behavior tree with redesigned safety system active | Not started |
| M6 | Fetch medicine on request with life-critical safety guarantees | Not started |
| M7 | Detect owner fall and call for help | Not started |
| M8 | Demonstrate multi-DOF arm grasping with safety integration | Not started |

---

## 7. Repository Structure

| Repository | Contents |
|---|---|
| `wimblerobotics/sigyn_teensy_boards` | Teensy 4.1 firmware for all three boards |
| `wimblerobotics/sigyn_to_teensy` | ROS 2 bridge and safety coordinator for PC (**to be created** from `sigyn_to_sensor_v2` in the monorepo) |
| `wimblerobotics/Sigyn` | Monorepo for all other ROS 2 packages, configuration, and documentation |

The monorepo is in the process of being split into focused repositories to enable
reuse in the outdoor robot project. The split plan is:

1. `sigyn_interfaces` — already a natural independent package; split first.
2. `sigyn_to_teensy` — the rewrite should begin in its own repository immediately.
3. `sigyn_description` — URDF and Gazebo; split when outdoor robot diverges.
4. `sigyn_behavior_trees` and `sigyn_house_patroller` — split last; depend on all
   others.
5. `sigyn_bringup` — stays in monorepo until all packages are split.

Until each split is complete, the monorepo remains the build source for that package.

---

## 8. Cross-Cutting Requirements

These requirements apply to all components of the system:

### 8.1 No silent failures

Every component must have an observable health state. A component that is
healthy should say so (heartbeat). A component that is degraded must say so and
describe the nature of the degradation. A component that fails without reporting
must itself be treated as a fault condition.

### 8.2 Timestamps and clocks

The main PC (ROS 2) clock is the authoritative time source. The Teensy boards use
their own millisecond timestamps for ordering within-board events. All messages
arriving at the PC are stamped with the ROS 2 clock on receipt. The Teensy timestamp
is retained for within-board latency measurement.

### 8.3 Consistent terminology

All documents, code, and messages must use the same terminology for the same
concepts. This specification defines the canonical terms in Section 4 (Definitions
in `safety_system.spec.md`) and in the forthcoming glossary.

### 8.4 No heap allocation in embedded firmware

Teensy firmware must use only static allocation and stack allocation. No `new`,
no `malloc`, no `std::vector` or `std::string` in production firmware code.
Test code may use heap allocation.

### 8.5 Documentation co-located with implementation

Every module, ROS 2 node, and significant class must have a description of its
purpose, its dependencies, and its observable state. API documentation must be
in-source (Doxygen or equivalent). Architectural documentation belongs in `docs/`.

---

---

## 9. Human Notification System

This section describes requirements for notifying a human being when the robot
encounters a safety event, needs assistance, or has a status report. The detailed
delivery mechanism is specified in `docs/specs/human_notification.spec.md` (not
yet written); this section establishes the architecture and tiers.

### 9.1 Notification tiers

Notification urgency is tiered by how quickly a human must receive and potentially
respond:

| Tier | Name | Response time SLA | Delivery mechanisms |
|---|---|---|---|
| T1 | Immediate local | < 1 second | Piezo buzzer on Board 1; audio file on PC speakers |
| T2 | In-home alert | < 5 seconds | Speech synthesis on PC; optional visual indicator (smart light, to be added) |
| T3 | Remote primary | < 30 seconds | Native iOS/Android app push notification via APNS/FCM |
| T4 | Remote redundant | < 5 minutes | SMS via carrier API (Twilio, AWS SNS, or equivalent); Email |
| T5 | Emergency services | Human decision | Human calls 911; automated call via PSTN module (to be specified) |

All tiers below T5 must be attempted in parallel for `SYSTEM_SHUTDOWN` and
`EMERGENCY_STOP` severity events that require human acknowledgment. T3 and T4
must fall back to each other: if the app push fails, the SMS and email must still
attempt delivery.

**Carrier email gateway deprecation note:** The TMobile/AT&T email-to-SMS gateway
is being deprecated and is not reliable enough for life-critical messaging. All
SMS must go through a dedicated carrier API (Twilio or AWS SNS). This must be
implemented before relying on SMS for any safety notification.

### 9.2 Native app design requirements

A native iOS (and optionally Android) application must be designed with:
- Push notification receipt and display even when the app is in the background.
- The ability to view current safety state (active faults, severity, robot status)
  without requiring app launch from a notification.
- Human override submission: the user can acknowledge a fault and select a reason
  from the approved list, or submit a free-form "other situation" text.
- No dependency on the robot being on the same Wi-Fi network for push notifications
  (notifications go through APNS/FCM over any internet connection).
- A fallback static web page accessible from Safari (for when the app is not
  installed) via a URL included in the SMS/email.

The app communicates with the robot over local Wi-Fi using the ROS 2 web bridge
(`sigyn_websocket` / Foxglove-compatible). It subscribes to safety state topics
and submits commands via the human override service.

### 9.3 Emergency services

Automatically calling 911 from a robot raises regulatory, legal, and ethical
questions. The current specification does not mandate automatic 911 calls. What
is required:

- The system must store the physical address of the home in persistent configuration.
- The system must have a mode in which it can present the emergency status and
  address to the owner's phone for a human-initiated 911 call.
- If a PSTN or VoIP module is added in the future, the notification architecture
  must accommodate it without changing the tier system.

### 9.4 Notification reliability requirements

- A notification must be retried if delivery cannot be confirmed within a tier-
  appropriate timeout.
- Repeated re-notifications for the same active fault are suppressed after the
  first delivery, until the fault clears and re-activates.
- All notifications must be logged with timestamp, tier, delivery attempt outcome,
  and human response (if any).

---

## 10. System-Wide Configuration Management

Individual components have their own configuration mechanisms (Teensy has SD YAML +
EEPROM; ROS nodes have the parameter server). This section defines how configuration
is managed across the full system.

### 10.1 Single source of truth

The `sigyn_bringup` package is the single source of truth for initial system
configuration values that span more than one component. It contains:
- YAML files for each ROS 2 node.
- Template configuration files for Teensy boards (to be copied to SD cards).
- Documentation of every configurable parameter, its allowed range, and its default.

### 10.2 Parameter naming convention

All configurable parameters throughout the system must follow a naming convention:
`<subsystem>.<component>.<parameter>`. Examples:
- `safety.proximity.ring2_threshold_m`
- `battery.thresholds.critical_voltage_v`
- `motion.limits.max_linear_mps`

This convention must be followed in ROS 2 parameter YAML files and in Teensy SD
YAML files. The same key is used in the `sigyn_to_teensy` `/sigyn/config/set`
service, creating a consistent parameter surface across the system.

### 10.3 Runtime tuning policy

Parameters that affect safety thresholds must be tunable at runtime via the ROS 2
parameter server and the `sigyn_to_teensy` config service. However:

- Each safety parameter must have a validated range; values outside that range are
  rejected with a descriptive error.
- Changes to safety parameters at runtime must be logged in the audit trail.
- Safety-critical parameter changes require an explicit confirmation flag in the
  service request to prevent accidental modification.

---

## 11. Clock Synchronization and Timestamps

### 11.1 The clock problem

Sigyn has multiple time sources:
- ROS 2 system clock (`sigyn7900a`): NTP-synced wall clock; authoritative.
- Teensy `millis()`: milliseconds since board boot; no wall-clock anchor.
- Raspberry Pi clocks: may or may not be NTP-synced depending on network availability.

### 11.2 ROS time as the anchor

The ROS 2 system clock on `sigyn7900a` is the authoritative time source. All
published ROS 2 message headers carry this clock's timestamp.

### 11.3 Teensy clock anchoring via PC heartbeat

The `sigyn_to_teensy` package sends a `HEARTBEAT_PC` command to Board 1 at ≥ 1 Hz
carrying the current ROS nanosecond timestamp. Board 1 records this alongside its
local `millis()` value, establishing a mapping:

```
ros_time_ns ≈ last_received_ros_ns + (millis() - last_received_millis_ms) × 10⁶
```

Error is bounded by USB round-trip latency (< 5 ms) and Teensy 4.1 crystal drift
(< 50 ppm). All Teensy SD log entries must record both values.

### 11.4 Log correlation

When debugging a safety event, an engineer must be able to correlate the Teensy
SD card log (estimated ROS time), the PC safety event log (ROS timestamps), and
the ROS 2 bag file. The `HEARTBEAT_PC` timestamp must be written to the SD log on
every receipt to enable this correlation.

### 11.5 No hardware RTC required (now)

At present, Teensy clocks are anchored to ROS time on each connection. A power-off
event loses the anchor; after reboot the estimated ROS time is invalid until the
first `HEARTBEAT_PC` is received. This is acceptable for current use.

For the outdoor robot operating without a PC, a hardware RTC on each board, or a
GPS receiver providing both time and position, should be evaluated.

---

## 12. Privacy and Data Handling

### 12.1 Principles

- **Minimum necessary collection:** The robot collects only the sensor data required
  to complete its current task.
- **Local processing preference:** Object recognition, person detection, fall
  detection, and processing involving identifiable images or audio must run locally
  on the robot whenever possible. Raw image data must not leave the home network.
- **Human visibility:** The owner must always be able to see what the robot is
  capturing.
- **Portability:** The owner can export the safety event log and notification history.

### 12.2 SMS and email notification content

Notification payloads carry minimal data: tier, robot status, and a URL linking to
the in-app or web UI for details. No raw sensor data is transmitted. API keys for
SMS delivery services must be stored in a secrets file outside of version control.
The `sigyn_bringup` package must document the expected secrets file format.

### 12.3 Camera data

Camera images are not recorded to persistent storage by default. If recording is
enabled for a specific task (intruder alert), the data is stored locally with a
rotating buffer. The retention policy is configurable.

### 12.4 Voice interaction

When voice interaction is added, wake-word detection must run locally. Any cloud
speech-to-text API call must be opt-in and documented. Voice recordings must be
deleted after the command is processed.

---

## 13. System-Wide Testing Strategy

### 13.1 Testing levels

| Level | Scope | Tooling | When required |
|---|---|---|---|
| Unit tests | Individual class; no I/O or ROS 2 | Google Test, `pio test` | Every module and safety-critical class |
| Integration tests | Components communicating via mocked interfaces | Google Test, rclcpp | Every cross-component path |
| Node tests | Single ROS 2 node with mock topics/services | `launch_pytest` | Each ROS 2 node |
| System tests | Full robot launch; possibly Gazebo | `launch_pytest`, Gazebo | Each milestone |
| Manual tests | Physical hardware or human interaction | Test scripts in `docs/Testing.md` | Safety-critical end-to-end paths |

### 13.2 CI/CD requirements

- Unit and integration tests must pass in CI before any merge to `main`.
- CI has no Teensy hardware, no serial ports, and no ROS 2 runtime.
- Teensy firmware tests use the `[env:test]` PlatformIO environment on a Linux host.
- ROS 2 package tests use `colcon test` inside a Docker container with ROS 2 Jazzy.
- System tests run in a separate pipeline requiring Gazebo.

### 13.3 Simulation gap

A Gazebo simulation of Teensy board behavior is a high-priority development item.
Until it exists, system tests require physical hardware, and every safety fault
scenario must have a manual test procedure recorded in `docs/Testing.md`.

### 13.4 Test coverage requirement

No safety-critical code is merged without a test that would have caught a regression
in the covered path. Coverage metrics are context, not the primary measure; the
primary measure is whether every fault condition in the safety specification has a
corresponding test.

---

## 14. Outdoor Robot Design Portability

### 14.1 Portability definition

The goal is architectural reuse, not binary reuse. A new robot built from this
platform should require only:
- A new board configuration header and `main.cpp` for its Teensy boards.
- A new SD YAML configuration file.
- Possibly new module classes for robot-specific sensors or actuators.
- No changes to the common framework, safety coordinator logic, or message parsers.

### 14.2 Decisions with portability impact

| Decision | Portability note |
|---|---|
| Three-board partition | The framework must support any board count; avoid BOARD_ID == 1 conditionals in shared code. |
| USB serial per board | Outdoors, CAN bus or RS-485 may be more appropriate; `ISerialBridge` abstraction must accommodate this. |
| VL53L0X TOF sensors | `IProximitySensor` must be sensor-agnostic. |
| RoboClaw motor controller | `IRoboClaw` should generalize to any closed-loop motor controller. |
| Nav2 | Should require only map source and sensor driver changes for the outdoor robot. |
| Battery chemistry | The SoC model must be fully parameterized, not hard-coded for LiPo. |
| ROS time anchor | The outdoor robot may not have a PC connection; evaluate hardware RTC or GPS time. |

---

## 15. Work in Progress and Open Questions

The following specification documents are needed and do not yet exist:

| Document | Contents | Priority |
|---|---|---|
| `docs/specs/message_protocol.spec.md` | Complete versioned definition of all Teensy-to-PC messages; all ROS 2 topic/service/action interfaces | High |
| `docs/specs/behavior_trees.spec.md` | Behavior tree node library, safety integration points, and policy definitions | Medium |
| `docs/specs/navigation.spec.md` | Nav2 configuration rationale, ring-of-protection integration, localization requirements | Medium |
| `docs/specs/gripper_arm.spec.md` | Multi-DOF arm specification, safety integration, grasping requirements | Low (arm pending) |
| `docs/specs/charging.spec.md` | Autonomous charging station hardware and software specification | High (M3 in progress) |
| `docs/specs/human_notification.spec.md` | Notification tiers, contact list management, escalation policy, SMS API selection, iOS app architecture | High |

**Open questions at the system level:**

| ID | Question |
|---|---|
| OQ-S1 | In what order should the monorepo be split? (Proposed plan in Section 7.) |
| OQ-S2 | What deployment model is used for the outdoor robot? Is `Sigyn2` sufficient or does it need extension for multi-robot management? |
| OQ-S3 | Should the Jetson Nano Orin be integrated into the safety authority chain, or only provide perception data to `sigyn7900a`? Current answer: not in the safety chain until further design. |
| OQ-S4 | What is the charging station communication protocol? Does the docking Pi participate in the ROS 2 network or communicate via a simpler channel? |
| OQ-S5 | Is the multi-DOF arm going to use Board 3 or a separate embedded controller? Does it share an authority level with Board 1 safety? |
| OQ-S6 | What Twilio/AWS SNS account and cost model should be used? What is the fallback if the SMS API changes billing policy? |
| OQ-S7 | Should `sigyn_interfaces` be published as an open-source package with documentation, given that the project is intended to be shareable? |
| OQ-S8 | Should the robot have a local touch-screen or physical buttons for human notification response, in addition to the iOS app? The owner may not always have their phone. |
| OQ-S9 | What is the concrete hardware design for solid-state sense of the physical e-stop button state? Is a GPIO sense across the battery disconnect circuit practical? |
