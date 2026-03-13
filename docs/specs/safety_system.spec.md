# Sigyn Safety System Specification

**Document status:** Draft v0.1  
**Last updated:** 2026-03-02  
**Scope:** Safety behavior from Teensy embedded firmware through all ROS 2 nodes,
across the full Sigyn hardware platform  
**See also:** `docs/specs/sigyn.spec.md` (overall system specification)

---

## Table of Contents

1. [Purpose and Motivation](#1-purpose-and-motivation)
2. [Core Safety Goals](#2-core-safety-goals)
3. [Trust Model](#3-trust-model)
4. [Definitions](#4-definitions)
5. [Fault Data Model](#5-fault-data-model)
6. [Authority Chain](#6-authority-chain)
7. [Rings of Protection](#7-rings-of-protection)
8. [Monitored Conditions](#8-monitored-conditions)
9. [Safety Responses](#9-safety-responses)
10. [Human Notification](#10-human-notification)
11. [Communication Requirements](#11-communication-requirements)
12. [Watchdog Requirements](#12-watchdog-requirements)
13. [Testing Requirements](#13-testing-requirements)
14. [Recovery Procedures](#14-recovery-procedures)
15. [Open Questions](#15-open-questions)

---

## 1. Purpose and Motivation

Sigyn is designed to eventually perform life-critical functions: delivering medicine,
calling for help, detecting a fall, and serving as a personal assistant for an aging
individual living alone. A safety system that fails silently, produces false alarms,
or prevents normal operation is not merely inconvenient — it makes the robot useless
or dangerous. The safety system therefore has a dual mandate:

1. **Prevent harm** — to people, objects, and the robot itself.
2. **Maintain trust** — a person relying on Sigyn for critical tasks must be able to
   believe the robot will complete them safely, or will communicate clearly when it
   cannot.

These two goals are in tension. Overly aggressive safety responses erode trust by
stopping the robot unnecessarily. Insufficiently aggressive responses risk injury
or damage. The design must support explicit, auditable tradeoffs between the two.

---

## 2. Core Safety Goals

### 2.1 Prevent harm to people

People near the robot must not be injured. Sigyn is fast, heavy, and has sharp
edges. The system must:

- Maintain safe clearance from detected people during autonomous navigation.
- Move slowly and predictably when interacting with a person (e.g., delivering an
  object), so that the person can anticipate the robot's motion and react.
- Announce intended movement when interacting with a person, if a suitable
  communication channel is available.
- Treat undetected people as possible rather than impossible: areas where a person
  could reasonably be present should be treated with additional caution even if no
  person is currently detected.

### 2.2 Prevent harm to objects

The robot must not damage the environment, furniture, or objects it carries.

- Navigation safety margins must account for sensor limitations (see
  [Ring 1](#ring-1-navigation-margins)).
- Approach to objects for delivery or manipulation must be controlled and verifiable.
- Carrying fragile objects (medicine, beverages) requires additional care during
  acceleration and deceleration.

### 2.3 Prevent harm to the robot

Sigyn represents significant investment and is the platform for ongoing development.
The robot must protect itself from:

- Battery thermal runaway or deep discharge.
- Motor or controller thermal damage.
- Mechanical damage from collisions.
- Firmware crash or communication failure that leaves the robot in an unsafe state.

### 2.4 Remain operational for the owner

When the owner needs the robot and the robot stops for a safety reason, that is a
safety failure just as much as a collision is. The safety system must:

- Distinguish recoverable from unrecoverable conditions.
- Prefer graceful degradation over total stop where safe to do so.
- Always communicate its state clearly to any available notification channel.

---

## 3. Trust Model

### 3.1 What trust means in this context

A safety system earns trust by behaving consistently and predictably. The system
must:

- **Never silently fail.** Every fault must be logged, reported to a notification
  channel, and tracked until explicitly cleared.
- **Always report its state.** Even when no fault is active, the safety system must
  publish its current state so that a human or supervising node can verify that it is
  running.
- **Distinguish levels of certainty.** A sensor that is degraded but still partially
  functional should be treated differently from one that has failed completely.

### 3.2 Human override and "other situation" commands

The human owner may have information the robot does not. If the robot stops while
delivering medicine because a proximity sensor is degraded, the owner — who can see
the robot and the path is clear — may need to command the robot to proceed anyway.
This is a deliberate tradeoff between caution and utility.

The safety system must support:

- **Human override:** A command from an authorized human that acknowledges a specific
  active fault and permits operation to continue despite it. The fault must remain
  recorded; the override must be logged with a timestamp and the identity of the
  person issuing it.
- **"Other situation" command:** A free-form input from a human that provides context
  the robot cannot sense. Examples: "The path is clear," "I am present and watching."
  The safety system must accept and log these, and may use them to adjust behavior
  tree responses. The format and semantics of this command require further
  specification (see [Open Questions](#15-open-questions)).
- **Policy constraint:** A human override does not disable the hardware e-stop path
  (authority levels 0–1 in Section 6). Override applies only to the deliberative
  response path.
  
  NOTE: we may reconsider whether the user can override e-stop errors. Also, the user
  may have to be continuously in the loop for an override to occur. We need to look
  at this on a case-by-case basis but we should assume we need a parameter for each
  safety fault of interest indicating:  
  - Can the user override this fault.
  - A list of valid reasons the user must select from as to why it is being overriden. The reason should be logged.
  - A boolean indicating if the user must continuously override or can a single
  override action clear the fault.

### 3.3 Audit trail

Every safety event — activation, deactivation, override, acknowledgment — must be
logged with a timestamp, the source of the action, and any available human identity.
Logs must survive a robot power cycle (SD card on each Teensy board, and persistent
storage on the main PC).

---

## 4. Definitions

| Term | Meaning |
|---|---|
| **Fault** | A detected condition whose presence requires the safety system to take some action or change state. |
| **Urgency class** | How quickly a fault requires a response. Independent of severity. |
| **Severity** | How serious a fault is when fully evaluated. |
| **Latching fault** | A fault that persists after the triggering condition clears because it requires human acknowledgment or a specific recovery procedure. |
| **Auto-clearing fault** | A fault that deactivates automatically when the monitored condition returns to normal. |
| **Power-cycle fault** | A fault that requires a hardware component to be power-cycled before it can be cleared. |
| **E-stop** | Emergency stop. For motors, this means immediately disabling motor drive. |
| **Authority level** | The rank of an actor in the authority chain; lower numbers are higher priority and cannot be overridden by higher numbers. |
| **Ring** | A concentric zone around the robot used to categorize proximity to obstacles or people. |
| **Watchdog** | A hardware or software timer that must be periodically reset; if it expires, it triggers a recovery action. |
| **Human alert** | A notification sent via any available channel to one or more people. |
| **Board ID** | The integer identifier of a Teensy board (1 = Navigation/Safety, 2 = Power/Sensors, 3 = Elevator/Gripper). |

---

## 5. Fault Data Model

A fault is described by the following independent dimensions. All dimensions are
required for every fault event.

### 5.1 Identity

| Field | Type | Description |
|---|---|---|
| `board_id` | uint8 | Which Teensy board detected this condition (1, 2, or 3; 0 = PC-side). |
| `source_class` | string | Category of the detecting component (e.g., `BATTERY`, `TEMPERATURE`, `ROBOCLAW`, `VL53L0X`, `IMU`, `HEARTBEAT`). |
| `instance` | uint8 | Which physical instance reported the fault (e.g., sensor index 0–7 for VL53L0X; battery rail 0–4 for power monitors). |
| `description` | string | Human-readable summary of the specific condition. |

### 5.2 Urgency Class

Urgency describes how quickly a response must happen and which path the response
takes. It is independent of how serious the condition is.

| Class | Response deadline | Handling path |
|---|---|---|
| `HARDWARE` | < 1 ms | GPIO or hardware logic only; no firmware code in the path |
| `EMBEDDED` | < 10 ms | Teensy firmware; interrupt or tight loop |
| `REALTIME` | < 50 ms | Teensy firmware; main loop |
| `DELIBERATIVE` | < 500 ms | PC-side ROS 2 node |
| `ADVISORY` | < 5 seconds | Behavior tree, human notification |

A single physical condition may trigger faults at multiple urgency classes. For
example, a proximity sensor detecting an object at 15 cm while the robot is
moving triggers an `EMBEDDED` fault (the Teensy immediately signals the RoboClaw
e-stop pin) and an `ADVISORY` fault (the behavior tree learns about the obstacle
and decides whether to re-plan).

### 5.3 Severity

Severity describes how serious the condition is after full evaluation.

| Level | Value | Meaning |
|---|---|---|
| `INFORMATIONAL` | 0 | Telemetry of interest; no action required. |
| `WARNING` | 1 | Condition is approaching a threshold; action may be needed soon. |
| `DEGRADED` | 2 | A component is functioning below specification; some capabilities are reduced. |
| `CRITICAL` | 3 | The condition requires immediate corrective action. |
| `EMERGENCY_STOP` | 4 | Immediate motor stop required. |
| `SYSTEM_SHUTDOWN` | 5 | The entire system must be powered down. |

### 5.4 Lifecycle

| Field | Type | Default | Description |
|---|---|---|---|
| `auto_clear` | bool | true | The fault deactivates automatically when the condition resolves. |
| `requires_power_cycle` | bool | false | A hardware component must be power-cycled before the fault can clear. |
| `requires_human_ack` | bool | false | A human must explicitly acknowledge the fault before it clears. |
| `human_override_active` | bool | false | A human has issued an override for this fault. The fault remains recorded. |

### 5.5 Context

The following fields are logged with every fault activation and are used for
forensic analysis but do not change the response:

| Field | Description |
|---|---|
| `timestamp_ms` | Milliseconds since board boot (Teensy) or ROS 2 time (PC). |
| `robot_velocity` | Linear and angular velocity at the time of activation (if available from PC). |
| `active_behavior` | Name of the currently active behavior tree node (if available from PC). |
| `override_by` | Identity of the human who issued an override, if applicable. |

### 5.6 Fault message format

Every published fault event must carry all identity, urgency, severity, and lifecycle
fields. The exact message format is specified in `docs/message_protocol.spec.md`
(see [Open Questions](#15-open-questions) — this document does not yet exist).

---

## 6. Authority Chain

The authority chain defines who is permitted to take which actions. A lower authority
level cannot be overridden by a higher one. The PC cannot command the Teensy to clear
a fault that the Teensy itself has determined is still active.

| Level | Actor | Actions permitted | Typical latency |
|---|---|---|---|
| 0 | Physical e-stop button | Disconnects main battery power | Hardware |
| 1 | Board 1 GPIO output | Asserts RoboClaw e-stop pin | < 1 ms |
| 2 | Board 1 Teensy firmware | Issues RoboClaw stop commands; asserts inter-board fault GPIO; activates/clears its own faults | < 10 ms |
| 3 | Board 2 / Board 3 Teensy firmware | Signals Board 1 via inter-board GPIO or UART; activates its own local faults | < 50 ms |
| 4 | `sigyn_to_teensy` PC node | Sends serial commands to Teensy boards; publishes fault events to ROS 2 | < 200 ms |
| 5 | ROS 2 safety coordinator | Aggregates fault state; triggers behavior tree responses; initiates human alerts | < 1 s |
| 6 | Behavior tree | Cancels or re-plans goals; requests velocity constraints; requests recovery behaviors | seconds |
| 7 | Human operator | Issues override or "other situation" commands via any available interface | manual |

**Rules:**

- Any actor may activate a fault. Only the original activating actor — or an actor at
  a lower authority level — may deactivate it.
- Authority level 7 (human override) may override levels 3–6 but never levels 0–2.
  Hardware safety paths are never overridable by software or human command.
- A fault cleared by human override must be re-evaluated at the next detection cycle.
  If the condition is still present the fault is immediately re-activated.
- The PC must not assume that a command sent to the Teensy was acted upon. The
  Teensy must publish its resulting state; the PC verifies by reading that state.

---

## 7. Rings of Protection

Rings of protection define concentric zones around the robot. The robot's operating
mode is determined by which zones are occupied and whether the occupancy is expected.

> **Implementation note:** Rings 1 and 2 require fusion of sensor data with
> navigation intent and are therefore computed on the PC. Ring 3 requires an
> immediate hardware-speed response and is computed on the Teensy.
> Ring 0 corresponds to physical contact with the robot and is out of scope
> for software safety.

### Ring 1 — Navigation margins

**Outer boundary:** Configurable; typically 0.5–1.0 m from robot edge.  
**Purpose:** The Nav2 obstacle layer is configured to avoid entering this zone
during autonomous navigation.  
**Expected occupancy:** Never, under normal autonomous operation.  
**Unexpected occupancy:** The safety coordinator on the PC signals `ADVISORY`
severity to check whether the approach is intentional.

**Limitations acknowledged:** Sensors cannot provide complete coverage at this
distance. Transparent surfaces, low-profile obstacles, and sound-absorbing
materials may not be detected. These limitations must be documented per-sensor.

### Ring 2 — Caution zone

**Outer boundary:** Configurable; typically 0.15–0.5 m from robot edge.  
**Purpose:** The robot should not enter this zone unexpectedly. If it does,
velocity must be reduced.  
**Expected occupancy:** Docking, object delivery, intentional close approach.  
**Unexpected occupancy:** `DELIBERATIVE` urgency fault toward the behavior tree;
velocity cap applied by the ROS 2 safety coordinator; request for context from
the behavior tree about whether the approach is intentional.  
**Recovery:** If the robot cannot determine that the occupancy is intentional
within a configurable timeout, the robot stops and requests human input.

### Ring 3 — Emergency stop zone

**Outer boundary:** Configurable; typically 0–0.15 m from robot edge.  
**Purpose:** The robot must stop immediately.  
**Expected occupancy:** Required for some docking operations with explicit
`allow_close_approach` flag asserted by the behavior tree.  
**Unexpected occupancy:** `EMBEDDED` urgency fault; motors stopped immediately.  
**Recovery:** Requires explicit human acknowledgment or assertion of
`allow_close_approach` by the behavior tree.  

> **Open question:** Should the Teensy signal the ring 3 fault to the PC via the
> existing serial channel, or via a dedicated GPIO line for minimum latency?
> GPIO is faster but requires board redesign or an available pin. (See
> [Open Questions](#15-open-questions).)
> 
> ANSWER: penetration of rings 2 and 3 should be signaled to the PC. We may
> enforce a speed limit which is different for rings 2 and 3. My note below
> indicates we may just stop there as an enforcement, but we might decide to
> e-stop the robot for ring 3. Let's plan on an enforced slowdown with the 
> option to stop. Like many parameters, I suggest we initialize them in the
> program from either a YAML file, or implement a database, and then allow
> the user to use, e.g., rqt to change the parameters at run time. 
> Note that nothing we can do can guarantee that the PC will be able to notice
> a signal in time nor react to it, so whatever the policy is should be delegated
> to the Teensy to handle. The PC can perhaps override a policy decision for ring
> 3, if we provide an API, but it can't be the main vector of response.

NEW FEATURE NOTE (future possibility):
Read configuration values from a YAML file the first time, then persist the values
somewhere. On the Teensy, there is an EEPROM. Then provide a hook so that when the
user dynamically changes them, the changes propagage to the EEPROM. Expose the
parameters via a sigyn_to_teensy hook. Have an option to reset from YAML.

Note: 0.15m is too far for a 3rd ring of protection if the robot is allowed to
travel at 1 meter/second. My thinking is that the 3rd ring would be more like 0.075 m
or maybe as much as 0.1m. So we may need to intervene at ring 2 and not allow to robot
to travel above a certain speed. In fact, that's probably an idea here too--rather than
forcing a stop, we may really limit the speed so that if anything is hit it will be
at a fairly slow speed, but we would still allow travel so that there is a path to
recovery, that is the robot could slowly back away still.

---

## 8. Monitored Conditions

### 8.1 Power monitoring

**Board:** 2 (5 INA226 sensors: main battery + 4 DC-DC rails: 24 V motor, 12 V PC,
5 V logic, 3.3 V auxiliary)

| Condition | Urgency | Severity | Lifecycle |
|---|---|---|---|
| Battery voltage below warning threshold (34.0 V) | ADVISORY | WARNING | auto-clear |
| Battery voltage below critical threshold (32.0 V) | REALTIME | EMERGENCY_STOP | auto-clear |
| Battery current above warning threshold (configurable) | DELIBERATIVE | WARNING | auto-clear |
| Battery current above critical threshold (configurable) | REALTIME | CRITICAL | auto-clear |
| DC-DC rail voltage out of tolerance (configurable %) | DELIBERATIVE | DEGRADED | auto-clear |
| DC-DC rail current above threshold (configurable) | DELIBERATIVE | WARNING | auto-clear |
| Power sensor I²C communication failure | REALTIME | DEGRADED | auto-clear |
| Battery hardware disconnect dongle triggered | REALTIME | SYSTEM_SHUTDOWN | latching |
| Battery remaining time below return-to-charger threshold | ADVISORY | WARNING | auto-clear |

**Additional requirements:**

- The battery monitor must maintain a rolling model of voltage and current draw
  to estimate remaining time before the battery must begin charging. The model must
  account for the distance and time required to return to the charging station.
- Per-cell battery data is not available with the current battery. When the battery
  is replaced, per-cell monitoring should be added if the new battery supports it.
- A battery temperature sensor must be added. Until it is present, the absence of
  battery temperature monitoring must be flagged as a `DEGRADED` system capability.

NOTE: I'm not sure if we want lack of temperature to be marked as DEGRADED. Let's
consider making that a boolean parameter option. It is hard to tell if a temperature
sensor is not available vs. detecting a bad reading. Well, maybe a pullup resistor would
solve the issue. If so, consider addressing this in the next PC board rework. Also,
add that we would like the PC side to occassionally ask the global planner to compute
a path back to the charger and use that to make an estimate of how far we are from the charger, maybe
both in distance and time, and send that as part of one of the regular messages from the PC to the teensy.
It's unclear, then, who makes the determination of when to return to charger. board2 has the voltage and
current data, but board1 is probably running a more sohphisticated safety system. Maybe we add and
api from board2 to send the current voltage/current/charging-discharging/percent-charge info to board1.
That would be part of the new UART communication to be added for inter-board communication.

### 8.2 Temperature monitoring

**Board:** 2 (analog TMP36 sensors at motor positions and other thermal points)

| Condition | Urgency | Severity | Lifecycle |
|---|---|---|---|
| Motor temperature above warning threshold (configurable) | DELIBERATIVE | WARNING | auto-clear |
| Motor temperature above critical threshold (configurable) | REALTIME | EMERGENCY_STOP | auto-clear |
| RoboClaw temperature above warning threshold | DELIBERATIVE | WARNING | auto-clear |
| RoboClaw temperature above critical threshold | REALTIME | CRITICAL | auto-clear |
| Temperature sensor read failure | REALTIME | DEGRADED | auto-clear |
| Battery temperature above warning threshold | DELIBERATIVE | WARNING | auto-clear |
| Battery temperature above critical threshold | REALTIME | SYSTEM_SHUTDOWN | latching |

**Recovery:** For motor and controller overtemperature, recovery is to stop all
movement and wait for temperature to fall below the auto-clear threshold. Recovery
behavior (navigate to a cooler location, increase ventilation if present) is the
responsibility of the behavior tree, not the safety system.

### 8.3 Proximity monitoring

**Board:** 1 (8× VL53L0X time-of-flight sensors, placed around the robot perimeter)

| Condition | Urgency | Severity | Lifecycle |
|---|---|---|---|
| Object detected in ring 3 zone (unexpected) | EMBEDDED | EMERGENCY_STOP | auto-clear |
| Object detected in ring 2 zone (unexpected) | DELIBERATIVE | CRITICAL | auto-clear |
| Individual sensor timeout or read failure | REALTIME | DEGRADED | auto-clear |
| ≥ 50% of sensors failed simultaneously | REALTIME | EMERGENCY_STOP | latching |

**Per-sensor identity:** Each fault identifies the board, source class
(`VL53L0X`), and sensor instance (0–7).

**Direction-aware detection:** Ring 2 and ring 3 faults should be qualified by whether
the obstacle is in the direction of current robot movement. An obstacle behind the
robot while the robot moves forward should not trigger an e-stop. Computing
direction-aware faults requires the current velocity vector, which is available from
Board 1 odometry. The specification of how this is computed remains open.

NOTE: The PC sends cmd_vel information which is passed on to the RoboClaw. That can
also be used to determine direction of movement. We should also get from the PC pose information
and LIDAR information at least in the expected direction of movement so we can determine if the robot 
is moving as expected. If the wheels are turning, wheel odometry will say the robot is moving
but if the LIDAR indicates the robot is not moving as expected, we know the wheels are spinning.
The cmd_vel can also tell us if we are about to imminently enter ring 2 or ring 3.
And, given the incomplete obstacle coverage by the sensors, we might use the complete LIDAR,
after doing our own tf operation, along with the TOF sensors to compute our own sense of 
obstacle distances to figure what ring the robot is in. Also note that the LIDAR to PC
to Teensy pathway may have too much latency for fast response, so the safety code may
rely quite a bit on prediction.

**Replacement note:** The VL53L0X sensors may be replaced with sensors providing an
8×8 detection window. This specification is intended to be sensor-model-agnostic.

### 8.4 Motor and RoboClaw monitoring

**Board:** 1

| Condition | Urgency | Severity | Lifecycle |
|---|---|---|---|
| Motor overcurrent (either motor) | EMBEDDED | EMERGENCY_STOP | requires-power-cycle |
| Motor runaway (encoder failure) | EMBEDDED | EMERGENCY_STOP | requires-human-ack |
| Robot spinning in place (motors running, no pose change) | DELIBERATIVE | CRITICAL | requires-human-ack |
| RoboClaw communication timeout | REALTIME | EMERGENCY_STOP | auto-clear |
| 24 V motor power rail failure | REALTIME | EMERGENCY_STOP | auto-clear |
| 5 V logic power rail failure | REALTIME | DEGRADED | auto-clear |
| RoboClaw API version mismatch | EMBEDDED | EMERGENCY_STOP | latching |

**Spin-in-place detection** requires pose data from the main PC (LIDAR-based
localization). The Teensy sends wheel encoder odometry; the PC compares with LIDAR
odometry to detect discrepancy. This fault originates at the PC (authority level 4).

NOTE: There are several mentions about using LIDAR data. LIDAR data may take too long to send
at the LIDAR's (current) 10 Hz rate with range in mm and 450 points per scan. We may need to move
all LIDAR-based safety computation to the PC side. Also note that when using the RoboClaw,
if the wheel encoder signal disappears, there doesn't appear to be an open-loop option
for PID control, so e-stop with human intervention may be the only option. I need to verify
that there isn't an open-loop PID fallback.

**Power-cycle recovery for overcurrent:** The RoboClaw must be power-cycled after
an overcurrent fault. The safety system must support commanding the relay that
controls the 24 V motor power rail (relay on Board 1, pin `PIN_RELAY_ROBOCLAW_POWER`,
currently GPIO 31) to cycle the RoboClaw power. The command sequence is:
1. De-assert RoboClaw e-stop pin.
2. Open relay.
3. Wait configurable cool-down period.
4. Close relay.
5. Allow RoboClaw to re-initialize.

### 8.5 IMU monitoring

**Board:** 2 (BNO055)

| Condition | Urgency | Severity | Lifecycle |
|---|---|---|---|
| Tilt angle exceeds warning threshold (configurable, e.g. 15°) | DELIBERATIVE | WARNING | auto-clear |
| Tilt angle exceeds critical threshold (configurable, e.g. 25°) | REALTIME | EMERGENCY_STOP | auto-clear |
| Robot-fallen-over detection (tilt > 60° or configurable) | REALTIME | EMERGENCY_STOP | requires-human-ack |
| Angular rate suggesting imminent tip-over | DELIBERATIVE | CRITICAL | auto-clear |
| IMU calibration status degraded | ADVISORY | WARNING | auto-clear |
| IMU communication failure | REALTIME | DEGRADED | auto-clear |

**Predictive tilt fault:** If the IMU tilt angle and its rate of change together
suggest the robot will exceed the critical threshold within a configurable time
window, a predictive `CRITICAL` fault should be raised before the threshold is
crossed. This is aspirational for the initial implementation.

### 8.6 Heartbeat and watchdog monitoring

| Condition | Urgency | Severity | Lifecycle |
|---|---|---|---|
| Board 2 heartbeat timeout (> configurable, e.g. 2 s) | DELIBERATIVE | DEGRADED | auto-clear |
| Board 3 heartbeat timeout (> configurable) | ADVISORY | WARNING | auto-clear |
| PC serial heartbeat timeout from Board 1's perspective | REALTIME | WARNING | auto-clear |
| ROS 2 `sigyn_to_teensy` heartbeat timeout (from PC perspective) | DELIBERATIVE | DEGRADED | auto-clear |
| Main PC unresponsive (> configurable, e.g. 5 s) | DELIBERATIVE | CRITICAL | auto-clear |
| Teensy hardware watchdog expiry (board reset) | REALTIME | EMERGENCY_STOP | latching |

**Consequence of Board 2 failure:** Board 2 controls power monitors and IMUs. If
Board 2 goes offline, the robot loses power monitoring and tilt detection. The
conservative response is to navigate to the charging station and await human
intervention. An immediate e-stop is not required unless another condition
triggers it.

**Consequence of Board 3 failure:** Board 3 controls the elevator and gripper.
Failure should stop elevator and gripper movement but not stop the robot's
mobility. The safety system raises a `WARNING` and notifies the human.

### 8.7 Sensor health monitoring

Every sensor module must actively report its own health status to the safety system
on the same board. Polling a sensor for its health status is not acceptable; a
sensor that has silently stopped updating must be flagged automatically.

**Requirements for every sensor module:**

- Must signal a `DEGRADED` fault if its reading is more than a configurable period
  stale (the module detects it has not been able to update).
- Must distinguish between "no data yet since boot" (not a fault) and "data was
  updating and has stopped" (a fault).
- Must report which board and instance is affected.
- Must clear the fault automatically when updates resume.

---

## 9. Safety Responses

See also the Open Notes secton that talks about multiple sensor degredations.

### 9.1 Motor e-stop

**Trigger:** Any `EMBEDDED` urgency fault of severity `EMERGENCY_STOP` or higher
on Board 1.  
**Mechanism:** Board 1 asserts the RoboClaw hardware e-stop pin (active-LOW to
the RoboClaw; the Teensy drives it LOW, which means the Teensy GPIO output is
effectively the inverse).  
**Effect:** RoboClaw stops driving the motors. Whether motors coast or brake is
determined by RoboClaw configuration (to be documented separately — see
[Open Questions](#15-open-questions)).  
**Recovery:** Cleared only when all `EMBEDDED` `EMERGENCY_STOP` faults on Board 1
are resolved and any required recovery procedure (power cycle, human ack) is
complete.

### 9.2 Software motor stop

**Trigger:** `REALTIME` urgency faults of severity `CRITICAL` or higher.  
**Mechanism:** Board 1 sends a stop velocity command to the RoboClaw.  
**Effect:** Motors decelerate to zero under RoboClaw control. Less abrupt than
an e-stop.  
**Distinction from e-stop:** The software stop is intentionally less aggressive. The
RoboClaw motor drivers remain energized; motion can resume with a velocity command
as soon as the fault is cleared.

### 9.3 Velocity constraint

**Trigger:** Ring 2 occupancy, `DELIBERATIVE` faults, or human override in progress.  
**Mechanism:** The ROS 2 safety coordinator publishes a maximum linear and angular
velocity constraint. The twist multiplexer or Nav2 cost function must honor it.  
**Configurable parameters:** Maximum linear velocity, maximum angular velocity,
ramp-down time.

### 9.4 Behavior tree signal

**Trigger:** Any `DELIBERATIVE` or `ADVISORY` fault.  
**Mechanism:** The ROS 2 safety coordinator publishes the fault to a dedicated
reliable-QoS topic. The behavior tree subscribes and reacts per its programmed
policies.  
**The safety system does not dictate behavior tree policy.** It is the source
of safety state information; what the behavior tree does with that information
is outside this specification.

### 9.5 Power rail control

**Trigger:** Overcurrent fault requiring power cycle (`requires_power_cycle = true`)
on a component connected to a controllable relay.  
**Mechanism:** Board 1 controls solid-state or mechanical relays:
- GPIO 31 — RoboClaw motor power relay
- GPIO 32 — Main battery relay (use with extreme caution)  

NOTE: Actually, operating a solid state relay may require going through one of the level shifters in order to guarantee enough current drive. That would change the pin number.

**Safety rule:** The main battery relay may only be opened by the safety system
when a `SYSTEM_SHUTDOWN` fault is active. The RoboClaw relay may be cycled for
overcurrent recovery.

### 9.6 System shutdown

**Trigger:** `SYSTEM_SHUTDOWN` fault active with no available recovery path.  
**Mechanism:** Board 1 opens the main battery relay after signaling all other
boards and the PC to prepare for shutdown.  
**Sequence:**
1. Immediately assert motor e-stop.
2. Broadcast shutdown-imminent fault to all boards and PC.
3. Wait a configurable brief period for systems to log final state.
4. Open main battery relay.

---

## 10. Human Notification

### 10.1 Principles

- Notification is non-interruptible by the safety system itself. A fault that
  fires continuously must not flood the notification channel.
- Notification must be persistent if the channel supports it. A text message cannot
  be "un-sent." Log the fact that a notification was sent.
- Multiple notification modalities may be used simultaneously. The most reliable
  modalities should be sent first.
- Notification must include: what happened, when it happened, where the robot is
  (if available), and what the expected human action is (if there is one).

### 10.2 Notification tiers

#### Tier 1 — Embedded audio (buzzer on Teensy Board 1)

- Always available; requires no PC, network, or external service.
- Urgency-coded tone patterns (to be defined; e.g., slow beep = WARNING, fast beep
  = CRITICAL, continuous = EMERGENCY).
- Hardware requirement: a piezo buzzer on a spare GPIO output of Board 1.

#### Tier 2 — PC audio and display

- Requires main PC to be functional.
- Audio output: alarm tones and optionally text-to-speech synthesis describing
  the fault.
- Display: status panel on any connected display or web interface.
- Triggered by the ROS 2 safety coordinator publishing to the `/sigyn/human_alert`
  topic.

#### Tier 3 — Remote text and email notification

- Requires network connectivity.
- Preferred channels: direct SMS via T-Mobile carrier in the US without a
  third-party gateway (investigate SMTP-to-SMS gateways at `tmomail.net`);
  email notification via a locally configured SMTP relay; native Android/SMS
  support if other options are unavailable.
- Aspirational: iPhone app (to be explored; may require a small companion app
  or use push notification services).
- A 4G cellular dongle is an acceptable fallback for offline network access.
  This should be considered optional hardware.
- A configurable contact list of recipients with per-recipient notification policy
  (e.g., some events wake the owner, others wait until morning).

#### Tier 4 — Autonomous approach and announcement

- Requires the robot to be mobile and a person to be detected (OAK-D camera or
  similar).
- The safety coordinator publishes an alert; the behavior tree decides whether to
  navigate toward the detected person and use TTS to describe the situation.
- This is a behavior tree responsibility, not a safety system responsibility.
  The safety system provides the trigger.

### 10.3 Human_alert message fields

The `/sigyn/human_alert` ROS 2 message must include at minimum:

| Field | Description |
|---|---|
| `fault_identity` | Board ID, source class, instance |
| `severity` | Fault severity level |
| `urgency` | Fault urgency class |
| `description` | Human-readable text |
| `timestamp` | When the condition was detected |
| `robot_location` | Most recent known robot pose (if available) |
| `requested_action` | What the human is being asked to do (if anything) |
| `timeout_s` | How long before the safety system takes action without a response (0 = no timeout) |

---

## 11. Communication Requirements

### 11.1 Channel between Teensy and PC

All three Teensy boards communicate with the main PC (`sigyn7900a`) via USB serial.
Each board has a dedicated USB connection, identified by udev symlinks:
`/dev/teensy_sensor` (Board 1), `/dev/teensy_sensor2` (Board 2),
`/dev/teensy_gripper` (Board 3).

A second USB serial interface (on the same physical connection) is not available
without hardware redesign. The single channel per board is a fixed hardware
constraint.

### 11.2 Message priority

Because all message types from a given board share a single channel, the Teensy
must implement a **prioritized transmission queue**:

| Priority | Message category | Examples |
|---|---|---|
| P0 — immediate | Fault activation/deactivation | FAULT messages |
| P1 — high | Safety status | ESTOP status, heartbeat |
| P2 — normal | Commanded responses | Velocity acknowledgment |
| P3 — periodic | Sensor telemetry | BATT, IMU, ROBOCLAW, VL53L0X |
| P4 — background | Diagnostics and performance | PERF, DIAG |

No P3 or P4 message may delay a P0 message by more than one transmission slot.
The PC-side receiver must drain P0 and P1 messages before processing P3 and P4.

The current Teensy `Module` architecture already enforces timing budgets per module.
The same mechanism should be extended to flag when the transmission queue exceeds
a depth threshold, so that systemic bandwidth problems are detected and logged rather
than silently causing message loss.

### 11.3 Message versioning

The serial message protocol must be versioned. The requirements are:

- Each message type has a version field (e.g., `"v":1`).
- When a Teensy board connects, it must broadcast a protocol capability announcement
  describing the version of each message type it supports.
- The PC node must verify version compatibility on connect. A version mismatch must
  be logged as a `DEGRADED` fault and the specific message type must be disabled
  rather than silently misinterpreted.
- Version numbers increment whenever a field is added, removed, renamed, or
  changes type.
- A companion document (`docs/specs/message_protocol.spec.md`) must define the
  current version of every message type in one place. This document does not yet
  exist and is on the work list.

### 11.4 Latency requirements

| Path | Maximum acceptable latency | Measurement point |
|---|---|---|
| Teensy fault detection → e-stop pin asserted | 1 ms | Board 1 internal |
| Teensy fault detection → serial transmission started | 5 ms | Board 1 internal |
| Serial byte received at PC → published to ROS 2 topic | 20 ms | PC node |
| ROS 2 safety topic published → behavior tree reacts | 100 ms | ROS 2 |
| Human alert triggered → tier 1 sound starts | 50 ms | Board 1 internal |
| Human alert triggered → tier 2 audio starts | 500 ms | PC |

These are targets. The system must include instrumentation capable of measuring
each path and logging violations. Persistent violations indicate a design or
implementation problem that must be investigated.

### 11.5 PC-side threading model

The ROS 2 node that interfaces with the Teensy serial ports must define an explicit
threading model. The minimum requirements are:

- A dedicated reader thread per serial port that reads raw bytes and enqueues
  complete messages.
- A separate parser thread per board that removes complete messages from the queue,
  validates them, and dispatches to registered handlers.
- Safety-critical handlers (fault events, e-stop status) must execute in the parser
  thread with a specified maximum execution time.
- Slow handlers (telemetry publication, logging) must not block the parser thread.
- The threading model must be documented. Any change to the model must be reviewed
  against the latency requirements in Section 11.4.

### 11.6 SD card logging

Each Teensy board with an SD card must log all safety events to the SD card with
microsecond-precision timestamps. The logging requirements are:

- Logging must not block the main loop by more than 1 ms per event in normal
  operation.
- Log rotation and file management must not cause I/O spikes that violate timing.
- On startup, the existing log must be preserved and a new session record started.
- The PC receives all safety events via serial and persists them to its own
  storage as the authoritative archive. The SD card log is a local backup for
  when the PC cannot be reached.

---

## 12. Watchdog Requirements

### 12.1 Hardware watchdog (Teensy internal)

Each Teensy 4.1 has a built-in hardware watchdog timer. The watchdog is currently
disabled. Requirements:

- The watchdog must be enabled on all production builds.
- Each module's `loop()` call must "pet" the watchdog (reset the timer) after
  completing its work. If any module hangs, the watchdog expires and resets the board.
- The reset reason must be stored in non-volatile memory and reported via the
  protocol capability announcement when the board reconnects.
- After a watchdog reset, the board must re-enter a safe state with motors stopped
  before reinitializing normal operation.

### 12.2 Software watchdog (inter-system heartbeat)

Each board must publish a heartbeat at a minimum configured frequency. Any system
component that depends on another must monitor its heartbeat.

**Heartbeat policy per dependency:**

| Dependency lost | Expected response | Severity |
|---|---|---|
| Board 2 heartbeat | Navigate to charger; alert human | DEGRADED |
| Board 3 heartbeat | Halt gripper/elevator; alert human | WARNING |
| PC serial heartbeat (Board 1 perspective) | Continue for configurable window; then soft-stop | WARNING → CRITICAL |
| `sigyn_to_teensy` watchdog (PC perspective) | Alert human; attempt node restart | DEGRADED |
| Board 1 serial heartbeat (PC perspective) | Alert human; do not command motors | CRITICAL |

---

## 13. Testing Requirements

### 13.1 Principles

Testing is a first-class design requirement, not an afterthought. Every safety
component must have unit tests exercisable without connected hardware.

### 13.2 Teensy firmware testing

- **Dependency injection (DI) is required for all production safety paths.** Every
  hardware interface (GPIO read/write, I²C, UART, timer) must be behind a C++
  interface that can be replaced with a mock in tests.
- **All fault activation/deactivation logic must be testable** by injecting mock
  sensor values and verifying that the correct fault events are generated.
- **The hardware e-stop output must be testable** by injecting a mock GPIO interface
  and verifying the pin state at the appropriate urgency levels.
- **Timing behavior must be testable** by injecting a mock time source.
- Tests must run via `pio test -e test` on the development host without any
  connected hardware.
- Every fault condition listed in Section 8 must have at least one test case
  covering: (a) normal→fault transition, (b) fault→normal recovery, (c) fault
  persistence when `auto_clear=false`.

### 13.3 PC-side ROS 2 testing

- Safety-critical ROS 2 nodes must have unit tests that run without a live ROS 2
  runtime (constructor-injectable dependencies for serial channels, ROS 2 clock,
  notification channels).
- Integration tests must cover the full path from Teensy serial message to ROS 2
  topic publication and to behavior tree signal.
- The threading model (Section 11.5) must have tests that verify ordering guarantees
  under simulated load.

### 13.4 Simulation testing

A Gazebo simulation of the Teensy boards' behavior (receiving commands, publishing
sensor data, publishing faults) is desirable but not required for a first
implementation. When Gazebo simulation is added, it must be possible to inject
fault scenarios (board disconnection, overcurrent, ring 3 proximity trigger) and
verify end-to-end safety response.

---

## 14. Recovery Procedures

Recovery procedures define the state machine that transitions from a fault state
back to normal operation. For each lifecycle type:

### 14.1 Auto-clearing faults

1. Condition is detected → fault activated.
2. System takes the appropriate urgency-level response.
3. Condition resolves → module calls `deactivateFault`.
4. If no other fault of the same or higher severity is active for that response
   path, the response is withdrawn (e.g., e-stop cleared, velocity constraint
   lifted).
5. Fault is recorded in the log with duration and resolution time.

### 14.2 Latching faults (requires human acknowledgment)

1. Condition is detected → fault activated.
2. Human alert is sent.
3. Human investigates and resolves the underlying cause manually.
4. Human submits acknowledgment via any available interface (app, web UI, physical
   button, text reply — to be specified).
5. Safety coordinator verifies the triggering condition is no longer present.
6. Fault is deactivated; acknowledgment and timestamp are logged.

### 14.3 Power-cycle faults

1. Condition is detected → fault activated (e.g., RoboClaw overcurrent).
2. Human alert is sent.
3. The safety system executes the power-cycle sequence for the affected component
   (see Section 9.5) if automatic recovery is authorized.
4. The component re-initializes.
5. The safety system verifies that the fault condition is gone.
6. Fault is deactivated and logged.

### 14.4 System shutdown recovery

System shutdown (main battery relay opened) requires human physical intervention
to power back on. After power-on, all boards perform watchdog-reset-style startup
(Section 12.1). The system logs the shutdown event and presents it on next connect.

---

## 15. Open Questions

The following questions must be answered before the corresponding parts of this
specification can be finalized or implementation can begin.

| ID | Question | Affects |
|---|---|---|
| OQ-1 | What is the exact behavior of the RoboClaw when the e-stop pin is asserted — coast or brake? Can this be configured per command? | Section 9.1 |

ANSWER: Still open. Should not effect design. We assume worst case until we get a clarifying answer.

| OQ-2 | What is the format and delivery mechanism for the human "other situation" command? Is it a text message reply, a physical button press, a web UI interaction, or all of the above? | Section 3.2 |

ANSWER: When the user selects "OTHER", he will enter a short text. The mechanism depends on how the message was presented to the human. E.g., whether via an App, or a web page link, etc.

| OQ-3 | What is the GPIO pin budget on Board 1 for inter-board fault signals and for a ring 3 fast-path GPIO output toward the PC? | Section 7 |

ANSWER: There are a small number of available pins. The will be assigned once we have an inventory of the needs. The expectation is that I have already determined what pins will be actually used.

| OQ-4 | Should ring 2 occupancy be computed on the Teensy (low latency, limited context) or on the PC (full context, higher latency)? | Section 7 |

ANSWER: Probably both sides. It will just be an OR situation, if both sides participate.

| OQ-5 | What is the exact direction-aware proximity algorithm for ring 2 and ring 3? Does it use robot chassis velocity, or the Nav2 planned path direction? | Section 8.3 |

ANSWER: Unsure. I'm thinking the PC will do this.

| OQ-6 | What is the iPhone app strategy? Native app, web app accessed from Safari, or push notification via APNS using a small relay server? | Section 10.2 |

ANSWER: A native app will be persued. The fall back strategy will be a websocket page. There will be redundant paths, if possible, like e-mail and text notification and response.

| OQ-7 | Does the Jetson Nano Orin have a safety role (e.g., as a vision-based ring 1 or ring 2 checker)? | Section 7 |

ANSWER: Unsure. Assume NO until we further implement the code.

| OQ-8 | Should the April Tags Pi that will control the docking station participate in the safety system (e.g., signal "charging bay is clear")? | Section 8.6 |

ANSWER: Probably not. It will probably all be handled at the behavior tree level. The behavior tree might send a message that charging is believed to be connected, or charging is believed to be disconnected. On the Teensy side, we should consider verifying the assumption using the battery power monitor to detect charging vs discharge.

| OQ-9 | What is the authorized human acknowledgment interface? Physical button location, text command syntax, web UI path? | Section 14.2 |

ANSWER: All of the above. To be implemented. It is expected that notifications will be send via multiple paths. And redundant replies via multiple paths may be received.

| OQ-10 | For spinning-in-place detection, what is the maximum acceptable discrepancy between wheel odometry and LIDAR odometry before triggering a fault? | Section 8.4 |

ANSWER: That will be by configuration after we have done real experiments. This will likely be done on the PC side since it involves LIDAR calculations (see Open Notes).

## 16. Open Notes
There currently is no design to deal with stair or cliffs. We could add a note sometime about how we would
ancitipate dealing with that. This is because Sigyn doesn't have that issue, but it would be nice
for someone looking at this code to see how to add that support. Cliffs, especially, probably fall into
ring 3 handling. Maybe also ring 2 if they can be detected from far enough away.

Note currently addressed is how we deal with multiple degredations. It's one thing to deal with some number of
time-of-flight sensors failing, but what if we then have the OAK-D camera fail? Can we live with a LIDAR
failure if we have the OAK-D detecting obstacles? Remember that we need the PC side to be computing safety
state as well and informing the Teensy side as the Teensy side is the ultimate safety arbiter. That means
the class of device (e.g. temperature, VL53L0X) should probably be open rather than an enumeration. We can
specify the maximum device name length, though.

We need to fully define interboard messages and UART baud rates to determine the latency we can expect
when signaling a fault. For example, if the IMU detects a pending tip over, will UART signaling be
fast enough?  Or do we rely on making such things a GPIO e-stop signal? Consider how interboard messages
are handled by the hardware -- are the software queues big enough, do we need character by character 
interrupt handling instead of the built in UART handling?

I need to look into dedicated API (like Twilio, SendGrid, or AWS SNS) or a hardware 4G LTE/Cat-M1 module on the PC/Teensy for reliable SMS, rather than carrier email gateways. The Tmobile message gateway is being deprecated and may not be
reliable enough for life-threatening messaging. We may need redundant message paths. 
Also look into how to deal with 911 calls -- are there any local regulations or fees? 
Should we hard-code the GPS location of the house? Should we add an override if there is an on-board GPS sensor?

Consider passing the ROS2 date and time to the Teensy at least in the heartbeat messages. The SD log should then log the millisecond clock and the best guess of the ROS time as well. This makes coordination with ROS logs easier.

The PC should report the behavior tree node name for aciton nodes, but we can probably ignore the other node types. This gets uses as part of the forensic logs, especially for the safety system. The node name should be written to the board1 SD log every time it is received and the last received node name is recorded during safety logging. When an action node reports success, perhaps we clear the node name. This requires further though to be sure that non-action node names don't need to be reported. Consider that the behavior tree will be ticked at 100 Hz (currently) and we don't want to flood the communication channel.

Consider when a software motor stop command is sufficient instead of an e-stop. In particular, software stops will likely include decelleration data while e-stops are probably abrupt and more dangerous, possibly causing the robot to fall over.

We may need a behavior tree node to signal intent to the safety system. For instance, if the robot must be very close to an object in order to do a task, the safety system needs to know that a ring of protection is being violated on purpose.

The Teensy should publish, either periodically or when requested, certain hard-coded limits that should be respected by the PC side. An e.g. would be the maximum linear velocity or angular velocity. We might want to break that down even further for, e.g., different limits within different rings of protection. This should be signaled to, e.g., Nav2 and my rewrite of the bluetooth joystick controller and the keyboard teleop node.

The message versioning scheme could be moved to just the class version, rather than a per-message version. Consider using the last-modified date available from the compiler as the version number. The sigyn_to_sensor side needs to agree on the version number before communication can continue.

Consider sending a message to the Teensy with the PC side is shutting down. It will allow the Teensy to flush log buffers.