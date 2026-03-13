# Teensy Board Firmware Specification

**Document status:** Draft v0.1  
**Last updated:** 2026-03-09  
**Scope:** Firmware architecture and requirements for all Teensy 4.1 boards in the
Sigyn robot platform — covering the common framework, per-board responsibilities,
inter-board communication, safety integration, configuration management, build
system, and test requirements  
**See also:**
- `docs/specs/sigyn.spec.md` (overall system specification)
- `docs/specs/safety_system.spec.md` (safety behavior specification)
- `docs/specs/wr_ros_teensy.spec.md` (PC-side bridge and safety coordinator)

---

## Table of Contents

1. [Purpose and Motivation](#1-purpose-and-motivation)
2. [Design Philosophy](#2-design-philosophy)
3. [Board Partition](#3-board-partition)
4. [Common Framework](#4-common-framework)
5. [Board 1 — Navigation and Safety](#5-board-1--navigation-and-safety)
6. [Board 2 — Power and Sensors](#6-board-2--power-and-sensors)
7. [Board 3 — Elevator and Gripper](#7-board-3--elevator-and-gripper)
8. [Inter-Board Communication](#8-inter-board-communication)
9. [PC Serial Bridge Protocol](#9-pc-serial-bridge-protocol)
10. [Safety Integration](#10-safety-integration)
11. [Configuration Management](#11-configuration-management)
12. [Hardware Watchdog](#12-hardware-watchdog)
13. [Build System](#13-build-system)
14. [Testing Requirements](#14-testing-requirements)
15. [Open Questions](#15-open-questions)

---

## 1. Purpose and Motivation

The Sigyn robot requires three Teensy 4.1 microcontrollers to perform real-time
tasks that the main PC cannot guarantee: sub-millisecond motor e-stop response,
continuous sensor polling at rates above 100 Hz, and hardware watchdog supervision
that survives an OS crash or ROS process failure.

This document specifies the **firmware** that runs on these boards. It is a
forward-looking design spec — not a description of the current implementation.
Where the current code differs from this spec, the spec is authoritative for the
rewrite.

The firmware is not designed only for Sigyn. Every architectural decision should be
evaluated for portability to the outdoor successor robot. Where a decision is
Sigyn-specific for practical reasons, that constraint must be noted.

---

## 2. Design Philosophy

### 2.1 Real-time correctness above all

The main loop on a Teensy board is a real-time control loop. Every implementation
choice must be evaluated against one question first: **will this cause the loop to
miss its deadline?** The current system regularly operates at 85–297 Hz depending
on the board. That loop rate must be maintained under all normal operating
conditions.

- **No blocking I/O.** Every sensor read, serial write, and I²C transaction must be
  non-blocking or bounded to a known worst-case time.
- **No heap allocation.** `new`, `malloc`, `std::vector`, and `std::string` are
  forbidden in production firmware. Fixed-size static buffers are required.
- **No unbounded loops.** Any iteration over a collection must have a compile-time
  upper bound.

### 2.2 Testability is a first-class requirement

Safety-critical firmware that cannot be tested without connected hardware is
firmware that will have latent bugs. Every safety path must be testable on a
development host machine without a Teensy, an oscilloscope, or a motor driver.

This requirement drives the architecture: every hardware access point must be behind
a C++ abstract interface that can be replaced with a mock in unit tests. This is not
optional. No safety-critical production code may directly call Arduino `digitalRead`,
`Wire.begin`, `Serial.print`, or equivalent hardware-specific APIs. All such calls
are delegated to injected interface implementations.

Cost: this requires slightly more code structure. Benefit: every fault condition is
testable in isolation, and the test suite can run in CI on a Linux host with no
embedded hardware.

### 2.3 Configuration should not require recompilation

The current system uses `#if BOARD_ID == 1` and compile-time `#define` flags to
configure board behavior. This works but forces a recompile to tune a sensor
threshold, adjust a ring boundary, or change a watchdog timeout. These are
operational parameters, not architectural ones.

The new design separates:
- **Structural configuration** (which modules exist on a board, which GPIO pins are
  used) — this remains compile-time, through a board configuration header.
- **Operational parameters** (thresholds, timeouts, ring boundaries) — these are
  loaded from a YAML file on the SD card at boot and persisted to EEPROM for
  runtime modifications. A recompile should never be required to change a threshold.

### 2.4 Fail-safe defaults everywhere

At power-on, before any configuration is loaded, all outputs must be in the
safest possible state:
- Motor e-stop pin: asserted (motors stopped).
- Relays: open (power removed from RoboClaw and main battery relay).
- Inter-board fault output: asserted (peer boards see a fault until this board
  announces itself healthy).

Any code that changes an output from the safe state must be in the normal
operating path, not the initialization path.

### 2.5 Observable state at all times

Every module must have a defined health state that it publishes continuously. A
module that is healthy says so. A module that is degraded says so with a
description. A module that is silent is treated as failed by the safety coordinator.

This means the safety coordinator on each board must distinguish:
- "This module reports NORMAL" (positive health assertion)
- "This module has not reported recently" (timeout — treat as fault)

Absence of a health report is a fault, not a benign condition.

---

## 3. Board Partition

### 3.1 Current partition

The existing partition separates concerns by hardware proximity:

| Board | Primary hardware | Safety authority |
|---|---|---|
| 1 — Navigation and Safety | RoboClaw, VL53L0X, Motor relay, Battery relay | Master: holds e-stop pin and relay control |
| 2 — Power and Sensors | INA226 power monitors, BNO055 IMU, Temperature | Satellite: reports to Board 1 |
| 3 — Elevator and Gripper | Stepper motors, Limit switches | Satellite: reports to Board 1 |

### 3.2 Rationale and evaluation

This partition is retained for the rewrite because:
- The physical hardware groupings limit cable routing; collocating electronics is
  practical.
- Board 1 is closest to the drive motors and relay circuits, making it the natural
  safety authority.
- Board 2 handles low-speed sensors (power monitoring at ~10 Hz, IMU at ~100 Hz)
  that do not require the tight loop frequency needed for motor control.
- Board 3 is a dedicated motion controller for the elevator/gripper. Its failure
  should be isolated from drive safety.

**Open question:** Should Board 3 communicate with the PC directly via its own USB
connection, or should it relay all messages through Board 1? See [Open
Questions](#15-open-questions) section.

### 3.3 Partition principles for the outdoor robot

When adapting this firmware for a future outdoor robot, the board partition should
be determined by:
1. Which actuators require sub-millisecond e-stop response? Those must be on the
   same board as the safety authority.
2. Which sensor groups share I²C buses or SPI buses? Collocate those on the same
   board.
3. Never put two large, slow sensors on the same board as the motor safety path.
   They will compete for loop time.

The firmware framework should support any board count from 1 to N by defining a new
board configuration header, with no changes to the shared module code.

---

## 4. Common Framework

### 4.1 Module system

Every hardware device or software service running on a Teensy board is a **Module**.
The Module base class provides:
- A standardized two-phase lifecycle: `setup()` called once at initialization,
  `loop()` called every tick.
- A priority tier that determines how often `loop()` is called relative to the
  main loop tick.
- A health state that the safety coordinator queries each tick.
- Injectable dependencies that are provided at construction time.

```cpp
namespace sigyn_teensy {

/// Priority tier for module scheduling.
/// SAFETY modules run every tick (highest rate guaranteed).
/// NORMAL modules run every tick unless the system is under load.
/// BACKGROUND modules run only when no higher-priority work is pending.
enum class ModulePriority {
  SAFETY,      ///< Must run every loop tick; timing violation is a fault.
  NORMAL,      ///< Should run every tick; occasional skip is tolerable.
  BACKGROUND,  ///< Runs when loop has spare time (logging, diagnostics).
};

/// Health state returned by modules to the safety coordinator.
enum class ModuleHealth {
  INITIALIZING,  ///< Not yet ready; no fault, but not yet safe to use.
  NOMINAL,       ///< Fully operational.
  DEGRADED,      ///< Partially functional; specific capability is reduced.
  FAILED,        ///< Not functional; cannot provide its service.
};

struct ModuleHealthReport {
  ModuleHealth health;
  char description[128];  ///< Human-readable description of degradation/failure.
  uint32_t timestamp_ms;  ///< When this report was last updated.
};

class Module {
 public:
  /// Construct a module with its priority tier and a human-readable name.
  /// All hardware interfaces required by the module must be injected here.
  explicit Module(const char* name, ModulePriority priority);
  virtual ~Module() = default;

  /// One-time initialization called by setupAll().
  /// Must not block for longer than the board's acceptable startup time.
  virtual void setup() = 0;

  /// Per-tick execution called by loopAll().
  /// Must not block; all I/O must be non-blocking or bounded.
  virtual void loop() = 0;

  /// Return the current health of this module.
  virtual ModuleHealthReport healthReport() const = 0;

  /// Called by the safety coordinator to clear recoverable faults.
  virtual void resetSafetyFlags() {}

  /// Human-readable name, used in logs and diagnostics.
  const char* name() const { return name_; }

  ModulePriority priority() const { return priority_; }

  /// Call setup() on all registered modules, in registration order.
  static void setupAll();

  /// Call loop() on all registered modules according to their priority tier.
  /// SAFETY and NORMAL modules run unconditionally.
  /// BACKGROUND modules run only if the current loop tick has spare time.
  static void loopAll(uint32_t available_time_us);

 protected:
  /// Modules self-register by calling this in their constructor.
  static void registerModule(Module* module);

 private:
  const char* name_;
  ModulePriority priority_;
};

}  // namespace sigyn_teensy
```

**Changes from current design:**

1. Modules are no longer singletons. They are constructed with injected
   dependencies in `main.cpp` and passed to `registerModule`. This enables
   testing without global state.
2. `isUnsafe()` is replaced by `healthReport()`, which returns a richer structure.
   The safety coordinator uses this to populate fault events rather than doing a
   boolean check.
3. `BACKGROUND` priority is new. The SD logger and diagnostics reporter are
   `BACKGROUND`; the safety coordinator and motor monitor are `SAFETY`.
4. `loopAll` now receives a `available_time_us` budget so background modules can
   self-limit.

### 4.2 SerialManager

The `SerialManager` is the single point of contact for USB serial communication
with the PC host. It:
- Owns a transmit queue organized by message priority (P0 through P4; see Section
  9.3).
- Reads incoming bytes into a ring buffer and assembles complete newline-terminated
  messages.
- Dispatches complete incoming messages to registered command handlers.
- Never blocks in the hot loop; all operations are bounded or deferred.

**Key change from current design:** The `SerialManager` no longer outputs raw JSON
strings assembled by each module. Instead, each module constructs a typed
`TxMessage` struct (with a message type and a data payload) and submits it to the
queue. The `SerialManager` serializes it to the wire format. This separates the
message protocol from the message production, allowing the wire format to change
without touching each module.

```cpp
enum class TxPriority {
  FAULT_EVENT     = 0,  ///< P0: immediate
  SAFETY_STATUS   = 1,  ///< P1: high
  COMMAND_ACK     = 2,  ///< P2: normal
  SENSOR_TELEMETRY = 3, ///< P3: periodic
  DIAGNOSTICS     = 4,  ///< P4: background
};

struct TxMessage {
  TxPriority priority;
  uint8_t    board_id;
  char       type[16];    ///< e.g., "FAULT", "BATT", "ODOM"
  char       payload[512];
};
```

### 4.3 PerformanceMonitor

The `PerformanceMonitor` module is a `BACKGROUND`-priority module that:
- Measures wall-clock time consumed by each `SAFETY` and `NORMAL` module per tick.
- Tracks per-module minimum, maximum, average, and 95th-percentile execution times.
- Tracks overall loop frequency and compares against the per-board target.
- Reports violations when a module exceeds its per-tick time budget.
- Publishes a `PERF` telemetry message once every 5 seconds.

**Key change:** The `PerformanceMonitor` now optionally activates a `DEGRADED`
safety fault (not an `EMERGENCY_STOP`) when the loop frequency drops below the
board's minimum threshold for a sustained period (configurable window). The
current code only logs this condition.

### 4.4 Hardware interface abstractions

The following abstract interfaces must be defined and used by all production code.
Each interface has a production implementation that calls Arduino APIs and a mock
implementation for testing.

```cpp
// GPIO
struct IGpioReader { virtual bool read(uint8_t pin) const = 0; };
struct IGpioWriter { virtual void write(uint8_t pin, bool value) = 0;
                     virtual void setMode(uint8_t pin, uint8_t mode) = 0; };

// I2C
struct II2cBus {
  virtual bool beginTransmission(uint8_t address) = 0;
  virtual bool requestFrom(uint8_t address, uint8_t count) = 0;
  virtual int  available() = 0;
  virtual int  read() = 0;
  virtual bool write(uint8_t byte) = 0;
  virtual bool endTransmission(bool stop = true) = 0;
};

// UART (used for RoboClaw and inter-board link)
struct IUart {
  virtual size_t write(const uint8_t* buf, size_t len) = 0;
  virtual int    read() = 0;
  virtual int    available() = 0;
};

// Timer
struct ITimer { virtual uint32_t millis() const = 0; };

// SD card (simplified facade)
struct ISdWriter {
  virtual bool open(const char* path) = 0;
  virtual bool write(const char* message, size_t len) = 0;
  virtual bool flush() = 0;
};
```

All production modules receive pointers or references to these interfaces in their
constructors. No module is permitted to use `digitalRead`, `Wire`, `Serial`, or
`millis()` directly in source files under `modules/`.

**Why this matters:** A `RoboClawMonitor` in a test can take a mock `IUart` that
simulates the RoboClaw protocol responses, including timeouts and error codes,
without any physical hardware. The test exercises the full fault detection and
recovery logic.

### 4.5 Configuration system

See Section 11 for the full configuration management specification. The common
framework provides a `ConfigStore` singleton that:
- Loads operational parameters from a YAML file on the SD card at boot.
- Falls back to compiled-in defaults if the SD file is absent.
- Persists runtime-modified parameters to EEPROM.
- Provides a typed accessor API (`configStore.get<float>("ring3_threshold_m")`).
- Reports which parameters are using defaults vs. loaded values as part of the
  boot diagnostic announcement.

---

## 5. Board 1 — Navigation and Safety

### 5.1 Responsibilities

Board 1 is the **embedded safety authority**. All safety actions that result in
motor behavior execute on Board 1 or via Board 1's GPIO outputs.

| Responsibility | Detail |
|---|---|
| Motor control | RoboClaw motor driver via UART (Serial7); full PID-controlled differential drive |
| Wheel odometry | Encoder pulses from RoboClaw; computes and publishes `ODOM` messages |
| Proximity sensing | 8× VL53L0X (or successor) TOF sensors; ring of protection enforcement |
| Ring 3 enforcement | Immediate e-stop assertion when Ring 3 is unexpectedly occupied |
| Ring 2 enforcement | Speed limit assertion when Ring 2 is unexpectedly occupied |
| Motor relay control | 24 V motor power rail relay (GPIO 31); for post-overcurrent power cycling |
| Main battery relay | Main battery relay (GPIO 32); for system shutdown |
| Physical e-stop input | Reads hardware e-stop button; triggers EMERGENCY_STOP fault on assertion |
| Inter-board fault input | Monitors GPIO fault signal from Board 2 |
| Inter-board UART | Full-duplex link to Board 2 (Serial5) for rich status exchange |
| Safety authority | Aggregates all faults; decides e-stop state; publishes `FAULT` messages |
| SD logging | Logs all safety events, faults, and configuration changes |

### 5.2 Loop frequency requirement

Board 1 must maintain a sustained loop frequency of ≥ 85 Hz. The current
implementation regularly achieves 297 Hz under light load. At 85 Hz, each loop
tick is ≤ 11.7 ms. The timing budget for all `SAFETY`-priority modules combined
must not exceed 8 ms, leaving headroom for communication overhead.

If the loop frequency falls below 50 Hz for more than 100 consecutive cycles, a
`DEGRADED` fault is activated. If it falls below 30 Hz for more than 50 cycles,
an `EMERGENCY_STOP` fault is activated.

### 5.3 RoboClaw monitor

The `RoboClawMonitor` module communicates with the RoboClaw via UART using the
injected `IUart` interface. It manages:
- Connection state machine (DISCONNECTED → CONNECTING → CONNECTED → ERROR_RECOVERY).
- Bidirectional non-blocking command/response protocol.
- Motor speed commands received from `SerialManager` (forwarded from PC `cmd_vel`).
- Encoder reading for odometry computation.
- RoboClaw status polling (voltage, current, temperature, error flags).

**Implementation status (2026-03-09): COMPLETE — firmware side**

The implementation lives in `wr_teensy_boards/modules/roboclaw/` and uses 13 states
(rather than the 4 originally described above):

`STARTUP` → `CONNECTING` → `VERSION_CHECK` → `CONNECTED` → `READ_ENCODERS` →
`READ_SPEEDS` → `READ_STATUS` → `READ_CURRENTS` → `SEND_COMMAND` /
`SEND_VELOCITY` → (`ESTOP_ACTIVE` or `FAULT` on error) → `RECONNECT` → ...

Key implementation details that differ from or extend the spec above:
- **UART:** `Serial7` at 230400 baud, RoboClaw address `0x80`
- **Interface:** `IRoboClaw` abstract interface in
  `modules/roboclaw/interfaces/i_roboclaw.h`; `RoboClawAdapter` wraps vendor
  `RoboClaw.h`/`.cpp` for production use; mock injection enabled for future tests
- **E-stop:** GPIO pin 30 driven directly by `RoboClawMonitor::AssertEstop()` /
  `ReleaseEstop()` — no `SafetyCoordinator` relay in firmware (see §10.1 note)
- **Kinematics:** `TwistToMotorSpeeds()`, `OdometryDelta()`,
  `DecodeRoboClawErrorBits()` extracted to `modules/roboclaw/roboclaw_kinematics.h`
  (Arduino-free) for host-system unit testing; 25 Unity tests passing
- **Version check:** exact string match `"USB Roboclaw 2x15a v4.3.6\n"` on every
  connect; version mismatch → `FAULT` state → reconnect cycle
- **Calibration constants** (do not change without hardware recalibration):
  - `wheel_diameter_m = 0.102224144529039`
  - `wheel_base_m = 0.3906`
  - `quadrature_pulses_per_revolution = 1000`
  - `cmd_vel_timeout_ms = 200`
  - Runaway threshold: ≥ 5000 QPPS with zero command → EMERGENCY_STOP

**Pending (D3):** End-to-end hardware verification (`/cmd_vel` → motors). Firmware
ready; awaiting hardware bring-up.

**Fault conditions (all sourced from Board 1):**

| Condition | Detection method | Urgency | Severity |
|---|---|---|---|
| Motor overcurrent | RoboClaw error status register | EMBEDDED | EMERGENCY_STOP |
| Motor runaway (encoder signal lost) | Encoder stuck while speed commanded | EMBEDDED | EMERGENCY_STOP |
| RoboClaw communication timeout | No response within `kRoboClawTimeoutMs` | REALTIME | EMERGENCY_STOP |
| RoboClaw API version mismatch | Version check at connection | EMBEDDED | EMERGENCY_STOP |
| 24 V motor rail voltage out of range | Power monitor data from Board 2 | REALTIME | EMERGENCY_STOP |

**Motor overcurrent recovery:** After overcurrent, automatic recovery must:
1. Assert e-stop pin.
2. Wait configurable cool-down (`kOvercurrentCooldownMs`, default 5 s).
3. Open 24 V relay.
4. Wait `kRelayOpenWaitMs` (default 2 s).
5. Close 24 V relay.
6. Allow RoboClaw to reinitialize (wait for connection state CONNECTED).
7. De-assert e-stop pin.
8. Log recovery sequence.

The decision to allow automatic recovery must be configurable (default: manual
only for overcurrent, automatic for communication timeout).

**cmd_vel forwarding:** Board 1 receives `TWIST` commands from the PC and forwards
them to the RoboClaw. The forwarded velocity must be capped to the current maximum
velocity limit, which is dynamically set by the safety coordinator (velocity limits
change when in Ring 2, or when a DEGRADED fault is active). Board 1 must never
send a velocity above the active limit regardless of what the PC commands.

**Velocity limit is a safety feature, not a convenience feature.** It must be
enforced in the embedded firmware, not relied upon in the PC software.

### 5.4 VL53L0X monitor

The `VL53L0XMonitor` (or its successor for 8×8-window sensors) manages all
proximity sensors on Board 1. It:
- Polls all N sensors in a non-blocking round-robin scan.
- Publishes the latest distance reading for each sensor as part of periodic
  telemetry.
- Evaluates ring membership for each sensor given the current robot velocity
  (from the most recently received `TWIST` command).

**Ring evaluation:**

The ring boundaries (`ring2_threshold_m`, `ring3_threshold_m`) are loaded from
configuration. Direction-aware evaluation works as follows:

1. From the most recent `TWIST` command, determine the primary direction of motion
   (forward, backward, rotating).
2. For each sensor, determine whether it is in the direction of motion (a per-sensor
   configuration property: `sensor_N_facing`; values: FRONT, REAR, LEFT, RIGHT, etc.)
3. A sensor that is NOT in the direction of motion generates a WARNING at Ring 2
   and does NOT trigger the Ring 3 e-stop unless the robot is stationary.
4. A sensor that IS in the direction of motion triggers the full ring response.

Because the `TWIST` command comes from the PC, which may be delayed, Board 1 must
also maintain its own velocity estimate from wheel encoders. If the encoder velocity
and `TWIST` velocity disagree significantly, the worst case (higher velocity) is
used for ring direction computation.

**Sensor failure handling:**
- Individual sensor timeout: activate `DEGRADED` fault for that instance.
- ≥ 50% of sensors failing simultaneously: activate `EMERGENCY_STOP` (likely cable
  or I²C bus fault).
- After a configurable recovery period, the monitor attempts to reinitialize
  failed sensors.

**Future sensor note:** This specification is intentionally sensor-model-agnostic.
The interface to the safety coordinator uses distances in meters per sensor — the
particular sensor family is hidden behind a `IProximitySensor` interface.

### 5.5 Safety coordinator on Board 1

Board 1 hosts the **master safety coordinator**: the final decision maker for all
embedded safety actions. All other boards contribute to the safety state, but Board
1 is the actor. See Section 10 for the safety coordinator specification.

### 5.6 Physical e-stop input

GPIO pin 2 is assigned as the hardware e-stop input. This pin must be configured
with `INPUT_PULLUP` in `setup()`. An interrupt must be attached on the falling edge.
The interrupt handler must atomically set a flag that the safety coordinator checks
at the top of each loop. Debounce must be applied in the flag-check logic, not in
the interrupt handler.

When asserted: activate `EMERGENCY_STOP` fault with source `"HardwareEstop"` and
lifecycle `latching` (requires human acknowledgment to clear, because the physical
mechanism must also be reset).

---

## 6. Board 2 — Power and Sensors

### 6.1 Responsibilities

Board 2 monitors the power system and all inertial/environmental sensors. It has no
actuator outputs; its role is to provide high-quality, continuously monitored data
to Board 1 and the PC.

| Responsibility | Detail |
|---|---|
| Power monitoring | 5× INA226 sensors: main battery + 24 V motor, 12 V PC, 5 V logic, 3.3 V auxiliary rails |
| Battery state estimation | Rolling current-integral and voltage model to estimate state of charge and remaining runtime |
| IMU | 2× BNO055; primary IMU used for EKF, secondary for cross-validation; tilt detection |
| Temperature monitoring | Analog TMP36 sensors at motor and other thermal points |
| Charger detection | Detects charging vs. discharging from INA226 current sign |
| Inter-board UART output | Status frames and fault signals to Board 1 |
| Inter-board GPIO output | Asserts fault pin when Board 2 enters EMERGENCY_STOP |
| SD logging | Local log of all power events; SD card required |

### 6.2 Loop frequency requirement

Board 2 must maintain a minimum of 50 Hz sustained. The IMU operating at 100 Hz
is the primary driver; the firmware loop must run fast enough not to drop IMU
samples. Battery monitoring at 10 Hz and temperature at 1 Hz do not drive the
loop requirement.

### 6.3 Battery monitor

**State of charge model:** The battery monitor must maintain a hybrid model:
- OCV (open-circuit voltage) model: maps voltage to state of charge for the
  specific battery chemistry and cell count (configurable: `battery_cell_count`,
  `battery_cell_nominal_v`, `battery_discharge_curve[]`).
- Coulomb counting: integrates current draw over time to track depletion.
- The two models are blended with a configurable weight; the OCV model dominates
  when current is near zero (at rest), the coulomb counter dominates during
  dynamic use.

**Return-to-charger threshold:** Board 2 computes and exports:
- `remaining_charge_pct`: state of charge estimate.
- `remaining_runtime_s`: estimated time until the battery must begin charging given
  current draw rate.
- `distance_to_charger_m` and `time_to_charger_s`: provided by the PC (see
  `wr_ros_teensy.spec.md` Section 6.3) and stored by Board 2 for threshold
  evaluation.
- When `remaining_runtime_s < time_to_charger_s + safety_margin_s`, Board 2
  activates an `ADVISORY` fault that the PC's safety coordinator escalates to a
  "return to charger" behavior request.

**Charging detection:** When `INA226` main battery current is negative (current
flowing into battery) and voltage is rising, report `state = CHARGING`. The behavior
tree uses this to confirm docking success. Board 2 must report charging state in
every periodic `BATT` message.

### 6.4 IMU monitor

Both BNO055 sensors are read at their configured output data rate (target: 100 Hz).
The primary IMU (configurable by parameter) is used for tilt detection and for EKF
fusion. The secondary IMU is used for cross-validation — if the two disagree by
more than a configurable angle threshold, a `DEGRADED` fault is raised for the
secondary.

**Tilt detection hierarchy** (see safety_system.spec.md Section 8.5):
- 15°: WARNING (advisory)
- 25°: EMERGENCY_STOP (auto-clear)
- 60°: EMERGENCY_STOP + latching (robot may have fallen; requires human ack)

**Predictive tilt:** If the current tilt speed (derivative of tilt angle, smoothed
over 50 ms) multiplied by the configurable lookahead time (`tilt_lookahead_s`,
default 0.5 s) would predict exceeding the 25° threshold, activate a `CRITICAL`
fault before the threshold is reached.

**IMU calibration:** BNO055 calibration status (0–3 per sensor type) must be
included in every IMU telemetry message. A calibration status below 1 for the
gyroscope or accelerometer raises an `ADVISORY`. An overall calibration score below
1 raises a `DEGRADED` fault because tilt angles become unreliable.

### 6.5 Temperature monitor

Temperature sensors are analog TMP36 devices read through the ADC. Each reading
requires moving-average filtering (window: 8 samples) because ADC noise on a
Teensy near active motors is significant.

Sensors must be identified by position in the configuration file
(`temp_sensor_0_location = "left_motor"`, etc.) so that fault messages contain
a meaningful human-readable location rather than an index.

---

## 7. Board 3 — Elevator and Gripper

### 7.1 Responsibilities

Board 3 controls the elevator and gripper mechanism. Unlike Boards 1 and 2, Board
3 is an actuator-only board with no continuous safety monitoring responsibility.

| Responsibility | Detail |
|---|---|
| Elevator stepper motor | Position control with configurable speed, acceleration, and limits |
| Extender stepper motor | Position control for gripper extension |
| Limit switches | Home position detection; over-travel protection |
| Safety coordinator | Must be enabled; Board 3 enters EMERGENCY_STOP locally when it detects hardware faults |
| Inter-board GPIO output | Asserts fault pin to Board 1 on EMERGENCY_STOP |

### 7.2 Safety requirements

Board 3 must have the `SafetyCoordinator` enabled. This is currently disabled
(`BOARD_HAS_SAFETY = 0`) and is a critical gap. The following faults must be
implemented:

| Condition | Urgency | Severity |
|---|---|---|
| Over-travel detected (limit switch hit at non-home position) | EMBEDDED | EMERGENCY_STOP |
| Stepper stall detected (position error exceeds threshold) | REALTIME | CRITICAL |
| Board 3 heartbeat timeout (PC side monitors) | N/A (PC detects) | DEGRADED |
| Board 3 loop frequency below threshold | REALTIME | DEGRADED |

### 7.3 Command interface

Board 3 accepts position commands and status queries from the PC via USB serial.
Commands are accepted in the standard `TYPE:{ payload }` format. Board 3 responds
to:
- `STEPPOS3: { target, speed, accel }` — move to absolute position.
- `STEPHOME3: {}` — home the specified axis.
- `STEPSTATUS3: {}` — query current position and state.

While a EMERGENCY_STOP is active (from any source), Board 3 must:
1. Halt all stepper motion immediately.
2. Hold position (do not free-wheel).
3. Reject all new position commands until the e-stop is cleared.

### 7.4 Communication pathway

An open question exists about whether Board 3 should have its own direct USB
connection to the PC or relay all communication through Board 1. The current
design uses a direct PC connection per board. Section 15 discusses when this
might change.

---

## 8. Inter-Board Communication

### 8.1 Two channels: GPIO and UART

Inter-board communication uses two complementary channels:

| Channel | Latency | Purpose |
|---|---|---|
| GPIO (dedicated fault line per board) | < 1 ms | Emergency stop propagation; heartbeat |
| UART (Serial5, 1 Mbit/s, full-duplex) | < 10 ms | Rich status frames, fault details, config sync |

**Never use only one channel.** A board that enters EMERGENCY_STOP must assert its
GPIO fault line AND send a UART fault frame. The GPIO path is the safety guarantee;
the UART path provides context for logging and recovery.

### 8.2 GPIO protocol

Each board has one dedicated GPIO output that acts as a **fault assertion line**:
- **Active HIGH = fault asserted.**
- The pin is driven HIGH when the board is in EMERGENCY_STOP or SYSTEM_SHUTDOWN.
- The pin is driven LOW when the board is in NORMAL, WARNING, or DEGRADED state.
- At power-on, before any software initialization, the pin is HIGH (defaulting to
  fault asserted; the board declares itself healthy only after initialization).
- Board 1 takes the logical OR of all fault lines: if any input is HIGH, Board 1
  enters an inter-board emergency stop.

Board 1 must not clear an inter-board fault until:
1. The asserting board's GPIO line is LOW.
2. At least one UART frame from the asserting board has confirmed the fault is clear.

**Pull-up behavior:** Board 1 configures its input pins with `INPUT_PULLUP`. If a
wire is disconnected or a board is physically absent, the pin reads HIGH and Board
1 sees a fault. This is the correct fail-safe behavior: absence of a board is a
fault condition, not a healthy condition.

**Suppression at boot:** Board 1 suppresses fault interpretation for a configurable
time after power-on (`kInterBoardFaultSuppressMs`, default 3000 ms) to allow all
boards to boot and begin driving their GPIO lines before faults are evaluated.
After suppression expires, the first UART frame from each peer board ends the
suppression for that board.

### 8.3 UART inter-board protocol

The UART protocol between boards uses a compact binary-framed format (not JSON).
The low bandwidth requirement and the need for deterministic parsing time make
binary framing appropriate here. This is an internal protocol, not user-visible.

**Frame format:**

```
[START: 0xAA] [TYPE: 1 byte] [LENGTH: 1 byte] [PAYLOAD: 0–N bytes] [CRC8: 1 byte]
```

Maximum frame size: 64 bytes. This bounds parsing time per frame to a
known-small constant.

**Frame types:**

| Type byte | Name | Direction | Frequency | Description |
|---|---|---|---|---|
| 0x01 | HEARTBEAT | Board N → Board 1 | 10 Hz | Board ID, health state, uptime, loop frequency |
| 0x02 | FAULT_ACTIVATE | Board N → Board 1 | On event | Board ID, fault instance, severity, source, description |
| 0x03 | FAULT_DEACTIVATE | Board N → Board 1 | On event | Board ID, fault instance |
| 0x04 | STATUS_UPDATE | Board 2 → Board 1 | 5 Hz | Power summary (voltage, current, SoC, charging state) |
| 0x05 | ESTOP_COMMAND | Board 1 → Board N | On event | Emergency stop command from Board 1 to satellite boards |
| 0x06 | CONFIG_SYNC | Board 1 ↔ Board N | On change | Configuration parameter update |
| 0x07 | PROTOCOL_ANNOUNCE | Both directions | On connect | Protocol version, supported frame types |

**Protocol version negotiation:** On startup, each board broadcasts `PROTOCOL_ANNOUNCE`
and waits for the peer's announcement before entering the operational state. A
version mismatch activates a `DEGRADED` fault and falls back to GPIO-only
communication.

### 8.4 Heartbeat and timeout policy

Board 1 maintains a heartbeat counter for each satellite board. If no `HEARTBEAT`
frame is received within `kInterBoardHeartbeatTimeoutMs` (default: 500 ms),
Board 1 activates a fault:

| Board lost | Fault severity | Auto-clear | Response |
|---|---|---|---|
| Board 2 | DEGRADED | Yes (on reconnect) | Alert human; disable power monitoring faults (data unavailable) |
| Board 3 | WARNING | Yes (on reconnect) | Halt gripper/elevator if in motion; alert human |

Board 2 similarly monitors for Board 1 heartbeats. If Board 1 is silent for
`kInterBoardHeartbeatTimeoutMs`, Board 2 asserts its GPIO fault line to force a
hardware e-stop via Board 1's input (if Board 1 is still powered) and alerts the PC
via serial.

---

## 9. PC Serial Bridge Protocol

### 9.1 Wire format

All messages between Teensy boards and the PC host are newline-delimited JSON:

```
TYPE<board_id>:{"v":<version>, ...payload...}\n
```

The `"v"` field is the per-message-type protocol version (see Section 9.4).
Short key names are used to conserve bandwidth.

**Example:**
```json
BATT2:{"v":1,"idx":0,"V":42.07,"A":0.23,"soc":0.87,"state":"DISCHARGING","loc":"36VLIPO"}
```

### 9.2 Baud rate

The current baud rate is 921600 (as observed in `sigyn_to_sensor_v2`). This is
retained as the default. The baud rate must be configurable via parameter; a build
flag is not acceptable.

### 9.3 Message priority queue

Every message transmitted by a Teensy board is assigned one of five priority levels.
The transmit queue enforces these priorities:

| Priority | Category | Examples | Maximum P0-delay allowed |
|---|---|---|---|
| P0 — immediate | Fault activation/deactivation | `FAULT` | — |
| P1 — high | Safety status, heartbeat | `ESTOP`, `HEARTBEAT` | 0 (cannot be delayed) |
| P2 — normal | Command responses | `ODOM` | 1 slot |
| P3 — periodic | Sensor telemetry | `BATT`, `IMU`, `TEMP` | — up to 2 slots |
| P4 — background | Diagnostics | `PERF`, `DIAG` | — up to 4 slots |

A P3 or P4 message in the queue may be skipped one or more times when higher-priority
messages are waiting. A P3 message skipped more than 4 times consecutively is elevated
to P2 (to prevent starvation).

### 9.4 Protocol versioning

Each message type has an independent version field (`"v"` key). Increasing the
version is required whenever any field is added, removed, renamed, or changes
type.

On connect, each Teensy board broadcasts a `PROTO_ANNOUNCE` message:

```json
PROTO1:{"v":1,"board":1,"types":{"FAULT":2,"ODOM":1,"BATT":0,"IMU":1,...}}
```

The PC bridge checks that all expected message types are present and that their
versions are within the range it supports. A version outside the supported range
activates a `DEGRADED` fault for that message type; the PC bridge logs the
mismatch and disables processing for that type rather than silently misinterpreting
fields.

### 9.5 Commands from PC to Teensy

The PC sends commands to Teensy boards as newline-delimited JSON:

| Command type | Target | Payload | Description |
|---|---|---|---|
| `TWIST` | Board 1 | `{"lx": float, "az": float}` | Velocity command (linear x, angular z) |
| `ESTOP` | Any | `{"cmd": "set"|"clear", "reset": bool}` | Emergency stop control |
| `STEPPOS` | Board 3 | `{"axis": int, "pos": int, "spd": int, "acc": int}` | Stepper position command |
| `STEPHOME` | Board 3 | `{"axis": int}` | Home a stepper axis |
| `STEPSTATUS` | Board 3 | `{}` | Query stepper state |
| `CONFIG_SET` | Any | `{"key": string, "value": varies}` | Set a runtime parameter |
| `CONFIG_GET` | Any | `{"key": string}` | Read a runtime parameter |
| `CONFIG_RESET` | Any | `{}` | Reset all parameters to YAML-loaded defaults |
| `SDIR` | Any | `{"path": string}` | List SD card directory |
| `SDLINE` | Any | `{"path": string, "line": int}` | Read a line from a SD file |
| `HEARTBEAT_PC` | Board 1 | `{"ts": int64}` | PC-side heartbeat; carries ROS time |

**The `HEARTBEAT_PC` command** serves two purposes: it confirms that the PC is
alive (Board 1 monitors heartbeat rate), and it carries the current ROS2
nanosecond timestamp so that the Teensy SD log can record both the local
millisecond clock and the best available wall-clock time. This timestamp should
be sent at ≥ 1 Hz.

---

## 10. Safety Integration

The safety coordinator is instantiated on all three boards:
- **Board 1:** Master safety coordinator; authoritative for motor e-stop and relay
  control.
- **Board 2:** Satellite coordinator; reports faults to Board 1 via UART and GPIO.
- **Board 3:** Satellite coordinator; reports faults to Board 1 via GPIO.

The safety coordinator design is fully specified in `docs/specs/safety_system.spec.md`.
The firmware-specific requirements are:

### 10.1 SafetyCoordinator class

```cpp
class SafetyCoordinator : public Module {
 public:
  /// Constructor. All hardware interfaces are injected.
  SafetyCoordinator(IGpioWriter& fault_gpio_out,
                    IGpioReader& fault_gpio_in,    // Board 1 only
                    ITimer&      timer,
                    ISerialManager& serial);

  /// Activate a fault. Anyone may call this.
  void activateFault(const char* source, uint8_t instance,
                     FaultSeverity severity, const char* description,
                     bool auto_clear = true, bool requires_power_cycle = false);

  /// Deactivate a fault (called by the originating module when condition clears).
  void deactivateFault(const char* source, uint8_t instance);

  /// Request a human override (PC sends this via command).
  void requestHumanOverride(const char* source, uint8_t instance,
                             const char* reason);

  /// Clear all non-latching faults (ESTOP clear command from PC or human).
  void clearNonLatchingFaults();

  /// Query: is any fault of the given severity or higher active?
  bool isActive(FaultSeverity min_severity) const;

  /// Set the active velocity limit (enforced in RoboClawMonitor).
  /// Called by the safety coordinator itself when ring occupancy changes.
  void setVelocityLimit(float max_linear_m_per_s, float max_angular_rad_per_s);

  float maxLinearVelocity()  const;
  float maxAngularVelocity() const;

  void setup() override;
  void loop() override;
  ModuleHealthReport healthReport() const override;
};
```

### 10.2 Fault registry

Each fault is identified by `(source, instance)`. The fault registry is a fixed-
size array sized at compile time (`kMaxFaults = 32`). If the registry is full and
a new fault is received, the lowest-severity existing fault is evicted and a 
`DEGRADED` fault ("FaultRegistryFull") replaces it.

### 10.3 E-stop output protocol

When the safety coordinator on Board 1 determines that the aggregated fault severity
is `EMERGENCY_STOP` or higher:
1. De-assert the RoboClaw e-stop pin (active LOW: this stops the RoboClaw).
2. Publish a `FAULT` message with full fault details.
3. Send inter-board `ESTOP_COMMAND` UART frames to satellites.
4. Set the velocity limit to zero.

When the severity drops below `EMERGENCY_STOP` (all relevant faults cleared):
1. Assert the RoboClaw e-stop pin HIGH (re-enables the RoboClaw).
2. Publish a `FAULT` deactivation message.
3. Restore velocity limits to configured values.

---

## 11. Configuration Management

### 11.1 Three-layer configuration

```
Layer 3: Runtime overrides   (in-memory, from CONFIG_SET commands; volatile)
Layer 2: EEPROM parameters   (persisted; survive power cycle; set by user)
Layer 1: SD YAML file        (default values; read at boot; managed file)
Layer 0: Compiled defaults   (fallback; hardcoded in firmware; never runtime)
```

On boot:
1. Load compiled defaults into `ConfigStore`.
2. If SD card is present, load YAML file values (override layer 0).
3. If EEPROM is not blank, load EEPROM values (override layers 0–1).
4. Layer 3 (runtime) begins empty; values can be set via `CONFIG_SET` commands.

A `CONFIG_RESET` command erases EEPROM and reloads from SD YAML (dropping layer 2).

### 11.2 Parameter categories

| Category | Examples | Persistence |
|---|---|---|
| Safety thresholds | `ring2_m`, `ring3_m`, `tilt_warning_deg`, `battery_critical_v` | EEPROM |
| Velocity limits | `max_linear_mps`, `max_angular_radps`, `ring2_speed_limit_mps` | EEPROM |
| Timing | `kRoboClawTimeoutMs`, `kImgHeartbeatTimeoutMs`, `kLoopWatchdogMs` | EEPROM |
| Sensor calibration | `temp_sensor_N_offset`, `imu_mount_rpy` | EEPROM |
| Board identity | `board_id`, `board_name` | Compiled |
| Battery model | `battery_discharge_curve[]`, `battery_capacity_mah` | SD YAML only |
| GPIO pin assignments | `pin_roboclaw_estop`, `pin_relay_motor` | Compiled |

### 11.3 Exposure to PC

All parameters in layers 1–2 are readable via `CONFIG_GET`. All parameters that
are not compile-time structural are writable via `CONFIG_SET`. The PC bridge
(`sigyn_to_teensy`) exposes these as ROS 2 parameters, allowing `ros2 param set`
and `rqt` to tune the embedded system at runtime.

---

## 12. Hardware Watchdog

### 12.1 Enable on all production builds

The Teensy 4.1 hardware watchdog (`WDT` peripheral) must be enabled in every
production build. The watchdog timeout is `kWatchdogTimeoutMs` (default:
200 ms, configurable via SD YAML but not via EEPROM to prevent accidental
watchdog disable by a misconfigured runtime write).

### 12.2 "Pet" policy

Each module in the `SAFETY` priority tier must "pet" the watchdog at the end of its
`loop()` call. If any `SAFETY` module hangs (infinite loop, blocked I/O), the
watchdog expires before the next module can pet it, and the board resets.

This requires that the injected `ITimer` interface also include a watchdog pet
method (`ITimer::petWatchdog()`). The mock implementation is a no-op; the
production implementation calls the Teensy WDT API.

### 12.3 Reset reason persistence and reporting

On boot, the firmware checks the reset reason register. If the reset was due to
a watchdog timeout:
1. Store the reset reason in a designated EEPROM location (a ring buffer of the
   last 8 resets with timestamps).
2. Enter e-stop state before reinitializing any module.
3. Include reset reason in the `PROTO_ANNOUNCE` message on reconnect.
4. Activate a `DEGRADED` fault ("WatchdogReset") that requires human acknowledgment
   to clear.

A watchdog reset indicates a bug or an extreme load condition. It must not be
silently absorbed.

---

## 13. Build System

### 13.1 PlatformIO

The firmware is built with PlatformIO. Each board has a dedicated build environment
in `platformio.ini`. All three environments share a common base environment that
defines compiler flags, library dependencies, upload settings, and the test
environment.

### 13.2 Board configuration headers

Each board environment must include exactly one board configuration header that
defines:
- Which modules are instantiated.
- Which GPIO pin numbers are used.
- Board identity constants (ID, name, USB serial description).

This header is `boards/board1_config.h` (etc.). The common framework, module code,
and test code must not include any board configuration header directly; they
receive configuration through injected interfaces and constructor parameters.

### 13.3 Single-firmware multi-board build

The current approach of a single source tree compiled N times for N boards is
correct and must be maintained. The `BOARD_ID` macro is the sole build-time
differentiator. All run-time behavior differences between boards must derive from:
- Different modules being instantiated in `main.cpp`.
- Different configuration parameters loaded from the SD YAML file.

Scattering `#if BOARD_ID == 1` in module source files is forbidden; all such
conditionals belong in `boards/board1_config.h` and `main.cpp`.

### 13.4 Test environment

The `[env:test]` build environment compiles all firmware code for the Linux host
(not for Teensy) using GCC and the Google Test framework. No Arduino SDK is
present; all hardware interfaces are provided by mock implementations.

Test coverage requirements:
- Every fault activation/deactivation condition in `safety_system.spec.md`
  Section 8 must have a test case.
- The `SerialManager` transmit priority queue must have ordering tests.
- The `ConfigStore` load/save/reset cycle must have tests.
- The inter-board UART protocol encoder/decoder must have round-trip tests.
- The ring-of-protection evaluation logic (direction-aware) must have tests with
  synthetic sensor readings and velocity vectors.

---

## 14. Testing Requirements

### 14.1 Principles

See `docs/specs/safety_system.spec.md` Section 13. All principles there apply.

This section adds firmware-specific requirements:

### 14.2 Hardware interface mocks

The following mock implementations must be provided in `test/mocks/`:

| Interface | Mock class | Notes |
|---|---|---|
| `IGpioReader` | `MockGpioReader` | Set per-pin return value; track call count |
| `IGpioWriter` | `MockGpioWriter` | Record all writes; verify e-stop pin state |
| `II2cBus` | `MockI2cBus` | Program response sequences per I²C address |
| `IUart` (RoboClaw) | `MockRoboClawUart` | Simulate full RoboClaw protocol including error responses |
| `IUart` (inter-board) | `MockInterBoardUart` | Inject UART frames; verify transmitted frames |
| `ITimer` | `MockTimer` | Advance time manually; verify timeout behaviors |
| `ISdWriter` | `MockSdWriter` | Capture log output for verification |
| `ISerialManager` | `MockSerialManager` | Capture all queued messages; inject commands |

### 14.3 Test coverage checklist

| Requirement | # Tests minimum |
|---|---|
| Each fault condition in Section 8 of safety_system.spec.md: normal→fault | 1 per condition |
| Each fault condition: fault→recovered (auto-clear) | 1 per auto-clear condition |
| Each fault condition: fault→latching (confirm fault persists after source clear) | 1 per latching condition |
| Hardware watchdog: reset-reason logging and DEGRADED fault activation | 1 |
| Inter-board UART: HEARTBEAT timeout triggers correct Board 1 response | 1 |
| Inter-board UART: FAULT_ACTIVATE from Board 2 propagates to fault registry | 1 |
| Inter-board GPIO: HIGH from Board 2 triggers Board 1 EMERGENCY_STOP | 1 |
| Ring 3: direction-aware non-trigger (obstacle behind, moving forward) | 1 |
| Ring 3: direction-aware trigger (obstacle in front, moving forward) | 1 |
| Ring 2: speed limit applied; cmd_vel above limit is clamped | 1 |
| ConfigStore: YAML load, EEPROM override, runtime override, reset | 4 |
| Priority queue: P0 message not delayed by P3 backlog | 1 |

---

## 15. Open Questions

| ID | Question | Affects |
|---|---|---|
| OQ-TB-1 | Should Board 3 have its own direct USB connection to the PC, or relay all messages through Board 1? Direct connection gives Board 3 independence if Board 1 fails; relay simplifies `sigyn_to_teensy` connection management. | Sections 7, 9 |
| OQ-TB-2 | For the successor 8×8 sensor (replacement for VL53L0X), what is the I²C address scheme? The current 8-sensor design uses XSHUT GPIO to address-assign at startup. The 8×8 equivalent must be evaluated. | Section 5.4 |
| OQ-TB-3 | What is the maximum safe closed-loop velocity command to the RoboClaw when the encoder is not providing feedback (e.g., encoder wire fault)? Must we always stop, or can we send an open-loop velocity? | Section 5.3 |
| OQ-TB-4 | Should the RoboClaw e-stop be wired active-LOW or active-HIGH? The fail-safe requirement argues strongly for active-LOW (broken wire = motor stop), but the current configuration has the pin definition commented out. This must be resolved in hardware before any software wiring implementation. | Section 5.3, config.h |
| OQ-TB-5 | Is a dedicated hardware clock chip (e.g., DS3231 RTC) needed on Board 1 to give the SD log entries a meaningful wall-clock timestamp when the PC is disconnected? Currently only milliseconds since boot are available. | Section 11 |
| OQ-TB-6 | What is the EEPROM layout version scheme? When firmware is updated and adds new config keys, how does the new firmware detect and migrate an EEPROM written by an older version? | Section 11 |
| OQ-TB-7 | For the outdoor robot, is a CAN bus more appropriate for inter-board communication than UART point-to-point? This would support more boards and better fault isolation, but requires hardware changes. | Section 8 |
| OQ-TB-8 | Should the `PerformanceMonitor` be able to dynamically adjust module scheduling (skip NORMAL modules during high-load ticks to protect SAFETY modules) or is static priority assignment sufficient? | Section 4.3 |
