# Sigyn Master TODO
*Consolidated from all repos under `~/sigyn_ws/src/` — last updated 2026-02-28*

Sources aggregated:
- `Sigyn/TODO.md`
- `sigyn_teensy_boards/TODO.md`
- `can_do_challenge/TODO.md`
- Inline `TODO`/`FIXME` comments in source files

---

## 🔴 CRITICAL — Must Fix Before Production

### Safety System

- **[CRITICAL] Enable Safety on Board 3 (Gripper/Elevator).**
  `BOARD_HAS_SAFETY` is currently `0` on Board 3. Set it to `1` and instantiate
  `SafetyCoordinator` in `elevator_board.cpp`. Without this, the gripper has no local e-stop
  and a firmware hang will not be detected.
  _Sources: `sigyn_teensy_boards/TODO.md`, `Sigyn/TODO.md`_

- **[CRITICAL] Enable `SafetyCoordinator` on Board 2.**
  `board2_main.cpp` needs `SafetyCoordinator::getInstance()` initialized and
  `BOARD_HAS_SAFETY=1` in its `config.h`. Each board must be able to send `FAULT`
  messages to `sigyn_to_teensy`.
  _Sources: `sigyn_teensy_boards/TODO.md`, `Sigyn/TODO.md`_

- **[CRITICAL] Fix `fault_handler` inter-board serial signaling.**
  `board1_main.cpp` has an unimplemented TODO to notify other boards via serial when
  Board 1 enters emergency stop. Implement this as the software complement to the GPIO
  e-stop.
  _Sources: `sigyn_teensy_boards/TODO.md`, `Sigyn/TODO.md`_

- **[CRITICAL] E-stop pull-down (fail-safe wiring).**
  The motor e-stop line must be pulled LOW so that if the Teensy loses power or a wire
  breaks, the robot stops. Verify in hardware schematic and test.
  _Source: `sigyn_teensy_boards/TODO.md`_

- **[CRITICAL] Sensor timeout safety.**
  Sensors that stop updating (I²C hang, disconnection) must trigger `isUnsafe()` within
  one loop period. Currently a stale sensor returns the last value forever.
  _Sources: `sigyn_teensy_boards/TODO.md`, `Sigyn/TODO.md`_

---

## 🔴 HIGH — Safety System (Next 6 Months)

### Cross-Board E-Stop via GPIO

- Implement hardware-level fault propagation using GPIO pins 10–12 (already assigned).
  1. Board 1 asserts GPIO high on `EMERGENCY_STOP`; Boards 2 & 3 interrupt on that pin.
  2. Board 2 asserts a separate GPIO pin; Board 1 monitors via interrupt.
  3. Board 3 asserts a third GPIO pin; Board 1 monitors via interrupt.
  4. Each board: `attachInterrupt()` → immediately invoke
     `SafetyCoordinator::raiseEmergencyStop(SOURCE_EXTERNAL)`.
  5. Board 1 drops e-stop only when ALL GPIO pins clear AND no local faults.
  6. Use active-HIGH assertion with pull-down (wire-break does NOT trigger false interrupt).
  - Files: `board1_main.cpp`, `board2_main.cpp`, `elevator_board.cpp`,
    `safety_coordinator.h` (add `SOURCE_EXTERNAL`), `docs/Safety_System.md`
  _Sources: `sigyn_teensy_boards/TODO.md`, `Sigyn/TODO.md`_

### IMU Safety Integration — Board 2 (BNO055 ready)

- Add `IMUSafetyMonitor` module to Board 2.
- WARNING at 20° pitch/roll; `EMERGENCY_STOP` at 30°.
- Rapid spin > 180°/s triggers fault.
- Integrate with Nav2: cancel active goals on WARNING.
- Self-healing: auto-clear when tilt returns to normal.
- Add config parameters in `imu_safety_monitor.h`.
- Tests: mock IMU data + physical tilt stand.
  _Sources: `sigyn_teensy_boards/TODO.md`, `Sigyn/TODO.md`_

### VL53L0X Obstacle Safety — Collision Prediction (8 sensors ready)

- Add collision prediction to `VL53L0XMonitor` (Board 1).
- Direction-aware: only trigger when obstacle is in direction of motion.
- WARNING at 500 mm (slow down); `EMERGENCY_STOP` at 200 mm (immediate stop).
- Hysteresis: 600 mm clearance before recovery (prevent oscillation).
- Add `allow_close_approach` behavior-tree flag to suppress during docking.
- Config parameters in `vl53l0x_monitor.h`.
- Tests: mock sensor data + real hardware wall-approach run.
  _Sources: `sigyn_teensy_boards/TODO.md`, `Sigyn/TODO.md`_

### SYSTEM_SHUTDOWN — Graceful Power-Down on Low Battery (relay ready)

- Implement `SYSTEM_SHUTDOWN` severity level (beyond `EMERGENCY_STOP`).
- Trigger: battery < 30 V sustained for > 30 seconds (both INA226 must agree).
- Shutdown sequence: ROS 2 notification → motor e-stop → SD card flush →
  main battery relay cut (GPIO pin 32).
- Not self-recovering — requires manual restart or charger.
- Config in `battery_monitor.h`; tests with mock battery data only.
  _Sources: `sigyn_teensy_boards/TODO.md`, `Sigyn/TODO.md`_

---

## 🟠 HIGH — Firmware Architecture

### A. Full Architectural Review

Review the overall multi-board firmware design and answer:
- Is the three-board split (Navigation+Safety / Power+Sensors / Elevator+Gripper) still correct?
- Is `serial_manager` + JSON the best long-term ROS 2 bridge, or should we move to micro-ROS?
- Does the module registration/lifecycle pattern scale well?
- Is `SafetyCoordinator` the right single point of truth, or should each board have equal authority?
- Are compile-time flags (`BOARD_HAS_SAFETY`, `ENABLE_PERFORMANCE_MONITOR`, etc.) the right
  approach, or should some be runtime-configurable?
- Update `docs/ARCHITECTURE.md` and `docs/Safety_System.md` as deliverables.
  _Source: `sigyn_teensy_boards/TODO.md`_

### B. Redesign Serial Message Protocol

The current JSON protocol (`~100–300 bytes/frame` at 85 Hz) is near bandwidth limits.
Evaluate:
1. Compact JSON (abbreviated key names).
2. Hybrid: short fixed-prefix + minimal JSON for common messages.
3. Binary framing (length-prefixed structs + type byte).
4. CBOR / MessagePack.

Constraints:
- Must remain debuggable via a serial terminal without a decoder.
- `message_parser.cpp` on the ROS side must be updated simultaneously.
- Document final format in `docs/Message_Formats.md`.
  _Source: `sigyn_teensy_boards/TODO.md`_

### C. `serial_manager.cpp` inline TODOs (`sigyn_teensy_boards`)

- L125: Implement configuration-update handling.
- L131: Implement comprehensive status-report sending.
- L144: Route incoming sensor-query messages to the appropriate sensor module.
  _Source: inline TODOs in `common/core/serial_manager.cpp`_

### D. `module.cpp` inline TODO

- L76: Decide whether a module registration failure should trigger an immediate e-stop
  or a fault indicator (currently just logged).
  _Source: inline TODO in `common/core/module.cpp`_

---

## 🟠 HIGH — Can-Do Challenge (`can_do_challenge` repo)

### Gripper / Behavior Tree Correctness

- Review gripper Teensy code to ensure stepping actions never return `RUNNING`
  while still stepping (must be atomic).
- Review places where plain `Sequence` was used instead of a guarded `ReactiveSequence`
  to preserve safety subtree responsiveness.
- Update `ExtendTowardsCan` to continuously monitor alignment during extension:
  - Verify gripper Z-height remains correct relative to the can.
  - Verify robot rotation keeps the can's vertical centerline aligned with
    `parallel_gripper_base_plate` centerline.
- Verify grasp success after retraction (`RetractExtender`):
  - Add logic to confirm can is still held.
  - Add recovery logic (retry grasp, re-detect) if grasp fails.

### Nav2 Integration (unimplemented stubs)

- `bt_nodes.cpp` L998 / `bt_nodes_real.cpp` L1235: Implement call to Nav2
  `ComputePathToPose` action.
- `bt_nodes.cpp` L1007 / `bt_nodes_real.cpp` L1244: Implement call to Nav2
  `FollowPath` action.
- `bt_nodes.cpp` L2411 / `bt_nodes_real.cpp` L3076: Implement navigation to
  charging dock and initiation of charging.
- `bt_nodes.cpp` L2430 / `bt_nodes_real.cpp` L3095: Implement publish to
  E-stop topic.

### Hardware Topic Subscriptions (unimplemented stubs)

- `bt_nodes.cpp` L411 / `bt_nodes_real.cpp` L492: Subscribe to real hardware
  topics (currently empty stubs).
- `bt_nodes.cpp` L797 / `bt_nodes_real.cpp` L906: Check gripper force sensor or
  verify can is still visible in Pi Camera after retraction.

### Sim ↔ Real Parity

- Align `WaitForNewOAKDFrame` (sim: fixed delay; real: heartbeat timestamp).
- Align `MoveTowardsCan` node type (`StatefulActionNode` in sim vs
  `SyncActionNode` in real).
- Add `OAKDDetectCan` node to sim (or add a stub mirroring real behavior).
- Unify `initializeObjectDetection` subscriptions and message types:
  - OAK-D topics (`/oakd_top` vs `/oakd`) and heartbeat mechanism.
  - Pi camera topics (`/gripper/can_detection` vs `/gripper/camera/detections`)
    and message types.
- Make `getFreshPiDetection` consistent (sim: blocking wait; real: non-blocking
  with different throttling).
- Make `piDetectionInRange` consistent (different Z defaults in sim/real).
- Align `ObjectDetectionState` defaults/fields (e.g., `WITHIN_REACH_DISTANCE`,
  2D centers, class/score tracking).
- Perform a full parity review of all nodes in `bt_nodes.cpp` vs
  `bt_nodes_real.cpp` and resolve remaining mismatches.

---

## 🟡 MEDIUM — RoboClaw (`sigyn_teensy_boards`)

- Cross-check localization data against encoder movement to detect wheels
  spinning without actual displacement (stuck/slipping).
  (`roboclaw_monitor.cpp` L842: Escalate encoder read failures via `SafetyCoordinator`.)
- Add power-cycle capability via `PIN_RELAY_ROBOCLAW_POWER` to clear latching hardware
  faults. (`roboclaw_monitor.cpp` L1112, L1130, L1153)
- Confirm which status bits in the RoboClaw manual require a power cycle.
  (`roboclaw_monitor.cpp` L47)
- Raise `WARNING` fault when RoboClaw temperature exceeds `roboclaw_temp_warning_c`.
  (`roboclaw_monitor.cpp` L788)
- Board 1 should publish RoboClaw temperature, motor 1 current, and motor 2 current
  to `sigyn_to_sensor_v2`.

---

## 🟡 MEDIUM — Behavior Trees (General)

- Create a single parameterized `IsFaultActive` condition node that takes a
  `target_fault` input port, replacing the per-fault individual condition nodes.
  _Sources: `Sigyn/TODO.md`, `sigyn_teensy_boards/TODO.md`_

---

## 🟡 MEDIUM — Safety System (6–12 Months)

### Watchdog Timer

- Enable Teensy 4.1 hardware WDT (500 ms timeout).
- Feed it after safety checks in the main loop.
- On WDT reset: `SafetyCoordinator` reinit → e-stop asserted by default until
  explicitly cleared.
- Tests: inject infinite loop, verify reset + e-stop assertion.

### Reverse E-Stop Signaling to All Boards

- Add e-stop relay output to Board 2 (GPIO pin 30, currently unused).
- Board 2 can independently enforce e-stop (cut power to Board 1).
- Symmetry: all boards have equal e-stop authority.
- Tests: Board 1 failure simulation.

### Safety Event Logging and Replay

- Circular buffer: last 100 safety events (timestamp, source, severity).
- Store to SD card on e-stop trigger (crash log).
- ROS 2 service `teensy_sensor_safety_history` to retrieve events.
- Replay capability: load recorded events into mock system.

### Adaptive Threshold Tuning

- Runtime config service: `teensy_sensor_update_config`.
- Behavior trees can temporarily adjust thresholds (e.g., cold environment).
- Persist tuned thresholds to EEPROM across reboots.
- Safety limits: prevent setting thresholds outside safe ranges.

---

## 🟡 MEDIUM — Perimeter Roamer V3

- Test with actual robot hardware.
- Implement TF2 integration for proper pose tracking.
- Add current-pose monitoring for better space classification.
- Implement wall-following behavior for room patrolling.
- Add obstacle avoidance integration.
  _Source: `Sigyn/TODO.md`, `sigyn_teensy_boards/TODO.md`_

---

## 🟡 MEDIUM — General / ROS 2

- Predict battery discharge time based on current draw trend (INA226 data).
- Implement heartbeat / watchdog from `sigyn_to_teensy`: detect if the ROS side
  stops sending heartbeats and raise a warning fault (not an emergency stop).
- IMU: detect critical tilt in X or Y axis and report to ROS 2.
- `nav2_bringup.launch.py` L65 / `navigation_launch.py` L63: Substitute
  `PushNodeRemapping` (upstream Nav2 TODO — evaluate if still relevant).

---

## 🟢 LOW — Testing Coverage (12+ Months)

- Expand mock framework for: RoboClaw serial protocol, VL53L0X, IMU.
- Current coverage: ~60% (SafetyCoordinator, Temperature, Battery tested).
- Target: 90%+ coverage for all modules.
- Add GPIO interrupt mock for cross-board e-stop tests.
  _Source: `sigyn_teensy_boards/TODO.md`_

---

## 🔵 FUTURE / Research

- Machine learning anomaly detection using historical sensor data.
- Predictive maintenance from gradual sensor degradation patterns.
- Multi-robot coordination: share safety status between Sigyn units.
- Remote safety monitoring dashboard (cloud or local).
- Evaluate micro-ROS as a replacement for the current JSON serial bridge.

---

## ✅ Recently Completed (context only — do not re-implement)

| Date | Item |
|------|------|
| 2026-02-08 | Board 3: `STEPPOS`, `STEPHOME`, `STEPSTATUS` commands |
| 2026-02-08 | Board 3: `STEPPERSTAT3` JSON message (elevator/extender position + limits) |
| 2026-02-08 | `GripperStatus.msg` and `GripperPositionCommand.msg` added to `sigyn_interfaces` |
| 2026-02-08 | `MoveElevator.action` and `MoveExtender.action` servers in `sigyn_to_sensor_v2` |
| 2026-02-08 | BT: `MoveElevatorAction`, `StepElevatorUpAction`, `ElevatorAtHeight` nodes |
| 2026-02-08 | Visual servoing loop: step 2 cm → check camera → repeat until target height |
| 2026-02-08 | `/gripper/status` publisher, `/gripper/position/command` + `/gripper/home` subscribers |
