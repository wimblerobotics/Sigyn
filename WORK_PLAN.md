# Sigyn Robot — Consolidated Work Plan

**Last Updated:** 2026-03-31  
**Branch:** sigyn2  
**Purpose:** Single authoritative source for all outstanding work across the Sigyn robotic platform

---

## AI Context Summary

This work plan covers the complete Sigyn robotic platform, including:
- **Hardware:** Teensy 4.1-based multi-board control system (Boards 1 & 2: Navigation/Safety, Power/Sensors)
- **Core Packages:** `wr_ros_teensy` (PC bridge), `wr_teensy_boards` (firmware), `wr_interfaces`, `wr_proto_msgs`, `sigyn_notifier`
- **Navigation:** Nav2-based autonomous navigation, perimeter patrol, house patrol
- **Vision:** OAK-D depth cameras for object detection
- **Safety:** Multi-level fault handling (FaultCoordinator), emergency stop, sensor monitoring
- **Note:** Board 3 (gripper/elevator) hardware is being replaced and all related work items have been removed from this plan

**Key Architecture Notes:**
- ROS 2 Jazzy on Ubuntu 24.04
- Behavior Trees for mission logic (BT.CPP v4)
- Multi-board Teensy system with JSON serial protocol
- FaultCoordinator pattern for multi-fault e-stop management
- Nav2 for obstacle-aware navigation

**Getting Started:**
- Main launch: `ros2 launch sigyn_bringup sigyn.launch.py`
- Behavior trees in `sigyn_behavior_trees` package
- Hardware bridge: `wr_ros_teensy` node
- Firmware: `wr_teensy_boards` (separate repo, PlatformIO)
- Can challenge: `ros2 launch can_do_challenge can_do_challenge_launch.py`

---

## 🔴 CRITICAL — Must Fix Before Production

### Safety System

#### E-Stop Pull-Down (Fail-Safe Wiring)
- **Status:** CRITICAL hardware safety requirement
- **Issue:** Motor e-stop line must fail to safe state if Teensy loses power or wire breaks
- **Required:**
  - Review hardware schematic
  - Verify e-stop line pulled LOW by default
  - Test with power disconnection scenarios
- **Impact:** Ensures robot stops if control system fails
- **Files:** Hardware schematic, `board1/board1_main.cpp`
- **Estimated Effort:** 2 hours review + hardware fix if needed

#### Sensor Timeout Safety
- **Status:** CRITICAL safety gap
- **Issue:** Sensors that stop updating (I²C hang, disconnection) return last value forever
- **Required:**
  - Add timestamp tracking to all sensor modules
  - Trigger `isUnsafe()` if sensor hasn't updated within 2x expected period
  - Test with sensor disconnection scenarios
- **Impact:** Prevents stale sensor data from causing unsafe commands
- **Files:** `sigyn_teensy_boards/common/modules/*_monitor.*`
- **Estimated Effort:** 6-8 hours (affects multiple modules)

---

## 🔴 HIGH — Safety System (Next 6 Months)

### Board Reconnection Recovery
- **Purpose:** Allow Board 1 or Board 2 to recover from temporary disconnection
- **Current State:** System restart required if either board goes offline (per 2026-03-13 architecture decision)
- **Design Considerations:**
  - Board 1 tracks Board 2 heartbeat loss as a fault
  - On reconnection, synchronize fault state (Board 2 → Board 1)
  - Handle e-stop state recovery (was GPIO asserted before disconnect?)
  - Test scenarios: UART disconnect, Board 2 power cycle, firmware upload
- **Testing:** Comprehensive reconnection test suite
- **Estimated Effort:** 12-16 hours

### 🔴 VL53L0X Outstanding Items
*Prioritize these as BT and navigation work progresses.*

#### VL53L0X — Runtime Threshold Tuning via ROS 2 Parameter Server
- **Status:** Deferred — thresholds are compile-time constants for now
- **Purpose:** Allow `estop_mm` / `warning_mm` / `hysteresis_mm` to be adjusted via
  `ros2 param set` without reflashing firmware
- **Design:** Requires a PC→Teensy `CONFIG` command pathway (see `ResponseFactory::ConfigAck`),
  a `set_parameters_callback` in `teensy_bridge.cpp`, and firmware-side parameter storage
- **Note:** No dynamic parameter server integration exists yet in `wr_ros_teensy`
- **Files:** `wr_ros_teensy/src/teensy_bridge.cpp`, `wr_teensy_boards/modules/vl53l0x/vl53l0x_monitor.h`
- **Estimated Effort:** 8-12 hours

#### VL53L0X — Direction-Aware Safety Gating (PC Side)
- **Status:** Deferred — **CANDIDATE FOR REMOVAL** (see note below)
- **Note (Q6):** `cmd_vel` is available via the `RoboclawMonitor` on the Teensy, so the signal
  path exists on the robot. However the value add is uncertain: the Teensy safety estop is
  a last-resort hardware override, not a navigation fence — suppressing it based on direction
  may be more dangerous than helpful. Re-evaluate before implementing.
- **Purpose:** Suppress forward-facing sensor faults when the robot is reversing (and vice versa),
  preventing needless emergency stops when backing away from a close object
- **Design:** Implement in `SafetyCoordinator` (PC side — Teensy does not receive cmd_vel)
  - If `cmd_vel.linear.x > 0`: only instances 3, 4 (`front_*_fwd`) contribute to ring2 estop
  - If `cmd_vel.linear.x < 0`: only instances 0, 7 (`rear_*_bkwd`) contribute
  - Side sensors (1, 2, 5, 6) always active for lateral obstacle detection
  - Requires `SafetyCoordinator` to track last cmd_vel direction and know sensor→direction map
- **Files:** `wr_ros_teensy/src/SafetyCoordinator.cpp`, `wr_ros_teensy/src/FaultRegistry.cpp`
- **Estimated Effort:** 6-8 hours

#### VL53L0X — `allow_close_approach` Behavior-Tree Flag
- **Status:** Deferred
- **Purpose:** Allow docking / close-approach manoeuvres to suppress inner-ring estop
- **Design:** BT condition flag → PC topic → wr_ros_teensy → inhibit VL53L0X_RING2 fault
  escalation to ESTOP (downgrade to WARNING only during docking)
- **Depends on:** Direction-aware gating item above
- **Estimated Effort:** 4-6 hours

#### VL53L0X — Sensor Timeout / Stale Data Safety
- **Status:** Deferred (part of broader sensor-timeout safety work item)
- **Purpose:** Trigger `VL53L0X_RING2` fault if any sensor stops updating for > 2× expected period
  (detects I²C hang, cable disconnection, firmware crash)
- **Files:** `wr_teensy_boards/modules/vl53l0x/vl53l0x_monitor.cpp`
- **Estimated Effort:** 3-4 hours (builds on the broader sensor-timeout work item)

#### VL53L0X — Custom Low-Level Driver (Performance Option)
- **Status:** Future consideration — do not start until navigation lag is measurable
- **Purpose:** Replace Pololu Arduino library with a direct register-level driver to reduce
  per-sensor read latency and increase per-sensor update rate above ~10 Hz
- **Trigger:** Profile actual loop rates after full navigation stack integration
- **Files:** New `wr_teensy_boards/modules/vl53l0x/vl53l0x_driver.h/.cpp`
- **Estimated Effort:** 16-24 hours (driver + validation tests)

### System Shutdown on Low Battery
- **Purpose:** Graceful power-down before battery damage
- **Hardware:** INA226 sensors on both batteries, relay on GPIO 32
- **Trigger:** Battery < 30V sustained for > 30 seconds (both INA226 must agree)
- **Shutdown Sequence:**
  1. ROS 2 notification to save state
  2. Motor e-stop assertion
  3. SD card flush
  4. Main battery relay cut (GPIO 32)
- **Implementation:**
  - Add `SYSTEM_SHUTDOWN` severity level beyond `EMERGENCY_STOP`
  - Config in `battery_monitor.h`
  - Not self-recovering (requires manual restart or charger)
- **Testing:** Mock battery data only (dangerous to test with real low battery)
- **Estimated Effort:** 8-12 hours

---

## 🟠 HIGH — Firmware Architecture

### Full Architectural Review
- **Purpose:** Validate current design before building more on top
- **Questions to Answer:**
  - Is three-board split still correct?
  - Should we migrate from JSON serial to micro-ROS?
  - Does module registration/lifecycle pattern scale?
  - Is SafetyCoordinator the right single point of truth?
  - Should compile-time flags become runtime config?
- **Deliverables:**
  - Updated `docs/ARCHITECTURE.md`
  - Updated `docs/Safety_System.md`
  - Decision document on JSON vs micro-ROS
- **Estimated Effort:** 16-24 hours (investigation + documentation)

### Serial Message Protocol Redesign
- **Issue:** Current JSON protocol (~100-300 bytes/frame at 85Hz) near bandwidth limits
- **Options to Evaluate:**
  1. Compact JSON (abbreviated key names)
  2. Hybrid: short fixed-prefix + minimal JSON
  3. Binary framing (length-prefixed structs + type byte)
  4. CBOR / MessagePack
- **Constraints:**
  - Must remain debuggable via serial terminal
  - `message_parser.cpp` must be updated simultaneously
  - Document in `docs/Message_Formats.md`
- **Decision Criteria:** Bandwidth savings vs development/debugging cost
- **Estimated Effort:** 20-30 hours (redesign + implementation + testing)

### module.cpp Registration Failure Handling
- **L76:** Decide whether module registration failure should trigger e-stop or fault indicator
- **Current:** Just logged, no safety action
- **Options:** EMERGENCY_STOP, FAULT, or dependency-based decision
- **Files:** `sigyn_teensy_boards/common/core/module.cpp`
- **Estimated Effort:** 2-4 hours

---

## 🟠 HIGH — ROS 2 Integration / Code Quality

### Proximity Sensor Topic Architecture for Nav2
- **Issue:** All VL53L0X sensors publish to single aggregated `/sigyn/sensors/range` topic
- **Problem:** Nav2's range_sensor_layer and costmap_2d require one topic per sensor or merged point cloud
- **Impact:** Cannot use Nav2 obstacle avoidance without demux layer
- **Design Options:**
  1. One publisher per sensor (e.g., `/sigyn/sensors/range/front_left`, `/sigyn/sensors/range/front_right`)
  2. Single `sensor_msgs/PointCloud2` aggregated topic
- **Current:** Publishers created per-sensor in TopicPublisher constructor but all use same topic
- **Required:** Decision on topic architecture + implementation + URDF/TF updates
- **Files:** `wr_ros_teensy/src/TopicPublisher.cpp`, sensor_names.json, URDF
- **Estimated Effort:** 8-12 hours (design + implementation + testing)

---

## 🟠 HIGH — Can-Do Challenge

### Behavior Tree Safety Preservation
- **Issue:** Some places use `Sequence` where `ReactiveSequence` needed
- **Requirement:** Safety subtrees must remain responsive during long actions
- **Review:** All BT XMLs in `can_do_challenge/bt_xml/`
- **Testing:** Trigger e-stop during long operations
- **Estimated Effort:** 6-8 hours

### Nav2 Integration (Unimplemented Stubs)
- **bt_nodes.cpp L998 / bt_nodes_real.cpp L1235:** Implement `ComputePathToPose`
- **bt_nodes.cpp L1007 / bt_nodes_real.cpp L1244:** Implement `FollowPath`
- **bt_nodes.cpp L2411 / bt_nodes_real.cpp L3076:** Implement navigation to charging dock
- **bt_nodes.cpp L2430 / bt_nodes_real.cpp L3095:** Implement e-stop topic publish
- **Files:** `can_do_challenge/src/bt_nodes.cpp`, `bt_nodes_real.cpp`
- **Testing:** Full navigation scenarios
- **Estimated Effort:** 16-20 hours

### Hardware Topic Subscriptions (Unimplemented Stubs)
- **bt_nodes.cpp L411 / bt_nodes_real.cpp L492:** Subscribe to real hardware topics
- **bt_nodes.cpp L797 / bt_nodes_real.cpp L906:** Check gripper force sensor / verify can visible
- **Files:** `can_do_challenge/src/bt_nodes_real.cpp`
- **Testing:** Run with real hardware, verify callbacks triggered
- **Estimated Effort:** 4-8 hours

### Sim ↔ Real Parity
- **Purpose:** Allow testing in simulation before real hardware runs
- **Issues:**
  - `WaitForNewOAKDFrame`: sim uses fixed delay, real uses heartbeat
  - `MoveTowardsCan`: sim is `StatefulActionNode`, real is `SyncActionNode`
  - `OAKDDetectCan`: missing in sim
  - `initializeObjectDetection`: different topics/message types
  - `getFreshPiDetection`: sim blocking, real non-blocking
  - `piDetectionInRange`: different Z defaults
  - `ObjectDetectionState`: different field defaults
- **Required:** Full parity review and alignment
- **Files:** `can_do_challenge/src/bt_nodes.cpp` vs `bt_nodes_real.cpp`
- **Testing:** Run same XML in sim and real, verify matching behavior
- **Estimated Effort:** 12-20 hours
Desired:** Single `IsFaultActive` node with `target_fault` input port
- **Benefits:** Cleaner BT XMLs, easier to add new fault types
- **Implementation:**
  - Create `IsFaultActive` condition node in `sigyn_behavior_trees`
  - Takes `target_fault` string input port
  - Queries fault system via service or blackboard
- **Files:** `sigyn_behavior_trees/src/`, behavior tree node registration
- **Testing:** BT XMLs using multiple fault checks
- **Estimated Effort:** 4-6 hours

### Complete sigyn_behavior_trees Extraction
- **Status:** Repo created but not functional
- **Blocking Issues:**
  - Action servers never call `goal_handle->succeed()` → hangs
  - Wrong include pattern: `#include "SaySomethingActionServer.cpp"` (ODR violation)
  - Double `return 0;` in `bt_test2_main.cpp`
  - Wrong namespace comment in `SS.hpp`
  - Raw `new BT::NodeConfiguration` instead of `std::make_unique`
  - `package.xml`: wrong dep `nav2_behaviors` (should be `nav2_behavior_tree`), missing `tf2_ros`
  - `CMakeLists.txt`: action server libs need `SHARED`, unconditional `-g`, old-style `include_directories()`
- **Cleanup:**
  - Remove `launch/xxxbt1.launch.py`
  - Remove dead comments
  - Remove `notes.md` (stale)
  - Add SPDX headers
  - Apply clang-format (Google style)
- **Deliverables:**
  - Push working repo to `wimblerobotics/sigyn_behavior_trees`
  - Update `Sigyn/packages.yaml`
- **Estimated Effort:** 12-16 hours

---

## 🟡 MEDIUM — RoboClaw

### Localization vs Encoder Movement Check
- **Purpose:** Detect wheels spinning without displacement (stuck/slipping)
- **Implementation:**
  - Cross-check localization data against encoder deltas
  - Raise WARNING if mismatch exceeds threshold
- **Files:** `sigyn_teensy_boards/board1/roboclaw_monitor.cpp`
- **Testing:** Lift robot, spin wheels, verify fault
- **Estimated Effort:** 6-8 hours

### Power Cycle Capability
- **Purpose:** Clear latching hardware faults in RoboClaw without manual intervention
- **Hardware:** `PIN_RELAY_ROBOCLAW_POWER` (already wired)
- **Implementation:**
  - Add power cycle function: relay off → 2sec delay → relay on
  - Trigger on specific fault conditions (see manual L47 for which faults latch)
- **Files:** `wr_teensy_boards/modules/roboclaw/roboclaw_monitor.cpp`
- **Testing:** Trigger latching fault, verify auto-recovery
- **Estimated Effort:** 6-8 hours

### RoboClaw Temperature Hysteresis
- **Current:** Over-temperature immediately asserts e-stop at `roboclaw_temp_fault_c`; no path to auto-recovery
- **Required:** Add clearance threshold (e.g., `temp_fault_c - 10°C`) before auto-clearing the thermal fault
- **Files:** `wr_teensy_boards/modules/roboclaw/roboclaw_monitor.cpp`
- **Estimated Effort:** 2-3 hours

---

## 🟡 MEDIUM — Perimeter Roamer V3

- **Status:** Code exists but untested on hardware
- **Location:** `sigyn_perimeter_roamer` package
- **Required:**
  - Test with actual robot hardware
  - Implement TF2 integration for proper pose tracking
  - Add current-pose monitoring for space classification
  - Implement wall-following behavior for room patrolling
  - Add obstacle avoidance integration
- **Estimated Effort:** 20-30 hours

---

## 🟡 MEDIUM — General / ROS 2

### Battery Discharge Prediction
- **Purpose:** Estimate remaining runtime based on current draw trend
- **Data:** INA226 current measurements
- **Implementation:**
  - Track rolling average of current draw
  - Compute time to `battery_critical_voltage` at current rate
  - Publish as diagnostic message
- **Files:** `sigyn_teensy_boards/board1/battery_monitor.cpp`, `sigyn_to_teensy`
- **Testing:** Load robot with tasks, verify predictions reasonable
- **Estimated Effort:** 6-8 hours

### Heartbeat/Watchdog from ROS
- **Purpose:** Detect if ROS side stops sending commands
- **Current:** PC already sends 1 Hz HB messages (`hb_timer_` in `teensy_bridge.cpp`). Teensy `Heartbeat` module stores host timestamp on receipt.
- **Missing:** Teensy does NOT raise WARNING if host heartbeat is stale > 5 seconds — `Heartbeat::Loop()` has no staleness check
- **Required:**
  - Add stale check to `Heartbeat::Loop()`: if `(millis() - last_local_time_ms_) > kStaleThresholdMs` and protocol agreement reached, raise WARNING via FaultCoordinator
  - Test by killing `wr_ros_teensy` node and verifying WARNING on serial
- **Files:** `wr_teensy_boards/modules/heartbeat.h`, `heartbeat.cpp`
- **Estimated Effort:** 2-3 hours

### Nav2 Configuration: AMCL Tuning
- **Issue:** AMCL update thresholds too aggressive (update_min_d=0.01m, update_min_a=0.01rad)
- **Symptoms:** Odom jumps from sensor noise
- **Recommendation:**
  - Increase `update_min_d` from 0.01m to 0.10m
  - Increase `update_min_a` from 0.01 rad to 0.15 rad
- **Testing:** Monitor /tf, test in long hallways
- **Files:** `sigyn_bringup/config/navigation.yaml`
- **Estimated Effort:** 2-3 hours (testing)

### Nav2 Configuration: Initial Pose
- **Issue:** `set_initial_pose=true` with hardcoded [0,0,0] problematic
- **Recommendation:** 
  - Change `set_initial_pose` to false
  - Use RViz "2D Pose Estimate" tool
- **Testing:** Power on in different locations, verify AMCL convergence
- **Files:** `sigyn_bringup/config/navigation.yaml`
- **Estimated Effort:** 1-2 hours

---

## 🟢 LOW — Testing Coverage (12+ Months)

### Expand Mock Framework
- **Current:** 176 tests passing; all 6 production modules DI-refactored; mocks exist for ISerialSink, IFaultReporter, IInterboardSink, IEstopController, IPowerSensor, ISerialManager, IProtocolGate, IRoboClaw
- **Target:** 90%+ coverage for all modules
- **Missing Mocks:**
  - VL53L0X I2C
  - BNO055 IMU (state machine level, beyond pure tilt math)
  - GPIO interrupts
  - Arduino `Serial` wrapper (`ISerial`) for SerialManager testability
- **Files:** `wr_teensy_boards/test/`
- **Estimated Effort:** 20-30 hours

---

## 🔵 FUTURE / RESEARCH

### Machine Learning Anomaly Detection
- **Purpose:** Detect unusual sensor patterns indicating problems
- **Data:** Historical sensor logs
- **Estimated Effort:** 80+ hours (research project)

### Predictive Maintenance
- **Purpose:** Detect gradual sensor degradation before failure
- **Estimated Effort:** 60+ hours

### Multi-Robot Coordination
- **Purpose:** Share safety status between multiple Sigyn units
- **Estimated Effort:** 40+ hours

### Remote Safety Monitoring Dashboard
- **Purpose:** Cloud or local dashboard for real-time safety status
- **Estimated Effort:** 30-40 hours

### Micro-ROS Evaluation
- **Purpose:** Replace JSON serial bridge with native ROS 2 micro-ROS
- **Benefits:** Lower latency, better integration, type safety
- **Risks:** Debugging difficulty, memory overhead
- **Estimated Effort:** 60-80 hours (investigation + prototype)

---

## Quick Reference

### Key Repositories
- `wimblerobotics/Sigyn` — Main monorepo (this file)
- `wimblerobotics/wr_teensy_boards` — Firmware (PlatformIO)
- `wimblerobotics/wr_ros_teensy` — PC-side hardware bridge (ROS 2)
- `wimblerobotics/wr_interfaces` — Custom ROS 2 message/service types
- `wimblerobotics/wr_proto_msgs` — Serial wire protocol library
- `wimblerobotics/sigyn_notifier` — Telegram notification node
- `wimblerobotics/sigyn_behavior_trees` — BT nodes (in progress)
- `wimblerobotics/can_do_challenge` — Can pickup challenge

### Key Files
- Safety firmware: `wr_teensy_boards/common/fault_coordinator.h/.cpp`
- Board 1 main: `wr_teensy_boards/src/board1_main.cpp`
- Board 2 main: `wr_teensy_boards/src/board2_main.cpp`
- PC bridge: `wr_ros_teensy/src/teensy_bridge.cpp`
- PC fault registry: `wr_ros_teensy/src/FaultRegistry.cpp`
- Notifier: `sigyn_notifier/sigyn_notifier/notifier_node.py`
- Nav Config: `sigyn_bringup/config/navigation.yaml`
- BT XML: `can_do_challenge/bt_xml/*.xml`

### Key Commands
```bash
# Build all
cd ~/sigyn_ws && colcon build --symlink-install

# Launch robot
ros2 launch sigyn_bringup sigyn.launch.py

# Run can challenge
ros2 launch can_do_challenge can_do_challenge_launch.py

# Flash firmware (from wr_teensy_boards repo)
pio run -e board1 -t upload
pio run -e board2 -t upload

# Run firmware unit tests (no hardware needed)
cd ~/sigyn_ws/src/wr_teensy_boards && pio test -e native_test

# Check active faults
ros2 topic echo /sigyn/safety/fault_list
```

---

**For new AI chat sessions:** Read this entire file to understand the current state of the project and outstanding work items. Priority order is indicated by emoji (🔴 Critical, 🟠 High, 🟡 Medium, 🟢 Low, 🔵 Future).
