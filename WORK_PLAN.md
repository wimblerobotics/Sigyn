# Sigyn Robot — Consolidated Work Plan

**Last Updated:** 2026-03-15  
**Branch:** sigyn2  
**Purpose:** Single authoritative source for all outstanding work across the Sigyn robotic platform

---

## AI Context Summary

This work plan covers the complete Sigyn robotic platform, including:
- **Hardware:** Teensy 4.1-based multi-board control system (Boards 1 & 2: Navigation/Safety, Power/Sensors)
- **Core Packages:** sigyn_bringup, sigyn_behavior_trees, sigyn_to_teensy, sigyn_teensy_boards
- **Navigation:** Nav2-based autonomous navigation, perimeter patrol, house patrol
- **Vision:** OAK-D depth cameras for object detection
- **Safety:** Multi-level fault handling (FaultCoordinator), emergency stop, sensor monitoring
- **Note:** Board 3 (gripper/elevator) hardware is being replaced and all related work items have been removed from this plan

**Key Architecture Notes:**
- ROS 2 Humble on Ubuntu 22.04
- Behavior Trees for mission logic (BT.CPP v4)
- Multi-board Teensy system with JSON serial protocol
- FaultCoordinator pattern for multi-fault e-stop management
- Nav2 for obstacle-aware navigation

**Getting Started:**
- Main launch: `ros2 launch sigyn_bringup sigyn.launch.py`
- Behavior trees in `sigyn_behavior_trees` package
- Hardware bridge: `sigyn_to_teensy` node
- Firmware: `sigyn_teensy_boards` (separate repo, PlatformIO)
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

### VL53L0X Collision Prediction (Board 1)
- **Purpose:** Direction-aware obstacle detection for collision prevention
- **Hardware:** 8 VL53L0X sensors on Board 1 (already present)
- **Thresholds:**
  - WARNING at 500mm → slow down
  - EMERGENCY_STOP at 200mm → immediate stop
  - Hysteresis: 600mm clearance before recovery
- **Implementation:**
  - Extend `VL53L0XMonitor` with collision prediction
  - Direction-aware: only trigger when obstacle is in direction of motion
  - Add `allow_close_approach` behavior-tree flag for docking
  - Config parameters in `vl53l0x_monitor.h`
- **Testing:** Mock sensor data + real wall-approach runs
- **Estimated Effort:** 12-16 hours

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

## ✅ Recently Completed (For Context)

These items are done and should not be re-implemented:

| Date | Item |
|------|------|
| 2026-02-08 | Board 3: STEPPOS, STEPHOME, STEPSTATUS commands |
| 2026-02-08 | Board 3: STEPPERSTAT3 JSON message with position/limits |
| 2026-02-08 | GripperStatus.msg and GripperPositionCommand.msg in sigyn_interfaces |
| 2026-02-08 | MoveElevator.action and MoveExtender.action servers in sigyn_to_teensy |
| 2026-02-08 | MoveElevatorAction BT node with action client integration |
| 2026-02-08 | StepElevatorUpAction for incremental visual servoing |
| 2026-02-08 | ElevatorAtHeight condition for pixel-based feedback |
| 2026-03-13 | FaultCoordinator fault tracking and e-stop coordination (Boards 1 & 2) |
| 2026-03-13 | BNO055Monitor tilt (>10°) and fallen (>20°) safety thresholds |
| 2026-03-13 | test_fault_coordinator, test_heartbeat, test_module_framework suites (wr_teensy_boards) |
| 2026-03-13 | test_sensor_name_table.cpp and expanded test_message_dispatcher.cpp (wr_ros_teensy) |
| 2026-03-14 | DI interfaces: ISerialSink, IFaultReporter, IInterboardSink, IEstopController |
| 2026-03-14 | DI mocks: MockSerialSink, MockFaultReporter, MockInterboardSink, MockEstopController, MockPowerSensor |
| 2026-03-14 | FaultCoordinator DI refactored (17 new unit tests) |
| 2026-03-14 | BatteryMonitor DI refactored (13 new unit tests) |
| 2026-03-14 | Module.ResetForTesting() + Reregister() added (11 new unit tests) |
| 2026-03-14 | Total embedded tests: 128 all passing |
| 2026-03-15 | serial_manager.cpp TODOs eliminated by registered-handler dispatch pattern |
| 2026-03-15 | Thread Safety: Bridge receive queue fully implemented (lock_guard + swap pattern) |
| 2026-03-15 | Dead code cleanup: GetReasonList(), rclcpp::Clock(RCL_STEADY_TIME), MultiThreadedExecutor all removed |
| 2026-03-15 | IMU Tilt Detection: BNO055 publishes sensor_msgs/Imu via TopicPublisher at /sigyn/sensors/imu_* |
| 2026-03-15 | RoboClaw Temperature Monitoring: CheckTemperature() asserts e-stop above roboclaw_temp_fault_c |
| 2026-03-15 | RoboClaw Status Publishing: RCLAW + ODOM handlers publish sigyn/roboclaw/status and odometry |
| 2026-03-15 | Encoder Read Failure Escalation: CheckCommFailures() asserts e-stop after max_comm_failures and resets to kConnecting |

---

## Quick Reference

### Key Repositories
- `wimblerobotics/Sigyn` - Main monorepo (this file)
- `wimblerobotics/sigyn_teensy_boards` - Firmware (PlatformIO)
- `wimblerobotics/sigyn_behavior_trees` - BT nodes (in progress)
- `wimblerobotics/sigyn_to_teensy` - Hardware bridge
- `wimblerobotics/can_do_challenge` - Can pickup challenge

### Key Files
- Safety: `sigyn_teensy_boards/common/core/safety_coordinator.*`
- Firmware: `sigyn_teensy_boards/board*/board*_main.cpp`
- Bridge: `sigyn_to_teensy/src/message_parser.cpp`
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

# Flash firmware (from sigyn_teensy_boards repo)
platformio run -e board1 -t upload

# Check safety status
ros2 topic echo /teensy/safety_status
```

---

**For new AI chat sessions:** Read this entire file to understand the current state of the project and outstanding work items. Priority order is indicated by emoji (🔴 Critical, 🟠 High, 🟡 Medium, 🟢 Low, 🔵 Future).
