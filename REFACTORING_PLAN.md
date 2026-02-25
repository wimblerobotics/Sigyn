# Sigyn Monorepo — Outstanding Work

**Updated:** 2026-02-24 | **Branch:** `sigyn2`

---

## Active Packages in This Monorepo

| Package | Repo / Location | Status |
|---|---|---|
| `sigyn_bringup` | this repo | Clean; minor config review remaining |
| `rviz` | this repo | Clean |
| `sigyn_behavior_trees` | `wimblerobotics/sigyn_behavior_trees` | Extraction in progress |
| `sigyn_to_teensy` | `wimblerobotics/sigyn_to_teensy` | Extracted; rework needed |
| `sigyn_teensy_boards` | `wimblerobotics/sigyn_teensy_boards` | Extracted; CRITICAL safety work needed |
| `sigyn_oakd_detection` | `wimblerobotics/sigyn_oakd_detection` | Extracted; Sigyn2 update pending |
| `can_do_challenge` | `wimblerobotics/can_do_challenge` | Extracted; BT v4 migration + cleanup needed |
| `pi_gripper` | sigynVision Pi 5 only | Needs cleanup |

---

## sigyn_bringup

- [ ] `config/bt1.xml` — review content; currently a dummy test tree, not used in production

---

## sigyn_behavior_trees (IN PROGRESS)

New repo `wimblerobotics/sigyn_behavior_trees` — initial commit made. Items below are blocking a working extraction:

- [ ] Fix action servers never calling `goal_handle->succeed()` — execution hangs
- [ ] Fix `#include "SaySomethingActionServer.cpp"` in `bt_test1_main.cpp` — ODR violation
- [ ] Fix double `return 0;` and wrong node name in `bt_test2_main.cpp`
- [ ] Fix wrong namespace comment in `SS.hpp`
- [ ] Replace raw `new BT::NodeConfiguration` with `std::make_unique`
- [ ] Fix `package.xml`: `nav2_behaviors` → `nav2_behavior_tree`; add missing `tf2_ros`; remove nonexistent `behavior_plugin.xml` export
- [ ] Fix `CMakeLists.txt`: action server libs need `SHARED`; remove unconditional `-g`; old-style `include_directories()`
- [ ] Remove `launch/xxxbt1.launch.py` and dead comments in all `.cpp` files
- [ ] Remove `notes.md` (stale single snippet)
- [ ] Add SPDX copyright headers to all files
- [ ] Apply Google style + clang-format
- [ ] Push to `wimblerobotics/sigyn_behavior_trees` on GitHub
- [ ] Update `Sigyn2/packages.yaml` to reference new repo

---

## sigyn_teensy_boards

- [ ] **CRITICAL**: Enable Safety on Board 3 (Gripper): set `BOARD_HAS_SAFETY 1`
- [ ] **CRITICAL**: Fix inter-board communication: implement `fault_handler` in `board1_main.cpp`
- [ ] **CRITICAL**: Sensor timeout safety: ensure sensors trigger `isUnsafe` on stall
- [ ] Review `modules/` and `common/` for deprecated or dead code
- [ ] Add SPDX copyright headers to all source files

---

## sigyn_to_teensy

- [ ] Update `Sigyn2/packages.yaml`: add `wimblerobotics/sigyn_to_teensy` to `sigyn_hardware` group
- [ ] Add unit tests for `message_parser.cpp` (`ament_cmake_gtest`)
- [ ] Add a mock serial port test to verify the bridge correctly publishes sensor data
- [ ] Wire up `BUILD_TESTING` in `CMakeLists.txt`
- [ ] Rework with dependency injection; rearchitect to align with `sigyn_teensy_boards` changes

---

## sigyn_oakd_detection

- [ ] Update `Sigyn2/packages.yaml`: add `wimblerobotics/sigyn_oakd_detection` to `sigyn_vision` group

---

## can_do_challenge

### Dependencies

- [ ] Migrate from `behaviortree_cpp_v3` to `behaviortree_cpp` (v4 API) — significant refactor; key diffs: `BT::NodeStatus`, `BT::InputPort`, action node base class names
- [ ] Resolve `<depend>gripper_camera_detector</depend>` — package does not exist; determine if it should be `pi_can_detector` or removed
- [ ] Review whether `depthai_ros_msgs` is still needed (OAK-D detection now via `sigyn_oakd_detection`)

### Dead code / files to remove

- [ ] `bt_xml/old.xml` — explicitly stale
- [ ] `bt_xml/main.orig.xml` — backup of main.xml
- [ ] `bt_xml/oakd_detection_test1.xml` — development test tree
- [ ] `resources/calibration_imgs/` — ~30 JPEG images; move to external storage

### Launch consolidation

- [ ] Audit `step1_real_launch.py` … `step5_elevator_height_launch.py` — keep only those still needed; remove the rest
- [ ] `can_do_challenge_launch.py` should be the canonical "run the challenge" entry point

### Code cleanup

- [ ] Add SPDX copyright headers to all files
- [ ] Apply Google style + clang-format to C++ files in `include/`
- [ ] Apply PEP 8 to Python files
- [ ] Audit `bt_nodes.hpp` vs `bt_nodes_real.hpp` — consolidate if possible
- [ ] Document `config/can_locations.json` (coordinate frame, field definitions)

---

## Sigyn2 Deployment (`packages.yaml`)

- [ ] Add `sigyn_behavior_trees` group: `wimblerobotics/sigyn_behavior_trees`
- [ ] Add `wimblerobotics/sigyn_to_teensy` to `sigyn_hardware` group
- [ ] Add `sigyn_vision` group: `wimblerobotics/sigyn_oakd_detection`

---

## pi_gripper

- [ ] Add SPDX copyright headers to all source files
- [ ] Fix `package.xml` maintainer if still placeholder
- [ ] Apply PEP 8 / `ruff` to all Python source
- [ ] Add `README.md`: hardware wiring (PCA9685 I2C address, servo channels), udev rules, deployment
- [ ] Verify `Sigyn2/packages.yaml` `pi_gripper` group entry has correct git URL and branch
- [ ] Ensure any action interface is defined in `sigyn_interfaces`, not privately in this repo
- [ ] Confirm topic/service names match what `sigyn_to_teensy` and `can_do_challenge` expect
- [ ] Add `pytest` unit tests for servo math/limit-checking logic

---

## Style — Cross-cutting

### Python

- [ ] Apply `ruff` / `black` to all Python in extracted repos (tracked per-repo)

### C++ (applies to `sigyn_behavior_trees` and `can_do_challenge`)

- [ ] Rename files to `snake_case.cpp` / `snake_case.hpp` (e.g. `SaySomethingActionServer.cpp` → `say_something_action_server.cpp`)
- [ ] Rename methods to `snake_case()`
- [ ] Use `trailing_underscore_` for member variables consistently
- [ ] Run `clang-format` (top-level `.clang-format` already exists: Google style, `ColumnLimit: 100`)

### Nav2 / ROS 2 notes

- [ ] Verify `can_do_challenge` uses `nav2_behavior_tree::BtActionNode` base class after v4 migration (not raw `BT::ActionNodeBase`)

---

## Testing

### sigyn_to_teensy

- [ ] `ament_cmake_gtest` tests for `message_parser.cpp` — unit test all message format strings
- [ ] Mock serial port test: verify bridge publishes correct sensor data

### sigyn_behavior_trees

- [ ] `ament_cmake_gtest` tests for each BT node class
- [ ] SaySomething: mock action server, verify `on_tick()` sets `goal_.message`
- [ ] MoveAShortDistanceAhead: verify `initialize()` called; negative distance clamped to 0
- [ ] Use `behaviortree_cpp` built-in test tools for tree tick simulation without a ROS spinner

### can_do_challenge

- [ ] Tests for BT condition nodes (`IsCanDetected`, etc.) with mock topic publishers
- [ ] `pytest` tests for Python detectors (`simple_can_detector.py`, `spatial_detection_annotator.py`)

### sigyn_bringup

- [ ] `ament_xmllint` validation for BT XML files
- [ ] Parameter validation smoke tests for launch files
- [ ] Verify `precheck.launch.py` emits `Shutdown` when devices are missing

---

## Miscellaneous

- [ ] **Sigyn2 `verify_deployment.py`**: script that checks `/dev/teensy_*`, `/dev/lidar_*` exist; ROS domain ID set; all expected packages present via `ros2 pkg list`
- [ ] **Custom nav BT**: `sigyn.launch.py` currently uses stock nav2 `navigate_to_pose` BT XML; bring together `sigyn_bringup/config/bt1.xml` and `sigyn_behavior_trees/config/patrol.xml` into a custom nav BT integrating Sigyn's recovery behaviors
- [ ] **`precheck.launch.py`**: consider making it the canonical robot startup entry point (includes `sigyn.launch.py`) so the robot cannot start without device checks
- [ ] **RViz config**: create one canonical general-purpose config and one OAK-D detection config; `sigyn_oakd_detection` workspace may ship its own
- [ ] **`navigation_launch.py`**: re-diff against nav2 stock version each Jazzy release — it adds the `/cmd_vel_nav` remapping for the twist multiplexer and could diverge
- [ ] **`perimeter_roamer_v3`**: restore to operational state on real robot (mapping launches now working)
