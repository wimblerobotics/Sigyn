# Sigyn Monorepo Refactoring Plan
**Created:** 2026-02-20  
**Status:** In progress — updated 2026-02-24

---

## Vision

Break the Sigyn monorepo into a set of clean, independently deployable repositories that serve as a reference example of how to build a ROS 2 robot. Each repo should be usable standalone and demonstrate best practices for naming, style, testing, and documentation.

---

## Current Repo Inventory

### In `wimblerobotics/Sigyn` (this monorepo, branch `sigyn2`)

| Package | Status | Destination |
|---|---|---|
| ~~`base`~~ | ✅ Renamed → `sigyn_bringup` | Stays here (bringup + launch + maps) |
| ~~`bluetooth_joystick`~~ | ✅ Extracted | Lives at `wimblerobotics/sigyn_bluetooth_joystick`; removal from monorepo pending |
| `can_do_challenge` | Major cleanup needed | Stays here as application layer |
| `rviz` | Minimal, keep | Merge into `sigyn_bringup` or keep standalone |
| `sigyn_behavior_trees` | Extraction in progress | New repo: `wimblerobotics/sigyn_behavior_trees` |
| ~~`sigyn_to_sensor_v2`~~ | ✅ Extracted | Lives at `wimblerobotics/sigyn_to_teensy`; removal from monorepo pending |
| ~~`TeensyV2`~~ | ✅ Extracted | Lives at `wimblerobotics/sigyn_teensy_boards`; removed from monorepo Feb 2026 |
| ~~`yolo_oakd_test`~~ | ✅ Extracted | Lives at `wimblerobotics/sigyn_oakd_detection`; removed from monorepo Feb 2026 |

### Separate repos already in `sigyn_ws/src`

| Repo | Notes |
|---|---|
| `sigyn_description` | Clean, keep separate |
| `sigyn_interfaces` | Clean, keep separate |
| `wr_ldlidar` | Clean, keep separate |
| `wr_teleop_twist_keyboard` | Clean, keep separate |
| `wr_twist_multiplexer` | Clean, keep separate |
| `pi_can_detector` | Lives on sigynVision Pi 5, keep separate |

### Repos deployed only on sigynVision (Pi 5) — not in `sigyn_ws/src`

| Repo | Notes |
|---|---|
| `pi_gripper` | Servo gripper control via PCA9685; deployed on sigynVision; not in sigyn_ws/src |
| `pi_can_detector` | Hailo-8 AI Hat can detector; deployed on sigynVision |

---

## Section 1 — Git / Repo Hygiene

### 1.1 Add missing `.gitignore` entries ✅ DONE

The current `.gitignore` now covers:
- `**/__pycache__/` directories ✅
- `can_do_challenge/resources/calibration_imgs/` ✅
- `CMakePresets.json` ✅ (machine-specific IDE file; `sigyn_bringup/CMakePresets.json` untracked Feb 2026)

### 1.2 Remove committed build artifacts ✅ DONE

`base/out/` was never committed to git (already excluded). Added `base/out/` to `.gitignore`.

### 1.3 Large binary assets in `can_do_challenge/resources/` ✅ DONE

`can_do_challenge/resources/calibration_imgs/` is now in `.gitignore`. The `can_do_challenge` package itself has been extracted to `wimblerobotics/can_do_challenge`.

---

## Section 2 — Rename `base` → `sigyn_bringup` ✅ DONE

**Completed:** February 2026
- [x] Directory renamed: `base/` → `sigyn_bringup/` (`git mv`)
- [x] `CMakeLists.txt`: `project(base)` → `project(sigyn_bringup)`
- [x] `package.xml`: `<name>base</name>` → `<name>sigyn_bringup</name>`
- [x] All `get_package_share_directory("base")` calls in all launch files updated
- [x] `README.md` and `.github/SIGYN_ARCHITECTURE_ONBOARDING.md` `ros2 launch base` → `ros2 launch sigyn_bringup`
- [x] `.github/copilot-instructions.md` updated
- [x] `Sigyn2/config/packages.yaml` comment updated

---

## Section 3 — `base`/`sigyn_bringup` Package Cleanup

### 3.1 Dead launch files to remove ✅ DONE

All dead launch files have been removed. Remaining active launch files:
- `launch/sigyn.launch.py` — main bringup
- `launch/nav2_bringup.launch.py` — called by `sigyn.launch.py`
- `launch/navigation_launch.py` — called by `nav2_bringup.launch.py`
- `launch/precheck.launch.py` — device node precheck
- `launch/map_cartographer.launch.py` — Cartographer SLAM mapping ✅ created
- `launch/map_slam_toolbox.launch.py` — SLAM Toolbox mapping ✅ created
- `launch/sigyn_sim.launch.py` — simulation bringup
- `launch/sub_launch/lidar.launch.py`
- `launch/sub_launch/oakd_yolo26_detector.launch.py`
- `launch/sub_launch/oakd_compressed_republisher.launch.py`

### 3.2 ✅ Mapping launch files — DONE

`launch/map_cartographer.launch.py` and `launch/map_slam_toolbox.launch.py` both exist and have been validated (Feb 2026). Key fixes applied:
- `cartographer.lua`: `use_odometry = true`, `published_frame = "odom"`, `provide_odom_frame = false`, `num_accumulated_range_data = 1`, submap resolution `0.0508` m
- `mapper_params_online_async.yaml`: `base_frame` corrected from `base_footprint` → `base_link` (URDF has no `base_footprint`), resolution `0.0508` m, `correlation_search_space_resolution` aligned to map resolution
- Both launches bring up EKF + Teensy bridge for dead-reckoning between scans
- Both launches include `map_saver_server` (lifecycle-managed) so the current map can be saved on demand via `/map_saver/save_map` service without stopping the mapping session

### 3.3 Separate real-robot and simulation navigation configs ✅ DONE

`navigation_sim.yaml` renamed to `navigation.yaml` (the file was always used for the real robot; the name was misleading). Reference in `sigyn.launch.py` updated.

### 3.4 Clarify which map is current ✅ DONE

Maps directory now contains only:
- `my_map2.pgm` — current operational map (Dec 2024)
- `my_map.yaml` — correctly references `my_map2.pgm`

Old dated maps were already removed. The launch file correctly references `my_map.yaml`. Resolution in `my_map.yaml` is `0.026176231` m (captured at original sensor resolution; different from the `0.0508` setting used when creating new maps with SLAM).

### 3.5 Fix `package.xml` dependency errors ✅ DONE

`package.xml` is now clean: correct `exec_depend` entries only, no spurious `ldlidar`, `python3-cython`, `rosidl_interface_packages`, or unneeded buildtool/exec deps.

### 3.6 Fix `CMakeLists.txt` ✅ DONE

`CMakeLists.txt` is now clean: no C++ targets, no spurious `find_package` calls, `cmake_minimum_required(VERSION 3.8)`, only installs `config/`, `launch/`, `maps/`, and `scripts/`.

### 3.7 Config files to audit

| File | Notes |
|---|---|
| `config/bt1.xml` | Wait node with negative duration was fixed; SPDX header added Feb 2026; review remaining content |
| ~~`config/nn/can_yolov5.json`, `can_yolov8.json`~~ | ✅ Removed — `nn/` directory deleted; files belonged to `yolo_oakd_test` (gone) |
| ~~`config/oakd_camera.yaml`~~ | ✅ Removed — moved to `sigyn_oakd_detection` workspace |
| ~~`config/pcl.yaml`~~ | ✅ Removed — `pointcloud.launch.py` was deleted |
| `config/gazebo.yaml` | Needed for sim; SPDX header added Feb 2026 |
| `config/gz_bridge.yaml` | Actively used for Gazebo simulation; SPDX header added Feb 2026 |

---

## Section 4 — `sigyn_behavior_trees` extraction (IN PROGRESS)

New repo: `~/sigyn_behavior_trees_ws/src/sigyn_behavior_trees` (initial commit made).

See `REFACTORING_PLAN.md` in that repo for the full issue list from code review. Key items not yet done:

- [ ] Fix action servers never calling `goal_handle->succeed()` — execution hangs
- [ ] Fix `#include "SaySomethingActionServer.cpp"` in `bt_test1_main.cpp` — ODR violation
- [ ] Fix double `return 0;` and wrong node name in `bt_test2_main.cpp`
- [ ] Fix wrong namespace comment in `SS.hpp`
- [ ] Replace raw `new BT::NodeConfiguration` with `std::make_unique`
- [ ] Fix package.xml: `nav2_behaviors` → `nav2_behavior_tree`; add missing `tf2_ros`; remove nonexistent `behavior_plugin.xml` export
- [ ] Fix CMakeLists: action server libs need `SHARED`; remove unconditional `-g`; old-style `include_directories()`
- [ ] Remove `launch/xxxbt1.launch.py`, dead comments in all `.cpp` files
- [ ] Remove `notes.md` (single stale snippet)
- [ ] Apply consistent Google style + clang-format
- [ ] Add SPDX copyright headers to all files
- [ ] Push to `wimblerobotics/sigyn_behavior_trees` on GitHub
- [ ] Update `Sigyn2/packages.yaml` to reference new repo (currently points to Sigyn monorepo)

---

## ✅ Section 5 — Extract `sigyn_to_sensor_v2` → `sigyn_to_teensy` — COMPLETE

**Actual repo name created:** `wimblerobotics/sigyn_to_teensy`
**Local path:** `~/sigyn_to_teensy_ws/src/sigyn_to_teensy/`
**Completed:** February 2026

### What was done

- [x] Renamed package: `sigyn_to_sensor_v2` → `sigyn_to_teensy`
- [x] Renamed all `TeensyV2` references → `sigyn_teensy_boards`
- [x] Removed unused `rclcpp_lifecycle`, `rclcpp_components` dependencies
- [x] Renamed config: `teensy_v2_config.yaml` → `sigyn_to_teensy.yaml`
- [x] Renamed launch: `teensy_bridge.launch.py` → `sigyn_to_teensy.launch.py`
- [x] Renamed include directory: `sigyn_to_sensor_v2/` → `sigyn_to_teensy/`
- [x] SPDX Apache-2.0 headers on all source files; copyright 2025 → 2026
- [x] `AI_CONTEXT.md`, `README.md` rewritten; `docs/` kept
- [x] Updated `base/launch/sigyn.launch.py` package/launch name references
- [x] Initial commit `491abf5` pushed to `wimblerobotics/sigyn_to_teensy`

Remaining follow-up:
- [ ] Remove `sigyn_to_sensor_v2/` from Sigyn monorepo; update `Sigyn2/packages.yaml`
- [ ] Add unit tests for `message_parser.cpp` (ament_cmake_gtest)
- [ ] Rework with dependency injection; rearchitect to align with `sigyn_teensy_boards` changes

---

## ✅ Section 6 — Extract `TeensyV2` → `sigyn_teensy_boards` — COMPLETE

**Actual repo name created:** `wimblerobotics/sigyn_teensy_boards` (not `sigyn_teensy_firmware` as originally planned)
**Local path:** `~/sigyn_ws/src/sigyn_teensy_boards/` (deployed by vcstool into sigyn_ws/src/ alongside ROS packages, `tool_repo: true` so colcon ignores it)
**Keep in sync with:** `sigyn_to_teensy` (tightly coupled — shared message protocol; now at `wimblerobotics/sigyn_to_teensy`)
**Completed:** February 2026

### 6.1 Pre-extraction cleanup

- [ ] Add SPDX copyright headers to all source files (tracked in `sigyn_teensy_boards` TODO.md)
- [x] `src/roboclaw_test.cpp` — kept in new repo as a dev/diagnostic tool (env: `roboclaw_test`)
- [x] `src/elevator_board.cpp` — confirmed as Board 3 main loop
- [ ] Review `modules/` and `common/` for any deprecated or dead code (tracked in TODO.md)
- [ ] Address **CRITICAL** TODO items (tracked in `sigyn_teensy_boards/TODO.md`):
  - Enable Safety on Board 3 (Gripper): set `BOARD_HAS_SAFETY 1`
  - Fix inter-board communication: implement `fault_handler` in `board1_main.cpp`
  - Sensor timeout safety: ensure sensors trigger `isUnsafe` on stall
- [x] README updated to reflect `sigyn_teensy_boards` name and correct deployed path
- [x] `AI_CONTEXT.md` and `TODO.md` added to new repo (architectural review, GPIO e-stop, message redesign)
- [x] `platformio.ini` is clean; all `boards/` config is up to date

### 6.2 Extraction steps

- [x] Clone `wimblerobotics/sigyn_teensy_boards` and populate with firmware source
- [x] Copy all firmware source (exclude build artifacts and ROS scaffolding)
- [x] Create clean `.vscode/settings.json` (no stale ROS Python paths)
- [x] Add `.gitignore` entries for `log/`, `install/`, `compile_commands.json`, `launch.json`
- [x] Initial commit pushed to `wimblerobotics/sigyn_teensy_boards` on GitHub
- [x] Add `sigyn_teensy_boards` to `Sigyn2/config/packages.yaml` (`sigyn_hardware` group)
- [x] Update `bashrc` aliases to point to `~/sigyn_ws/src/sigyn_teensy_boards/`
- [x] Remove `TeensyV2/` from Sigyn monorepo (Feb 2026)

---

## ✅ Section 7 — Extract `yolo_oakd_test` → `sigyn_oakd_detection` — COMPLETE

**Actual final repo name:** `wimblerobotics/sigyn_oakd_detection` (not `sigyn_oakd_detector` as originally planned)
**Workspace:** `~/sigyn_oakd_detection_ws/src/sigyn_oakd_detection/`
**Completed:** February 2026

### What was done

- [x] Renamed package: `yolo_oakd_test` → `sigyn_oakd_detection`
- [x] Fixed `package.xml` maintainer and license
- [x] Resolved model file discrepancy (correct blob shipped with the new package)
- [x] `OakdDetection.msg` (renamed from `CanDetection.msg`) moved to `sigyn_interfaces` (v0.9.4)
- [x] Applied Google style and SPDX copyright headers
- [x] Launch file renamed to `oakd_detector.launch.py`
- [x] Updated `base/launch/sub_launch/oakd_yolo26_detector.launch.py` and `can_do_challenge` launch files to delegate to `sigyn_oakd_detection`
- [x] Removed `yolo_oakd_test/` from Sigyn monorepo

Remaining follow-up (not blocking):
- [ ] Move `base/config/nn/can_yolov5.json` / `can_yolov8.json` to the new workspace or remove
- [ ] Move `base/config/oakd_camera.yaml` to the new workspace
- [ ] Update `Sigyn2/packages.yaml` to add `sigyn_vision` group with `wimblerobotics/sigyn_oakd_detection`

---

## Section 8 — `can_do_challenge` Cleanup ✅ EXTRACTED

`can_do_challenge` has been extracted to `wimblerobotics/can_do_challenge`. The items below apply to that repo.

### 8.1 Dependency issues

- [ ] `<depend>behaviortree_cpp_v3</depend>` — The rest of the project uses `behaviortree_cpp` (v4 API). Migrate `can_do_challenge` from v3 to v4. Key API differences: `BT::NodeStatus`, `BT::InputPort`, action node base class names. This is a significant refactor.
- [ ] `<depend>gripper_camera_detector</depend>` — This package does not exist in `sigyn_ws/src`. Determine whether this should be `pi_can_detector` (on sigynVision) or something else. Either add it or remove the dependency.
- [ ] Review whether `depthai_ros_msgs` is still needed now that OAK-D detection is via `sigyn_oakd_detection` (separate workspace)

### 8.2 Dead code / files to move to `~/other_repository`

| File | Reason |
|---|---|
| `bt_xml/old.xml` | Explicitly named as old |
| `bt_xml/main.orig.xml` | Backup of main.xml |
| `bt_xml/oakd_detection_test1.xml` | Test tree for development |
| `launch/__pycache__/` | Build artifact, not source |
| `resources/calibration_imgs/` | ~30 JPEG images; use git-lfs or external storage |

### 8.3 Launch file consolidation

Currently 9 step-by-step launch files (`step1_real_launch.py` ... `step5_elevator_height_launch.py`) plus `can_do_challenge_launch.py`, `can_do_sim_launch.py`, `manual_sim_launch.py`, `view_world.launch.py`. 

- [ ] Determine which step-based launch files are still needed vs. just development scaffolding; move unneeded ones to `~/other_repository`
- [ ] The `can_do_challenge_launch.py` should be the canonical "run the challenge" launch

### 8.4 Code cleanup

- [ ] Add SPDX copyright headers to all files
- [ ] Apply Google style + clang-format to C++ files in `include/`
- [ ] Apply PEP 8 to Python files in `can_do_challenge/`
- [ ] `include/can_do_challenge/bt_nodes.hpp` vs `bt_nodes_real.hpp` — audit which is used; consolidate if possible
- [ ] `config/can_locations.json` — document what coordinate frame and what fields this file contains

---

## ✅ Section 9 — `bluetooth_joystick` → `sigyn_bluetooth_joystick` — COMPLETE

**Actual repo name created:** `wimblerobotics/sigyn_bluetooth_joystick`
**Local path:** `~/sigyn_bluetooth_joystick_ws/src/sigyn_bluetooth_joystick/`
**Completed:** February 2026

- [x] Renamed package and executable to `sigyn_bluetooth_joystick`
- [x] Fixed include path, removed unused globals, fixed misleading button comment
- [x] Added missing `std_msgs` dependency
- [x] SPDX Apache-2.0 headers; README rewritten (fixed MIT→Apache-2.0 error)
- [x] `AI_CONTEXT.md` created; initial commit pushed

Remaining: Remove `bluetooth_joystick/` from Sigyn monorepo

---

## Section 10 — `rviz` Package ✅ DONE

- ✅ SPDX headers added to `CMakeLists.txt` and `package.xml`
- ✅ `CMakeLists.txt` cleaned: `cmake_minimum_required(3.8)`, dead C99/C++14 boilerplate removed
- ✅ `package.xml` description fixed
- ✅ `config/config.rviz` audited Feb 2026: no stale topic references. OAK-D topics (`/oakd_top/oak/stereo/image_raw`, `/local_costmap/oakd_top_layer`) will show "no data" when `sigyn_oakd_detection` workspace is not active — expected behaviour.
- Merged into `sigyn_bringup` or kept standalone: **keeping standalone** (renamed via `get_package_share_directory("rviz")` in all launch files — clean separation)

---

## Section 11 — Sigyn2 Deployment Repo Updates

After each extraction above, `Sigyn2/config/packages.yaml` and `Sigyn2/config/robots.yaml` need updates:

| Action | `packages.yaml` change |
|---|---|
| `sigyn_behavior_trees` extracted | Add new group `sigyn_behavior_trees` with `wimblerobotics/sigyn_behavior_trees` |
| `base` renamed to `sigyn_bringup` | Update `sigyn_navigation` group — Sigyn monorepo name stays but package name changes |
| ✅ `sigyn_to_sensor_v2` extracted | `sigyn_hardware` group: `wimblerobotics/sigyn_to_teensy` (done — monorepo removal pending) |
| `TeensyV2` extracted | Add `sigyn_hardware` group: `wimblerobotics/sigyn_teensy_boards` (done — pending TeensyV2/ removal from monorepo) |
| ✅ `yolo_oakd_test` extracted | Add `sigyn_vision` group: `wimblerobotics/sigyn_oakd_detection` (done — Sigyn2 packages.yaml update pending) |

---

## Section 11b — `pi_gripper` Cleanup

`pi_gripper` provides PCA9685 PWM servo control for the gripper assembly on sigynVision (Raspberry Pi 5). It is deployed independently and is not part of `sigyn_ws/src` on sigyn7900a.

- [ ] Add SPDX copyright headers: `# SPDX-License-Identifier: Apache-2.0` / `# Copyright 2026 Wimble Robotics`
- [ ] Fix `package.xml` maintainer if it still has placeholder values
- [ ] Apply PEP 8 / `ruff` formatting to all Python source files
- [ ] Add a `README.md` covering: hardware wiring (PCA9685 I2C address, servo channel assignments), udev rules needed, deployment instructions
- [ ] Verify `Sigyn2/packages.yaml` `pi_gripper` group entry has correct git URL and branch
- [ ] Ensure the `pi_gripper` action interface (if any) is defined in `sigyn_interfaces`, not privately inside this repo
- [ ] Confirm the topic/service names match what `sigyn_to_teensy` and `can_do_challenge` expect
- [ ] Add basic unit tests with `pytest` for any non-ROS servo math/limit-checking logic

---

## Section 12 — Cross-Cutting: Style, Naming, and Copyright

### 12.1 SPDX copyright headers — partial

Add to **every** `.cpp`, `.hpp`, `.h`, `.py`, `.lua`, `CMakeLists.txt`, `package.xml`, `.launch.py` file.

Done as of 2026-02-24:
- ✅ All `sigyn_bringup/launch/*.py` and `sub_launch/*.py` files
- ✅ `sigyn_bringup/package.xml`, `sigyn_bringup/CMakeLists.txt`
- ✅ `sigyn_bringup/config/ekf.yaml`, `mapper_params_online_async.yaml`, `mapper_params_lifelong.yaml`
- ✅ `sigyn_bringup/config/gazebo.yaml`, `gz_bridge.yaml`, `laser_filters_angular.yaml`, `bt1.xml`
- ✅ `sigyn_bringup/scripts/battery_overlay_publisher.py`
- ✅ `rviz/CMakeLists.txt`, `rviz/package.xml`

Still missing:
- `sigyn_bringup/config/gazebo.yaml`, `gz_bridge.yaml` sub-comments (non-critical)
- All files in extracted repos (tracked in their own repos)

### 12.2 Google C++ Style

Key naming conventions that differ from what's currently in the codebase:

| Element | Google Style | Current Issues |
|---|---|---|
| Files | `snake_case.cpp` / `snake_case.hpp` | `SaySomethingActionServer.cpp`, `MoveAShortDistanceAheadActionServer.cpp`, `SS.hpp` |
| Classes | `PascalCase` | Mostly OK |
| Methods | `snake_case()` | Nav2-derived code uses mixed |
| Member variables | `trailing_underscore_` or just consistent | Some use `node_` (OK), some shadow base class |
| Constants | `kConstantName` | Not used anywhere yet |
| Macros | `ALL_CAPS` | OK |

Apply `.clang-format` with Google base style to all C++ files. Create a top-level `.clang-format` file in the monorepo root.

### 12.3 ROS 2 / Nav2 naming conventions

ROS 2 and nav2 have their own idioms that sometimes differ from strict Google style:
- Node names: `snake_case` (Google-compatible)
- Topic names: `/namespace/snake_case` (already followed)
- Parameter names: `snake_case` (already followed)
- Action/service definitions: `PascalCase.action` / `PascalCase.srv` (already followed)
- Package names: `snake_case` (already followed, mostly)

Research: Nav2 uses `BtActionNode` base class from `nav2_behavior_tree` package, not the raw `BT::ActionNodeBase`. The `sigyn_behavior_trees` package correctly inherits from `nav2_behavior_tree::BtActionNode`. Verify `can_do_challenge` also migrates to this base class when upgrading from v3 → v4.

### 12.4 Python style

Apply `ruff` or `flake8`/`black` formatting to all Python files:
- `battery_overlay_publisher.py`
- All launch files
- `can_do_challenge/can_do_challenge/*.py`
- `scripts/*.py`
- (Note: `yolo_oakd_test` has been removed; the successor `sigyn_oakd_detection` lives in its own workspace)

### 12.5 ✅ Add a top-level `.clang-format` — DONE

`Sigyn/.clang-format` exists with Google base style, `ColumnLimit: 100`, `PointerAlignment: Left`. Updated Feb 2026 to add SPDX header and `DerivePointerAlignment: false`.

---

## Section 13 — Testing

### 13.1 `sigyn_to_teensy` (formerly `sigyn_to_sensor_v2`)

Tests exist in `sigyn_teensy_boards/test/` (PlatformIO side). On the ROS 2 side:
- [ ] Add `ament_cmake_gtest` tests for `message_parser.cpp` — unit test all message format strings
- [ ] Add a mock serial port test to verify the bridge correctly publishes sensor data
- [ ] Wire up `BUILD_TESTING` in `CMakeLists.txt` — currently the block is present in `package.xml` test_depend but the CMake side may be empty

### 13.2 `sigyn_behavior_trees`

- [ ] Add `ament_cmake_gtest` tests for each BT node class
- [ ] SaySomething: mock the action server, verify `on_tick()` sets `goal_.message` correctly
- [ ] MoveAShortDistanceAhead: verify `initialize()` is called, verify negative distance clamping to 0
- [ ] Use `behaviortree_cpp` built-in test tools (tree tick simulation without a ROS spinner)

### 13.3 `can_do_challenge`

- [ ] Add tests for BT condition nodes (`IsCanDetected`, etc.) with mock topic publishers
- [ ] Add tests for the Python detectors (`simple_can_detector.py`, `spatial_detection_annotator.py`) using `pytest` and mock camera data

### 13.4 `sigyn_bringup`

- [ ] Add `ament_xmllint` validation for BT XML files
- [ ] Tests not realistic for launch files, but add parameter validation smoke tests
- [ ] Validate that `precheck.launch.py` correctly emits `Shutdown` when devices are missing

---

## Section 14 — Additional Suggestions

### 14.1 Create a Sigyn2 setup verification script

`Sigyn2` deploys config but there's no quick way to verify a freshly deployed machine has everything correct. Create `scripts/verify_deployment.py` that checks:
- All expected device nodes exist (`/dev/teensy_*`, `/dev/lidar_*`)
- ROS domain ID is set correctly
- All expected packages are present (via `ros2 pkg list`)

### 14.2 Navigation — consider splitting `navigate_to_pose_w_replanning_and_recovery.xml` from nav2 defaults vs. custom Sigyn BT

Currently `sigyn.launch.py` defaults to the nav2 stock BT XML for `navigate_to_pose`. The `sigyn_bringup/config/bt1.xml` and `sigyn_behavior_trees/config/patrol.xml` are custom. These should eventually be brought together under a custom nav BT that integrates Sigyn's recovery behaviors.

### 14.3 `precheck.launch.py` — integrate with `sigyn.launch.py`

Currently `precheck` is a separate launch. Consider making it the canonical entry point that then includes `sigyn.launch.py`, so the robot can never accidentally be started without device checks.

### 14.4 Consolidate `rviz` config

`rviz/config/config.rviz` is the monorepo RViz config. `yolo_oakd_test/config/can_detection.rviz` no longer exists (package deleted). The `sigyn_oakd_detection` workspace may ship its own RViz config. Ensure all topic references are current and create one canonical general-purpose config and one OAK-D detection visualization config.

### 14.5 ✅ Remove `CMakePresets.json` from `sigyn_bringup/` — DONE

`sigyn_bringup/CMakePresets.json` has been removed from git tracking (`git rm --cached`) and `CMakePresets.json` added to `.gitignore`. The file remains on disk for those who want local IDE integration.

### 14.6 `navigation_launch.py` — confirm it is the right nav2 bringup entry point

`nav2_bringup.launch.py` includes `navigation_launch.py`. The `navigation_launch.py` remaps Nav2's output to `/cmd_vel_nav` (which feeds the twist multiplexer). Confirm this remapping survives across nav2 package updates (it's a custom file that may need to be re-diffed against the nav2 stock version each release).

---

## Execution Order (Updated 2026-02-24)

1. ✅ **Git/Repo hygiene** (Section 1) — `sigyn_bringup/out/` confirmed never committed; `.gitignore` fixed
2. ✅ **Rename `base` → `sigyn_bringup`** (Section 2) — DONE Feb 2026
3. ✅ **`base` launch file purge** (Section 3.1) — dead launch files removed
4. ✅ **`base` package.xml + CMakeLists.txt** (Section 3.5, 3.6) — cleaned up
5. ✅ **Mapping launches created and validated** (Section 3.2) — Cartographer and SLAM Toolbox both working Feb 2026; `map_saver_server` added Feb 2026
6. ✅ **`sigyn_bringup` remaining cleanup** (Section 3.3, 3.4, 3.7) — nav config renamed, maps audited, dead config files removed, SPDX headers complete
7. **Extract `sigyn_behavior_trees`** (Section 4) — in progress
8. **Style sweep: SPDX + clang-format** (Section 12) — done per-package as each is touched
9. ✅ **Extract `yolo_oakd_test`** (Section 7) — DONE (`wimblerobotics/sigyn_oakd_detection`)
10. ✅ **Verify + remove `TeensyV2/`** (Section 6) — DONE (`wimblerobotics/sigyn_teensy_boards`; Feb 2026)
11. ✅ **Extract `can_do_challenge`** — DONE (`wimblerobotics/can_do_challenge`; Feb 2026)
12. ✅ **Extract `bluetooth_joystick`** (Section 9) — DONE (`wimblerobotics/sigyn_bluetooth_joystick`; Feb 2026)
13. ✅ **Extract `sigyn_to_sensor_v2`** (Section 5) — DONE (`wimblerobotics/sigyn_to_teensy`; Feb 2026)
14. ✅ **Remove extracted packages from monorepo** — `bluetooth_joystick/`, `sigyn_to_sensor_v2/`, `TeensyV2/`, `yolo_oakd_test/`, `can_do_challenge/` all removed
15. **`Sigyn2` updates** (Section 11) — add new repos to packages.yaml
16. **Testing** (Section 13) — add tests after each package is in a clean state

## Next Priorities (2026-02-24)

In rough order of urgency:

1. **Finish `sigyn_behavior_trees` extraction** (Section 4) — fix action server bugs, push to GitHub, update `Sigyn2/packages.yaml`.

2. **Restore `~/other_repository/perimeter_roamer_v3`** — bring back to operational state on the real robot. Mapping launches are now working (Cartographer + SLAM Toolbox both validated Feb 2026).

3. **Integrate `can_do_challenge`** — extracted to `wimblerobotics/can_do_challenge`; needs integration testing on both simulation and the real robot, including the BT v3→v4 migration (Section 8.1).

4. **`sigyn_teensy_boards` rework** — analyze, rearchitect as necessary, shorten message lengths, implement board-to-board e-stop via GPIO.

5. **`sigyn_to_teensy` rework** — correspond to `sigyn_teensy_boards` changes, add unit tests for `message_parser.cpp`, rework with dependency injection.

6. **Audit all repos in `Sigyn2`** — check architecture, clean up, add tests.

---

## Open Questions

1. ✅ Should `bluetooth_joystick` stay in the monorepo or get its own repo? **Decision: extract to own repo (priority 9 above)**
2. ✅ Should `rviz` merge into `sigyn_bringup` or stay standalone? **Decision: keep standalone** — clean separation via `get_package_share_directory("rviz")`
3. ✅ Should `can_do_challenge` eventually move to its own repo? **Decision: yes (priority 8 above)**
4. ✅ The `CanDetection.msg` question is resolved: `yolo_oakd_test` deleted; `OakdDetection.msg` (renamed) now lives in `sigyn_interfaces` v0.9.4.
5. ✅ `base/config/pcl.yaml` — deleted; the pointcloud launch was already gone.
6. ✅ `config/gazebo.yaml` is for the `ros_gz_bridge` publication rate parameter (not the bridge config itself); `config/gz_bridge.yaml` is the actual topic bridge mapping. Both are needed for simulation.
