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
| ~~`bluetooth_joystick`~~ | ✅ Extracted + removed | `wimblerobotics/sigyn_bluetooth_joystick` |
| `can_do_challenge` | Major cleanup needed | Extracted to `wimblerobotics/can_do_challenge` |
| `rviz` | ✅ Clean | Keep standalone |
| `sigyn_behavior_trees` | Extraction in progress | New repo: `wimblerobotics/sigyn_behavior_trees` |
| ~~`sigyn_to_sensor_v2`~~ | ✅ Extracted + removed | `wimblerobotics/sigyn_to_teensy` |
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

## ✅ Section 1 — Git / Repo Hygiene — DONE

`.gitignore` covers `__pycache__`, calibration images, `CMakePresets.json`. No build artifacts were ever committed. `sigyn_bringup/CMakePresets.json` untracked Feb 2026.

---

## ✅ Section 2 — Rename `base` → `sigyn_bringup` — DONE (February 2026)

All package refs, launch files, docs, Sigyn2 deployment config updated. Single-quoted `'base'` in `nav2_bringup.launch.py` and `precheck.launch.py` fixed. `__pycache__` clearing added to `setup_robot.py`.

---

## Section 3 — `base`/`sigyn_bringup` Package Cleanup

✅ **3.1–3.6 all complete** (Feb 2026): dead launch files removed; mapping launches validated with `map_saver_server`; `navigation_sim.yaml` → `navigation.yaml`; maps audited (one canonical map: `my_map.yaml` → `my_map2.pgm`); `package.xml` and `CMakeLists.txt` cleaned; SPDX headers added to all `sigyn_bringup` files.

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

**Repo:** `wimblerobotics/sigyn_to_teensy` | **Completed:** February 2026 | Removed from monorepo Feb 2026.

Remaining:
- [ ] Update `Sigyn2/packages.yaml` to add `wimblerobotics/sigyn_to_teensy` to `sigyn_hardware` group
- [ ] Add unit tests for `message_parser.cpp` (ament_cmake_gtest)
- [ ] Rework with dependency injection; rearchitect to align with `sigyn_teensy_boards` changes

---

## ✅ Section 6 — Extract `TeensyV2` → `sigyn_teensy_boards` — COMPLETE

**Repo:** `wimblerobotics/sigyn_teensy_boards` | **Local:** `~/sigyn_ws/src/sigyn_teensy_boards/` (`tool_repo: true`, not colcon-built) | **Completed:** February 2026.

Remaining (tracked in `sigyn_teensy_boards/TODO.md`):
- [ ] Add SPDX copyright headers to all source files
- [ ] Review `modules/` and `common/` for deprecated or dead code
- [ ] **CRITICAL**: Enable Safety on Board 3 (Gripper): set `BOARD_HAS_SAFETY 1`
- [ ] **CRITICAL**: Fix inter-board communication: implement `fault_handler` in `board1_main.cpp`
- [ ] **CRITICAL**: Sensor timeout safety: ensure sensors trigger `isUnsafe` on stall

---

## ✅ Section 7 — Extract `yolo_oakd_test` → `sigyn_oakd_detection` — COMPLETE

**Repo:** `wimblerobotics/sigyn_oakd_detection` | **Completed:** February 2026. `OakdDetection.msg` in `sigyn_interfaces` v0.9.4. `nn/` config dir and `oakd_camera.yaml` removed from `sigyn_bringup`.

Remaining:
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

**Repo:** `wimblerobotics/sigyn_bluetooth_joystick` | **Completed:** February 2026. Extracted, cleaned, pushed, and removed from monorepo.

---

## ✅ Section 10 — `rviz` Package — DONE

SPDX headers added, `CMakeLists.txt` cleaned, `config.rviz` audited (no stale topics; OAK-D topics show "no data" when `sigyn_oakd_detection` workspace inactive — expected). **Keeping standalone** via `get_package_share_directory("rviz")`.

---

## Section 11 — Sigyn2 Deployment Repo Updates

After each extraction above, `Sigyn2/config/packages.yaml` and `Sigyn2/config/robots.yaml` need updates:

| Action | `packages.yaml` change |
|---|---|
| `sigyn_behavior_trees` extracted | Add `sigyn_behavior_trees` group: `wimblerobotics/sigyn_behavior_trees` — **pending** |
| ✅ `base` renamed to `sigyn_bringup` | `sigyn_navigation` group updated |
| ✅ `sigyn_to_teensy` extracted | `sigyn_hardware` group: add `wimblerobotics/sigyn_to_teensy` — **pending** |
| ✅ `sigyn_teensy_boards` extracted | `sigyn_hardware` group: `wimblerobotics/sigyn_teensy_boards` already added |
| ✅ `sigyn_oakd_detection` extracted | `sigyn_vision` group: add `wimblerobotics/sigyn_oakd_detection` — **pending** |

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

### 14.5 (was: `CMakePresets.json`) ✅ Done — untracked Feb 2026; `CMakePresets.json` added to `.gitignore`.

### 14.6 `navigation_launch.py` — confirm it is the right nav2 bringup entry point

`nav2_bringup.launch.py` includes `navigation_launch.py`. The `navigation_launch.py` remaps Nav2's output to `/cmd_vel_nav` (which feeds the twist multiplexer). Confirm this remapping survives across nav2 package updates (it's a custom file that may need to be re-diffed against the nav2 stock version each release).

---

## Remaining Work (as of 2026-02-24)

Sections 1, 2, 3, 5 (extraction), 6 (extraction), 7 (extraction), 9, 10 are complete. All extracted packages removed from monorepo. Remaining in priority order:

1. **Finish `sigyn_behavior_trees` extraction** (Section 4) — fix action server bugs, push to GitHub, update `Sigyn2/packages.yaml`
2. **`Sigyn2/packages.yaml`** (Section 11) — add `sigyn_to_teensy`, `sigyn_oakd_detection` entries; add `sigyn_behavior_trees` group
3. **Restore `perimeter_roamer_v3`** — bring back to operational state; mapping launches are now working
4. **Integrate `can_do_challenge`** — integration testing on sim + real robot; BT v3→v4 migration (Section 8.1)
5. **`sigyn_teensy_boards` rework** (Section 6 remaining) — safety fixes, shorten messages, GPIO e-stop
6. **`sigyn_to_teensy` rework** (Section 5 remaining) — unit tests, dependency injection, align with teensy_boards
7. **Testing** (Section 13) — add tests after each package is in a clean state
8. **Style sweep** (Section 12) — SPDX + clang-format done per-package as each is touched; `can_do_challenge` next


