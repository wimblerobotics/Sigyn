# Sigyn Monorepo Refactoring Plan
**Created:** 2026-02-20  
**Status:** In progress — no changes made yet; this document is the roadmap.

---

## Vision

Break the Sigyn monorepo into a set of clean, independently deployable repositories that serve as a reference example of how to build a ROS 2 robot. Each repo should be usable standalone and demonstrate best practices for naming, style, testing, and documentation.

---

## Current Repo Inventory

### In `wimblerobotics/Sigyn` (this monorepo, branch `sigyn2`)

| Package | Status | Destination |
|---|---|---|
| `base` | Keep, but rename and purge | Rename → `sigyn_bringup` (or `sigyn_launch`), stays here |
| `bluetooth_joystick` | Keep or extract | Stays here for now; extract later if desired |
| `can_do_challenge` | Major cleanup needed | Stays here as application layer |
| `rviz` | Minimal, keep | Merge into `sigyn_bringup` or keep standalone |
| `sigyn_behavior_trees` | Extraction in progress | New repo: `wimblerobotics/sigyn_behavior_trees` |
| `sigyn_to_sensor_v2` | Extract + rename | New repo: `wimblerobotics/sigyn_teensy_bridge` |
| `TeensyV2` | Extract + rename | New repo: `wimblerobotics/sigyn_teensy_firmware` |
| `yolo_oakd_test` | Extract + rename | New repo: `wimblerobotics/sigyn_oakd_detector` |

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

### 1.1 Add missing `.gitignore` entries

The current `.gitignore` covers `.pyc` but not:
- `**/__pycache__/` directories (present in `base/launch/` and `base/launch/sub_launch/` and `can_do_challenge/launch/`)
- `base/out/` — entire CMake IDE build tree committed to git (hundreds of generated files)
- `can_do_challenge/launch/__pycache__/` (one file already committed: `step3_visual_acquire_launch.cpython-312.pyc`)
- `base/launch/__pycache__/nav2_bringup.launch.cpython-312.pyc` committed to git

**Action:** Add `**/__pycache__/` and `base/out/` to `.gitignore`, then `git rm -r --cached` those paths.

### 1.2 Remove committed build artifacts

- `base/out/` — full CMake IDE configure tree with binaries; should never be committed.

### 1.3 Large binary assets in `can_do_challenge/resources/`

Calibration images (JPEGs) are committed to the main git history. These should either:
- Be removed and documented with instructions to re-acquire, or
- Moved to git-lfs.

Affected path: `can_do_challenge/resources/calibration_imgs/` (~30 images).

---

## Section 2 — Rename `base` → `sigyn_bringup`

`base` is dangerously generic and collides with common CMake/ROS concepts. The package's purpose is the top-level bringup of the Sigyn robot — rename it everywhere.

**Files to update:**
- `base/CMakeLists.txt` — `project(base)` → `project(sigyn_bringup)`
- `base/package.xml` — `<name>base</name>` → `<name>sigyn_bringup</name>`
- All launch files that call `get_package_share_directory("base")` (appears >10 times in `sigyn.launch.py`, and in several sub-launch files)
- `~/.bash_aliases` — `nav` alias (currently `ros2 launch base sigyn.launch.py`)
- `Sigyn2/config/robots.yaml` — package group reference
- `Sigyn2/config/packages.yaml` (if `base` is listed there; it is implicit via `sigyn_navigation`)

**Note:** The directory name `base/` can remain as-is for now (the ROS package name and directory name can differ), or rename the directory to `sigyn_bringup/` in a single `git mv` operation.

---

## Section 3 — `base`/`sigyn_bringup` Package Cleanup

### 3.1 Dead launch files to remove

| File | Reason |
|---|---|
| `launch/foo.launch.py` | Placeholder code with `your_package`/`object_detection_node` references; never ran |
| `launch/sub_launch/base.launch.py` | Never included by `sigyn.launch.py`; logic now inlined; references `wr_twist_multiplexer.launch.py` which may not exist |
| `launch/sub_launch/common.py` | Not imported by anything; contains old `snowberry4v31.yaml` map reference and wrong description package name (`description` not `sigyn_description`) |
| `launch/sub_launch/oakd_stereo.launch.py` | `oakd_camera` call in `sigyn.launch.py` is fully commented out |
| `launch/sub_launch/pointcloud.launch.py` | Not referenced in `sigyn.launch.py`; re-composite of `sigyn_camera_launch.py` |
| `launch/sub_launch/sigyn_camera_launch.py` | Only used by the two dead files above; dead code |

**To keep / actively used:**
- `launch/sigyn.launch.py` — main launch
- `launch/nav2_bringup.launch.py` — called by `sigyn.launch.py`
- `launch/navigation_launch.py` — called by `nav2_bringup.launch.py`
- `launch/precheck.launch.py` — device node precheck; good, keep
- `launch/sub_launch/lidar.launch.py` — called by `sigyn.launch.py`
- `launch/sub_launch/oakd_yolo26_detector.launch.py` — called by `sigyn.launch.py`
- `launch/sub_launch/oakd_compressed_republisher.launch.py` — called by `sigyn.launch.py`

### 3.2 Create a Cartographer mapping launch file

`base/config/cartographer.lua` exists and is well-configured, but **there is no launch file to actually run Cartographer**. This is needed to regenerate the map.

**Create:** `launch/cartographer_mapping.launch.py` that:
- Launches `cartographer_ros` with `cartographer.lua`
- Brings up the LIDAR (via `lidar.launch.py` sub-launch)
- Brings up `robot_state_publisher` + `joint_state_publisher`
- Brings up EKF for odometry
- Optionally brings up RViz with a cartographer-appropriate config

Also confirm `mapper_params_lifelong.yaml` vs `mapper_params_online_async.yaml` — which is used for mapping and which for localization-only?

### 3.3 Separate real-robot and simulation navigation configs

Currently `navigation_sim.yaml` is used for **both** real robot and simulation (the filename is misleading). 

- Rename `navigation_sim.yaml` → `navigation.yaml`
- If there are genuinely sim-specific settings, split into `navigation_real.yaml` and `navigation_sim.yaml`, and pick the right one in `sigyn.launch.py` based on `use_sim_time`.

### 3.4 Clarify which map is current

Maps directory contains:
- `20241210l.pgm`/`20241210l.yaml` — dated map from Dec 2024
- `map2.pgm`/`map2.yaml` + `map2s.data`/`map2s.posegraph` — another map
- `my_map.yaml` (no corresponding `.pgm` found with correct name; `my_map2.pgm` exists)
- The launch file references `my_map.yaml` for both real and sim

**Action:** Audit maps. Keep one canonical current map. Move old maps to `~/other_repository` or a `maps/archive/` subdirectory. Document which map is in use.

### 3.5 Fix `package.xml` dependency errors

- `<depend>ldlidar</depend>` — old package name; should be `<depend>wr_ldlidar</depend>`
- `<depend>python3-cython</depend>` — not needed by anything in this package
- `<member_of_group>rosidl_interface_packages</member_of_group>` — this package defines no interfaces; remove
- `<buildtool_depend>rosidl_default_generators</buildtool_depend>` + `<exec_depend>rosidl_default_runtime</exec_depend>` — same issue, not needed

### 3.6 Fix `CMakeLists.txt`

- `add_compile_options(-g)` — this package has **no C++ targets**; the line does nothing except bloat future accidental additions. Remove.
- `find_package(geometry_msgs REQUIRED)` / `nav_msgs` / `rclcpp` / `tf2_ros` — none are used in CMakeLists.txt (no C++ targets); remove all.
- Minimum CMake version: `3.5` → `3.8` (to match other packages in the monorepo).
- Add `set(CMAKE_CXX_STANDARD_REQUIRED ON)` if C++ ever gets added.

### 3.7 Config files to audit

| File | Notes |
|---|---|
| `config/bt1.xml` | Wait node with negative duration was fixed; review remaining content |
| `config/nn/can_yolov5.json`, `can_yolov8.json` | Used by `yolo_oakd_test`; should move to that package when extracted |
| `config/oakd_camera.yaml` | OAK-D camera config; should move to `sigyn_oakd_detector` when extracted |
| `config/pcl.yaml` | PointCloud filter config; appears unused (pointcloud.launch.py is dead) |
| `config/gazebo.yaml` | Review if still relevant for sim |
| `config/gz_bridge.yaml` | Actively used for Gazebo simulation |

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

## Section 5 — Extract `sigyn_to_sensor_v2` → `sigyn_teensy_bridge`

**New repo name:** `wimblerobotics/sigyn_teensy_bridge`

### 5.1 Pre-extraction cleanup inside the current directory

- [ ] Add SPDX copyright headers to all source files
- [ ] Rename package in `CMakeLists.txt` / `package.xml`: `sigyn_to_sensor_v2` → `sigyn_teensy_bridge`
- [ ] Apply Google style + clang-format to all C++ headers and sources
- [ ] Review `include/sigyn_to_sensor_v2/` — any `.h` files should become `.hpp` (ROS 2 / Google convention)
- [ ] `performance_monitor.h` and `safety_publisher.h` — audit whether these are used in compilation; if headers-only features that were planned but not yet wired, move to `~/other_repository`
- [ ] Check for and remove any `printf` / `cout` calls; use `RCLCPP_*` logging
- [ ] The `docs/` directory has good audit documents; keep but convert to proper Doxygen or package README sections
- [ ] **TODO from TODO_list.txt:**
  - Implement heartbeat/watchdog from `sigyn_to_sensor_v2`
  - Battery discharge prediction
  - Enable SafetyCoordinator on Board2 and Board3

### 5.2 Extraction steps

- [ ] Create `~/sigyn_teensy_bridge_ws/src/sigyn_teensy_bridge/`
- [ ] `git init`, rename package, initial commit
- [ ] Create `wimblerobotics/sigyn_teensy_bridge` on GitHub, push
- [ ] Update `Sigyn2/packages.yaml` to reference new repo
- [ ] Update `sigyn.launch.py` to use `sigyn_teensy_bridge` package name
- [ ] Remove `sigyn_to_sensor_v2/` from Sigyn monorepo

---

## Section 6 — Extract `TeensyV2` → `sigyn_teensy_firmware`

**New repo name:** `wimblerobotics/sigyn_teensy_firmware`  
**Keep in sync with:** `sigyn_teensy_bridge` (they are tightly coupled — shared message protocol)

### 6.1 Pre-extraction cleanup

- [ ] Add SPDX copyright headers to all source files
- [ ] `src/roboclaw_test.cpp` — is this a test file or active code? If test, add `#ifdef`/`TEST_ONLY` guard or move to `test/`
- [ ] `src/elevator_board.cpp` — audit whether this is Board 3 main or a helper
- [ ] Review `modules/` and `common/` for any deprecated or dead code
- [ ] Address **CRITICAL** TODO items before extraction:
  - Enable Safety on Board 3 (Gripper): set `BOARD_HAS_SAFETY 1`
  - Fix inter-board communication: implement `fault_handler` in `board1_main.cpp`
  - Sensor timeout safety: ensure sensors trigger `isUnsafe` on stall
- [ ] Add a `README.md` covering: hardware revision, board pinout references, build instructions with PlatformIO
- [ ] Ensure `platformio.ini` is clean and all `boards/` config is up to date

### 6.2 Extraction steps

- [ ] Create `~/sigyn_teensy_firmware_ws/src/sigyn_teensy_firmware/`
- [ ] `git init`, rename references, initial commit
- [ ] Create `wimblerobotics/sigyn_teensy_firmware` on GitHub, push
- [ ] Remove `TeensyV2/` from Sigyn monorepo

---

## Section 7 — Extract `yolo_oakd_test` → `sigyn_oakd_detector`

**New repo name:** `wimblerobotics/sigyn_oakd_detector`

### 7.1 Pre-extraction cleanup

- [ ] Rename package: `yolo_oakd_test` → `sigyn_oakd_detector` in `CMakeLists.txt` and `package.xml`
- [ ] Fix `package.xml` maintainer: `"Your Name" / "you@example.com"` → correct values
- [ ] Fix `package.xml` license: `MIT` → `Apache-2.0` (to match rest of project)
- [ ] Update description: remove "Isolated test package" wording
- [ ] **Model file discrepancy:** The launch sub-file `oakd_yolo26_detector.launch.py` calls the node from `yolo_oakd_test` and expects `models/can_detector.blob`, but the package ships `models/oakd_yolov5_v5a.blob`. Determine which is the correct model, rename / update `launch.py` accordingly. Also note `models/can_data.yaml` references V5a.
- [ ] Move `base/config/nn/can_yolov5.json` and `can_yolov8.json` into this package under `config/nn/`
- [ ] Move `base/config/oakd_camera.yaml` into this package (it's OAK-D specific config)
- [ ] `CanDetection.msg` is defined here but used across packages; evaluate moving to `sigyn_interfaces` for proper multi-package sharing
- [ ] Apply Google style + clang-format to `oakd_can_detector.py`
- [ ] Add SPDX copyright headers
- [ ] Rename launch file `test_can_detection.launch.py` → `oakd_detector.launch.py`

### 7.2 Impact on `base`/`sigyn_bringup`

After extraction, update references in `sigyn.launch.py`:
- `sub_launch/oakd_yolo26_detector.launch.py` should use new package name

### 7.3 Extraction steps

- [ ] Create `~/sigyn_oakd_detector_ws/src/sigyn_oakd_detector/`
- [ ] `git init`, rename package, initial commit
- [ ] Create `wimblerobotics/sigyn_oakd_detector` on GitHub, push
- [ ] Update `Sigyn2/packages.yaml` to reference new repo
- [ ] Remove `yolo_oakd_test/` from Sigyn monorepo

---

## Section 8 — `can_do_challenge` Cleanup

This package will remain in the monorepo as the application-level behavior for the Coke can challenge.

### 8.1 Dependency issues

- [ ] `<depend>behaviortree_cpp_v3</depend>` — The rest of the project uses `behaviortree_cpp` (v4 API). Migrate `can_do_challenge` from v3 to v4. Key API differences: `BT::NodeStatus`, `BT::InputPort`, action node base class names. This is a significant refactor.
- [ ] `<depend>gripper_camera_detector</depend>` — This package does not exist in `sigyn_ws/src`. Determine whether this should be `pi_can_detector` (on sigynVision) or something else. Either add it or remove the dependency.
- [ ] Review whether `depthai_ros_msgs` is still needed now that OAK-D detection is via `yolo_oakd_test`/`sigyn_oakd_detector`

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

## Section 9 — `bluetooth_joystick` Cleanup

This package is self-contained and relatively clean.

- [ ] Add SPDX copyright headers
- [ ] Apply Google style + clang-format to `src/joystick_node.cpp`
- [ ] `udev_rules/` has two files (`01-joystick.rules` and `93.nimbus.rules` — note the `.` separator on one, `_` on the other); consolidate/fix naming
- [ ] README is present — review for accuracy
- [ ] Consider whether this warrants its own repo (`wimblerobotics/sigyn_bluetooth_joystick`) or stays in the monorepo

---

## Section 10 — `rviz` Package

Very minimal: one config file, `CMakeLists.txt`, `package.xml`, `README.md`.

- [ ] Option A: Merge into `sigyn_bringup` (the bringup package already references the rviz config by `get_package_share_directory("rviz")`)
- [ ] Option B: Keep as a standalone package (clean, minimal, reasonable)
- [ ] Whichever option: add SPDX header to `CMakeLists.txt` and `package.xml`
- [ ] Review `config/config.rviz` to ensure all panel/display references match current topic names (especially after renaming/refactoring)

---

## Section 11 — Sigyn2 Deployment Repo Updates

After each extraction above, `Sigyn2/config/packages.yaml` and `Sigyn2/config/robots.yaml` need updates:

| Action | `packages.yaml` change |
|---|---|
| `sigyn_behavior_trees` extracted | Add new group `sigyn_behavior_trees` with `wimblerobotics/sigyn_behavior_trees` |
| `base` renamed to `sigyn_bringup` | Update `sigyn_navigation` group — Sigyn monorepo name stays but package name changes |
| `sigyn_to_sensor_v2` extracted | Add `sigyn_hardware` repo entry: `wimblerobotics/sigyn_teensy_bridge` |
| `TeensyV2` extracted | No ROS package to add (firmware only), but document it |
| `yolo_oakd_test` extracted | Add `sigyn_vision` group: `wimblerobotics/sigyn_oakd_detector` |

---

## Section 11b — `pi_gripper` Cleanup

`pi_gripper` provides PCA9685 PWM servo control for the gripper assembly on sigynVision (Raspberry Pi 5). It is deployed independently and is not part of `sigyn_ws/src` on sigyn7900a.

- [ ] Add SPDX copyright headers: `# SPDX-License-Identifier: Apache-2.0` / `# Copyright 2026 Wimble Robotics`
- [ ] Fix `package.xml` maintainer if it still has placeholder values
- [ ] Apply PEP 8 / `ruff` formatting to all Python source files
- [ ] Add a `README.md` covering: hardware wiring (PCA9685 I2C address, servo channel assignments), udev rules needed, deployment instructions
- [ ] Verify `Sigyn2/packages.yaml` `pi_gripper` group entry has correct git URL and branch
- [ ] Ensure the `pi_gripper` action interface (if any) is defined in `sigyn_interfaces`, not privately inside this repo
- [ ] Confirm the topic/service names match what `sigyn_to_sensor_v2` (or `sigyn_teensy_bridge`) and `can_do_challenge` expect
- [ ] Add basic unit tests with `pytest` for any non-ROS servo math/limit-checking logic

---

## Section 12 — Cross-Cutting: Style, Naming, and Copyright

### 12.1 SPDX copyright headers

Add to **every** `.cpp`, `.hpp`, `.h`, `.py`, `.lua`, `CMakeLists.txt`, `package.xml`, `.launch.py` file:

```
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Wimble Robotics
```

For Python:
```python
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Wimble Robotics
```

For CMake/Lua:
```
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Wimble Robotics
```

**Note:** The GitHub organization URL slug is `wimblerobotics` (no space) — this is correct and intentional. The human-readable company name in copyright notices, package.xml `<maintainer>`, and documentation is **Wimble Robotics** (with a space).

Some files already have SPDX headers (the newer OAK-D sub-launch files). Apply consistently everywhere.

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
- `yolo_oakd_test/yolo_oakd_test/oakd_can_detector.py`
- `scripts/*.py`

### 12.5 Add a top-level `.clang-format`

Place at `Sigyn/.clang-format`:
```yaml
BasedOnStyle: Google
ColumnLimit: 100
DerivePointerAlignment: false
PointerAlignment: Left
```

---

## Section 13 — Testing

### 13.1 `sigyn_to_sensor_v2` (→ `sigyn_teensy_bridge`)

Tests exist in `TeensyV2/test/` (PlatformIO side). On the ROS 2 side:
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

Currently `sigyn.launch.py` defaults to the nav2 stock BT XML for `navigate_to_pose`. The `base/config/bt1.xml` and `sigyn_behavior_trees/config/patrol.xml` are custom. These should eventually be brought together under a custom nav BT that integrates Sigyn's recovery behaviors.

### 14.3 `precheck.launch.py` — integrate with `sigyn.launch.py`

Currently `precheck` is a separate launch. Consider making it the canonical entry point that then includes `sigyn.launch.py`, so the robot can never accidentally be started without device checks.

### 14.4 Consolidate `rviz` config

`rviz/config/config.rviz` and `yolo_oakd_test/config/can_detection.rviz` are separate RViz configs. After renaming packages, ensure all topic references are current and create one canonical general-purpose config and one OAK-D detection visualization config.

### 14.5 Remove `CMakePresets.json` from `base/`

This file is for local IDE integration (VS Code CMake Tools) and references machine-specific toolchain paths. It should be in `.gitignore` or replaced with a generic version.

### 14.6 `navigation_launch.py` — confirm it is the right nav2 bringup entry point

`nav2_bringup.launch.py` includes `navigation_launch.py`. The `navigation_launch.py` remaps Nav2's output to `/cmd_vel_nav` (which feeds the twist multiplexer). Confirm this remapping survives across nav2 package updates (it's a custom file that may need to be re-diffed against the nav2 stock version each release).

---

## Execution Order (Suggested)

1. **Git/Repo hygiene** (Section 1) — quick wins, no functional impact
2. **Rename `base` → `sigyn_bringup`** (Section 2) — do in one PR to keep it atomic
3. **`base` config/launch file purge** (Section 3) — remove dead files, create cartographer launch
4. **Extract `sigyn_behavior_trees`** (Section 4) — in progress
5. **Style sweep: SPDX + clang-format** (Section 12) — do per-package as each is touched
6. **Extract `sigyn_to_sensor_v2` + `TeensyV2`** (Sections 5, 6) — do together (tightly coupled)
7. **Extract `yolo_oakd_test`** (Section 7)
8. **`can_do_challenge` cleanup** (Section 8) — especially BT v3 → v4 migration
9. **Testing** (Section 13) — add tests after each package is in a clean state
10. **`Sigyn2` updates** (Section 11) — update after each extraction

---

## Open Questions

1. Should `bluetooth_joystick` stay in the monorepo or get its own repo? (Lean: own repo, as it is hardware-specific and others might reuse it)
2. Should `rviz` merge into `sigyn_bringup` or stay standalone? (Lean: merge — it's only one config file)
3. Should `can_do_challenge` eventually move to its own repo? It's application-specific but has significant infrastructure (BT nodes, action servers, PI camera integration).
4. The `can_do_challenge` `CanDetection.msg` vs. `yolo_oakd_test` `CanDetection.msg` — are these the same message or different? If the same, consolidate into `sigyn_interfaces`.
5. Does anything actually use `base/config/pcl.yaml`? If the pointcloud launch is dead, this config is too.
6. What is the intended purpose of `config/gazebo.yaml` vs `config/gz_bridge.yaml`? Both appear to exist. Is `gazebo.yaml` for something other than the bridge?
