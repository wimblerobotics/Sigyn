# AI_CONTEXT.md — Sigyn Monorepo State

> Machine-readable context for AI coding agents working in this repo.
> **Branch:** `sigyn2` | **ROS Distro:** Jazzy | **Last updated:** 2026-02-25

---

## 1. What This Repo Is

`wimblerobotics/Sigyn` is the primary monorepo for the Sigyn house-patrolling robot. It contains:
- ROS 2 packages (C++ and Python)
- Behavior tree XMLs
- Configuration and launch files
- Teensy 4.1 firmware (PlatformIO) in `wimblerobotics/sigyn_teensy_boards` (migrated from `TeensyV2/` which has been removed from this repo)

The robot runs on Ubuntu 24.04 + ROS 2 Jazzy. Simulation uses Gazebo Harmonic.

---

## 2. Active Packages in This Monorepo

| Package | Language | Purpose |
|---|---|---|
| `sigyn_bringup` | Python (launch) | Main bringup: `sigyn.launch.py`, Navigation configs, map files |
| `rviz` | — | RViz config only |
| `udev` | — | udev rules for device nodes |

### Packages in separate repos (symlinked into `~/sigyn_ws/src/`)

| Package | Repo | Workspace |
|---|---|---|
| `sigyn_description` | `wimblerobotics/sigyn_description` | `~/sigyn_ws/src/` |
| `sigyn_interfaces` | `wimblerobotics/sigyn_interfaces` | `~/sigyn_ws/src/` |
| `sigyn_behavior_trees` | `wimblerobotics/sigyn_behavior_trees` | extraction in progress |
| **`sigyn_oakd_detection`** | **`wimblerobotics/sigyn_oakd_detection`** | **`~/sigyn_oakd_detection_ws/src/`** |
| **`sigyn_teensy_boards`** | **`wimblerobotics/sigyn_teensy_boards`** | **`~/sigyn_ws/src/sigyn_teensy_boards/`** (PlatformIO only, not colcon-built) |
| `wr_ldlidar` | `wimblerobotics/wr_ldlidar` | `~/sigyn_ws/src/` |
| `wr_teleop_twist_keyboard` | *(fork)* | `~/sigyn_ws/src/` |

---

## 3. Recently Completed: `yolo_oakd_test` Removal

**`yolo_oakd_test` has been deleted from this monorepo** (commit on branch `sigyn2`, Feb 2026).

It has been superseded by the standalone package **`sigyn_oakd_detection`**:
- **New location:** `~/sigyn_oakd_detection_ws/src/sigyn_oakd_detection/`
- **GitHub:** `wimblerobotics/sigyn_oakd_detection`
- **Symlinked** into `~/sigyn_ws/src/` so it builds alongside monorepo packages
- **Launch entry point:** `oakd_detector.launch.py` (from `sigyn_oakd_detection`)

### Key message change

`OakdDetection.msg` (formerly `CanDetection.msg`) now lives in **`sigyn_interfaces`** (tagged v0.9.4). All consumers must import from `sigyn_interfaces`, not from `can_do_challenge` or any stale source.

### Updated launch files

| File | Change |
|---|---|
| `can_do_challenge/launch/step3_visual_acquire_launch.py` | Now delegates to `sigyn_oakd_detection/oakd_detector.launch.py` |
| `can_do_challenge/launch/oakd_detection_test1.launch.py` | Now delegates to `sigyn_oakd_detection/oakd_detector.launch.py` |
| `sigyn_bringup/launch/sub_launch/oakd_yolo26_detector.launch.py` | Should be updated to delegate to `sigyn_oakd_detection` (pending) |

---

## 4. In Progress: `TeensyV2` Migration to `sigyn_teensy_boards`

**Status:** `TeensyV2/` is kept in this monorepo as a read-only reference until
the new standalone repo is confirmed working. **Do not make firmware changes in
`TeensyV2/` — edit `~/sigyn_ws/src/sigyn_teensy_boards/` instead.**

- **New repo:** `wimblerobotics/sigyn_teensy_boards`
- **Local path:** `~/sigyn_ws/src/sigyn_teensy_boards/` (deployed by vcstool into `sigyn_ws/src/` alongside ROS packages, but `tool_repo: true` so colcon ignores it)
- **Contents:** Identical to `TeensyV2/` minus build artifacts and ROS scaffolding (`package.xml`, `CMakeLists.txt`)
- **New docs:** `AI_CONTEXT.md` + `TODO.md` added (architectural review, GPIO e-stop, message redesign)

### Bash aliases updated

All `compileBoard1`, `buildBoard1`, `buildBoard2`, `buildElevator`, `compileElevator`, `test_teensy` aliases now point to `~/sigyn_ws/src/sigyn_teensy_boards/`. The comment in `bashrc` notes when `TeensyV2/` can be removed.

### When to remove `TeensyV2/` from this monorepo

- [ ] Verify `compileBoard1` / `compileBoard2` / `compileElevator` succeed from new location
- [ ] Verify `buildBoard1` / `buildBoard2` / `buildElevator` upload successfully
- [ ] Verify `test_teensy` passes
- [ ] Update `REFACTORING_PLAN.md` Section 5/6 to mark TeensyV2 extraction complete

---

## 5. Key Topics

| Topic | Type | Publisher |
|---|---|---|
| `/oakd/detections` | `sigyn_interfaces/OakdDetection` | `sigyn_oakd_detection` |
| `/oakd/annotated_image` | `sensor_msgs/Image` | `sigyn_oakd_detection` |
| `/oakd/raw_image` | `sensor_msgs/Image` | `sigyn_oakd_detection` |
| `/cmd_vel` | `geometry_msgs/Twist` | twist_multiplexer output |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Nav2 (remapped into mux) |
| `/gripper/status` | `sigyn_interfaces/GripperStatus` | `sigyn_to_sensor_v2` |
| `/gripper/position/command` | `sigyn_interfaces/GripperPositionCommand` | behavior tree |
| `/gripper/home` | `std_msgs/Empty` | behavior tree |

---

## 6. Key Custom Messages (sigyn_interfaces)

| Message | Purpose |
|---|---|
| `OakdDetection` | OAK-D YOLO detection result (bounding box, class, pose) |
| `GripperStatus` | Elevator + extender position, limit switches |
| `GripperPositionCommand` | Absolute position command for elevator/extender |

Actions defined in `sigyn_interfaces`:
- `MoveElevator.action`
- `MoveExtender.action`

---

## 7. Behavior Tree Library

- **Framework:** BehaviorTree.CPP **v4** (`behaviortree_cpp` package)
- **`sigyn_behavior_trees`:** Core patrol/navigation BT nodes (separate repo, see above)
- **`can_do_challenge/include/can_do_challenge/bt_nodes.hpp`:** Challenge-specific nodes
- **`can_do_challenge/bt_xml/`:** XML trees. Current canonical tree is `main.xml`.
  - `old.xml` and `main.orig.xml` are stale backups (candidates for deletion)
- **`OAKDDetectCan`** node: subscribes to `/oakd/detections`; uses `OakdDetection.msg`

> ⚠️ `can_do_challenge` `package.xml` still declares `behaviortree_cpp_v3`. This is a known bug — it currently builds against v4 API. Migration cleanup is tracked in `REFACTORING_PLAN.md` Section 8.

---

## 8. Build

```bash
# From ~/sigyn_ws
colcon build --symlink-install

# Build only one package
colcon build --symlink-install --packages-select can_do_challenge

# Build sigyn_oakd_detection (separate workspace)
cd ~/sigyn_oakd_detection_ws
colcon build --symlink-install
```

Source both workspaces for full environment:
```bash
source ~/sigyn_ws/install/setup.bash
source ~/sigyn_oakd_detection_ws/install/setup.bash
```

---

## 9. Open Refactoring Work

See `REFACTORING_PLAN.md` for full details. Current priority order:

1. ✅ ~~Rename `base` → `sigyn_bringup`~~ (Section 2 — **DONE**)
2. ✅ ~~Extract `yolo_oakd_test`~~ (Section 7 — **DONE**)
3. Extract `sigyn_behavior_trees` (Section 4, in progress)
4. `can_do_challenge` BT v3→v4 migration + cleanup (Section 8)
5. Update `Sigyn2/packages.yaml` to add `sigyn_oakd_detection` entry

---

## 10. Safety Notes

- E-stop topics flow through `sigyn_to_sensor_v2` → Teensy serial
- Never send velocity commands while the elevator is in motion unless the BT safety subtree is active
- Board 3 (gripper/elevator) currently lacks `BOARD_HAS_SAFETY` — tracked in `TODO.md`
