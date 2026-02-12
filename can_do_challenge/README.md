# Can Do Challenge - CokeZero Can Fetching

## Overview

This package implements an autonomous can-fetching behavior for Sigyn the robot. The robot:
1. Saves its starting location
2. Navigates to a known can location (from JSON database)
3. Uses OAK-D camera to visually acquire the can
4. Approaches within gripper reach
5. Uses Pi Camera for fine positioning
6. Raises elevator, extends gripper, and grasps the can
7. Returns to the starting location

## Phase 1 - Complete ✓

**Status**: Package created with complete behavior tree structure and placeholder implementations.

### What's Implemented:
- ✅ Complete package structure (CMakeLists.txt, package.xml)
- ✅ Behavior tree XML (`bt_xml/main.xml`) with full mission logic
- ✅ 40+ custom BT nodes with proper port declarations
- ✅ Placeholder implementations for all nodes
- ✅ JSON-based can location database
- ✅ Launch file with Groot monitoring support
- ✅ Reactive safety monitoring (battery, E-stop, tilt)
- ✅ Modular subtree design

## Phase 3 - Visual Acquisition (Action Server Integration) ✓

**Status**: Elevator control integrated with action server for reliable visual servoing.

### What's Implemented:
- ✅ `MoveElevatorAction`: StatefulActionNode using `/gripper/move_elevator` action
- ✅ `StepElevatorUpAction`: Incremental elevator movement with action server
- ✅ Position feedback from `/gripper/status` (GripperStatus messages)
- ✅ Visual servoing without velocity commands (action-based only)
- ✅ `ElevatorAtHeight`: Checks if can is at target pixel height
- ✅ `WaitForNewPiFrameProcessed`: Ensures fresh camera data between steps
- ✅ Behavior tree `raise_elevator.xml` with incremental stepping
- ✅ Tested on real hardware with successful can positioning

### Key Design Decisions:
- **No velocity commands**: All elevator movement uses MoveElevator action server
- **Incremental stepping**: Starts from home, steps up 2cm at a time until can reaches target
- **Visual feedback loop**: Each step followed by camera check (342±10 pixels target)
- **Robust position tracking**: Uses `/gripper/status` from teensy_bridge for real-time feedback
- **Graceful limits**: Max 50 attempts (1m travel) prevents runaway behavior

### Behavior Tree Structure:

```
MainTree
├── SaveRobotPose (save starting location)
├── SetupCanChallenge (load can location, prepare gripper)
└── ReactiveSequence
    ├── ReactiveRobotSafety (continuous monitoring)
    ├── NavigateToCanLocation (Nav2 path planning)
    ├── VisuallyAcquireCan
    │   ├── SearchForCanWithOAKD (rotate to find can)
    │   └── ApproachCanWithOAKD (move within reach)
    ├── GraspCan
    │   ├── RaiseElevatorToCanHeight
    │   ├── CenterCanInPiCamera (using extender)
    │   ├── ExecuteGrasp (lower, extend, close)
    │   └── RetractAfterGrasp
    └── ReturnToStart (Nav2 path back)
```

## Building

```bash
cd /home/ros/sigyn_ws
colcon build --symlink-install --packages-select can_do_challenge
source install/setup.bash
```

## Running

```bash
ros2 launch can_do_challenge can_do_challenge_launch.py
```

### Launch Arguments:
- `bt_xml_file`: Path to behavior tree XML (default: `main.xml`)
- `enable_groot`: Enable Groot monitoring (default: `true`)
- `groot_port`: Port for Groot ZMQ (default: `1667`)

## Configuration

### Can Locations Database
Edit `config/can_locations.json` to add/modify can locations:

```json
{
  "can_locations": [
    {
      "name": "CokeZeroCan",
      "location": {
        "x": 0.0,
        "y": 20.0,
        "z": 0.75
      }
    }
  ]
}
```

### Vision Model Artifacts
Place the latest Hailo model artifacts here (tracked in repo):
- `/home/ros/sigyn_ws/src/Sigyn/can_do_challenge/resources/models/`

Expected files:
- `fcc2_best.hef` (required)
- `fcc2_labels.txt` (required)
- `fcc2_best.onnx` (optional, reference/debug)

#### Compiling ONNX to HEF for Hailo-8

**IMPORTANT**: The HEF file must be compiled specifically for Hailo-8 (not Hailo-8L) to match the hardware on the Raspberry Pi.

**Prerequisites**:
- Docker installed on your system
- Hailo AI Software Suite Docker image: `hailo8_ai_sw_suite_2025-10:1`
- ONNX model file: `resources/models/fcc2_best.onnx`

**Compilation Steps**:

1. **Interactive Docker Session** (recommended for debugging):
   ```bash
   cd ~
   docker run --rm -it \
     -v ~/sigyn_ws/src/Sigyn/can_do_challenge:/workspace \
     hailo8_ai_sw_suite_2025-10:1 \
     /bin/bash
   
   # Inside the container:
   cd /workspace/resources/models
   hailo parser onnx fcc2_best.onnx
   hailo compiler --hw-arch hailo8 fcc2_best.har
   ```

2. **One-line Compilation** (automated):
   ```bash
   cd ~
   docker run --rm -it \
     -v ~/sigyn_ws/src/Sigyn/can_do_challenge:/workspace \
     hailo8_ai_sw_suite_2025-10:1 \
     /bin/bash -c "set -e && cd /tmp && \
      hailo parser onnx /workspace/resources/models/fcc2_best.onnx \
        --end-node-names /model.22/Sigmoid /model.22/Concat \
        --har-path /tmp/fcc2_best.har -y && \
      hailo optimize --use-random-calib-set /tmp/fcc2_best.har \
        --output-har-path /tmp/fcc2_best_optimized.har && \
      hailo compiler --hw-arch hailo8 /tmp/fcc2_best_optimized.har \
        --output-dir /tmp && \
      cp -f /tmp/fcc2_best.har /tmp/fcc2_best_optimized.har /tmp/fcc2_best.hef /workspace/resources/models/"
   ```

   Note: Output files may be owned by root. Fix permissions after compilation:
   ```bash
   sudo chown -R $(id -u):$(id -g) ~/sigyn_ws/src/Sigyn/can_do_challenge/resources/models/
   ```

The resulting `fcc2_best.hef` file will be saved in `resources/models/` and is ready for deployment on the Raspberry Pi with Hailo-8.

**Troubleshooting**:
- **Permission denied in `/workspace/resources/models`** (e.g., `pyhailort.log`):
  - Run Hailo commands from `/tmp` and use absolute paths so logs are written to `/tmp` instead of the mounted folder.
- **Copy failed with Permission denied** (root-owned artifacts): remove or change ownership of existing `fcc2_best.*` files in `resources/models/` on the host, then rerun the one-liner.
- If you see "HEF was compiled for Hailo8L", recompile with `--hw-arch hailo8` (not `hailo8l`)
- **No calibration images available**: Use `hailo optimize --use-random-calib-set` to generate a quantized HAR before compiling.
- Ensure the ONNX model uses opset 11-13 and fixed input dimensions (no dynamic axes)
- For model optimization options, consult the Hailo Dataflow Compiler documentation

## Next Steps (Phase 2 & Beyond)

### ✅ COMPLETE - Visual Acquisition (Phase 3):
1. **Elevator Control with Action Server**:
   - ✅ MoveElevatorAction integrated with `/gripper/move_elevator`
   - ✅ StepElevatorUpAction for incremental visual servoing
   - ✅ Position feedback from `/gripper/status`
   - ✅ `raise_elevator.xml` behavior tree tested on hardware
   - ✅ Visual feedback from Pi camera at 342±10 pixels

### TODO - Vision Integration:
1. **OAK-D Object Detection**:
   - Implement HSV color detection for red cylinder (CokeZero)
   - Use Canny edge detection for outline
   - Return bounding box in camera frame
   - Compute depth from OAK-D stereo/depth
   - Node: `CanDetectedByOAKD`, `CanWithinReach`

2. **Pi Camera Integration**:
   - ✅ Detection working (`/gripper/camera/detections`)
   - ✅ ElevatorAtHeight condition implemented
   - TODO: Return bounding box for centering
   - TODO: Verify can still grasped
   - Nodes: `CanDetectedByPiCamera`, `CanCenteredInPiCamera`, `CanIsGrasped`

### TODO - Gripper Control Integration:
Look at `sigyn_to_sensor_v2` package for:
- ✅ Elevator position control (MoveElevatorAction complete)
- ✅ Elevator position feedback (`/gripper/status` integrated)
- TODO: Extender position control  
- TODO: Gripper open/close commands

### TODO - Nav2 Integration:
- Replace placeholder `ComputePathToPose` and `FollowPath` with actual Nav2 action clients
- Similar to `perimeter_roamer_v3` implementation

### TODO - Simulation:
- Clone `description/worlds/home.world` to `can_challenge_world.world`
- Add table model at (0.0, 20.0)
- Add CokeZero can model (red cylinder) on table
- Configure OAK-D and Pi Camera in Gazebo (see URDF)
- Create sim launch file based on `sim.launch.py`

## Custom Nodes Reference

### Vision Conditions:
- `CanDetectedByOAKD`: Check if can visible in OAK-D
- `CanDetectedByPiCamera`: Check if can visible in Pi Camera
- `CanCenteredInPiCamera`: Check if bounding box centered
- `CanWithinReach`: Check distance < 0.3m
- `CanIsGrasped`: Verify can still detected after grasp

### Gripper Actions:
- `LowerElevator`: Lower to minimum height
- `LowerElevatorSafely`: Lower to safe travel height with can
- `MoveElevatorAction`: Move to specific height using action server ✅ **Phase 3**
- `StepElevatorUpAction`: Incremental 2cm steps with visual feedback ✅ **Phase 3**
- `RetractExtender`: Fully retract extender
- `OpenGripper` / `CloseGripperAroundCan`: Control gripper jaws
- `AdjustExtenderToCenterCan`: Fine position using Pi Camera

### Navigation Actions:
- `ComputePathToCanLocation`: Plan path to can area
- `FollowPath`: Execute Nav2 path
- `MoveTowardsCan`: Visual servoing approach
- `RotateRobot`: Spin to search

## Architecture Notes

- **Reactive Safety**: Continuously monitors battery, E-stop, tilt throughout mission
- **Shared Blackboard**: All subtrees use `__shared_blackboard="true"` for data flow
- **ReactiveRepeat Decorator**: Allows safety interrupts during loops
- **Nav2 Integration Ready**: Structure matches `perimeter_roamer_v3` patterns
- **Placeholder Implementations**: All nodes return SUCCESS/log for testing structure

## Dependencies

- ROS 2 Jazzy
- BehaviorTree.CPP v3
- Nav2 (for navigation)
- OpenCV (for vision)
- nlohmann-json (for config parsing)
- cv_bridge

## License

Apache-2.0
