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

## Next Steps (Phase 2 & 3)

### TODO - Vision Integration:
1. **OAK-D Object Detection**:
   - Implement HSV color detection for red cylinder (CokeZero)
   - Use Canny edge detection for outline
   - Return bounding box in camera frame
   - Compute depth from OAK-D stereo/depth
   - Node: `CanDetectedByOAKD`, `CanWithinReach`

2. **Pi Camera Integration**:
   - Similar HSV + Canny detection
   - Return bounding box for centering
   - Verify can still grasped
   - Nodes: `CanDetectedByPiCamera`, `CanCenteredInPiCamera`, `CanIsGrasped`

### TODO - Gripper Control Integration:
Look at `sigyn_to_sensor_v2` package for:
- Elevator position control
- Extender position control  
- Gripper open/close commands
- Current position feedback

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
- `MoveElevatorToHeight`: Move to specific height
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
