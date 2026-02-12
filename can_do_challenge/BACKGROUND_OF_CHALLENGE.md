# Can-Do Challenge: Background & Constraints

## Goal
Enable the Sigyn robot to autonomously navigate to, approach, and grasp a soda can placed on a table surface in an obstacle-rich environment.

## Robot Specifications

### Physical Configuration
- **Base**: Two-wheeled differential drive with circular body
- **Gripper Assembly**: 
  - Mounted on linear elevator (vertical motion)
  - Elevator mounted on linear extender (forward/backward motion)
  - **No rotational freedom** - arm is fixed relative to robot top plate
  - Gripper can only move vertically (elevator) and extend/retract (linear motion)
  
### Reachability Constraints
- **Gripper reach**: Single fixed extension distance (TODO: measure actual value, estimated 12-18 inches / 0.30-0.46m from robot edge)
- **Minimum grasp height**: ~0.6m above floor (base_footprint)
- **Maximum grasp height**: Set by elevator maximum elevation
- **Stability**: Maximum extension is validated to prevent tipping with can-sized objects
- **Offset from body**: Gripper extends slightly beyond circular body radius (Nav2 does not currently account for this overhang)

### Height Reference Frames
- `base_footprint`: Floor level
- `base_link`: ~0.22m above floor (0.07m + 0.15m)
- Robot top plate: ~0.15m above base_link
- OAK-D camera: ~1m+ above top plate, pointed downward

## Sensor Suite

### OAK-D Camera (Primary Perception)
**Mounted**: >1m above top plate, angled downward to see robot front and gripper

**Capabilities**:
1. **Depth pointcloud** for obstacle detection (better than LIDAR alone)
2. **Object detection** with 3D pose estimation (can centroid relative to gripper frame)
3. **Local costmap generation** (3m x 3m) for Nav2 navigation
4. **Future**: Semantic segmentation (floor mats, cables)

**Accuracy**: Sub-cm at <2m range (assumption to be validated)

### Pi Camera (Fine-Tuning)
**Mounted**: Behind and above gripper

**Purpose**: Final alignment verification
- Detect can top position for elevator height adjustment
- Confirm can remains grasped after pickup

### LIDAR
Provides additional costmap data, complements OAK-D depth sensing

## Navigation Infrastructure

### Nav2 Stack
- **Status**: Integrated and functional
- **Local costmap**: 3m x 3m, reliable obstacle representation
- **Tolerance**: Currently too loose, needs tuning in `navigation.sim.yaml`
- **Capabilities**: Obstacle-aware path planning with `ComputePathToPose` and `NavigateToPoseAction`

### Current Problem with ApproachCanWithOAKD
The existing subtree performs **blind visual servoing**:
- Rotates until can is detected
- Moves directly toward can using cmd_vel with `within_distance` threshold (no obstacle awareness)
- **Failure modes**:
  - Crashes into table if approaching from side of narrow table (moves to `within_distance` from can, ignoring table)
  - Doesn't consider optimal approach angle (e.g., prefers long hypotenuse vs. short table edge)
  - No validation of reachability before attempting approach
  - Ignores obstacles between robot and can
  - The `within_distance` parameter only measured robot-to-can distance, not accounting for table geometry

## Task Simplifications (Current Scope)

### Environment Assumptions
- Can is on a **flat, horizontal table surface**
- Can is **within reachable position** on table (not in center of wide table)
- Table treated as **box-like obstacle extending to floor** (robot cannot fit under table)
- **No refrigerator opening** or shelf navigation (future work)
- **No clutter** on table surface (future work)

### Gripper Limitations (Current Scope)
- Can only grasp from the **side** (no tilting for angled objects)
- No wrist articulation for orientation adjustment
- Future gripper will have more DOF (new BT required)

## Approach Strategy Requirements

### Phase 1: Basic Safety (Obstacle-Aware Navigation)
Replace blind cmd_vel approach with Nav2-based navigation:
- Compute goal pose as offset from can position
- Use `NavigateToPoseAction` for obstacle-aware movement
- Validates basic safety without optimization

### Phase 2: Optimal Pose Selection
Analyze table geometry and select best approach pose:
1. **Extract table boundary** from local costmap (obstacle containing can)
2. **Approximate as rectangle** (simplification for now)
3. **Generate 4-8 candidate poses** at table edges, perpendicular to edges
4. **Offset for robot radius** plus gripper overhang
5. **Evaluate paths** using `ComputePathToPose` for each candidate
6. **Select best feasible path**:
   - Shortest path among valid options
   - Within reachability tolerance
   - Accounts for obstacles (chairs, trash cans) blocking some approaches
7. **Navigate** to chosen pose

### Phase 3: Advanced Reachability & Refinement
- **Pre-compute elevator height** from OAK-D 3D can pose
- **Early failure detection**: Verify can is reachable given table geometry
- **Visual servoing fine-tuning**: Use Pi camera for final alignment
- **Grasp verification**: Confirm can remains in gripper after pickup
- **Error reporting**: `SaySomething` node explains failure reasons

## Key Parameters (Blackboard Configuration)
- `max_gripper_extension`: TODO: measure actual value (estimated 0.30-0.46m)
- `min_grasp_height`: 0.6m (above base_footprint)
- `max_grasp_height`: [elevator maximum]
- `gripper_overhang_offset`: Small offset for gripper extending beyond robot body (Nav2 doesn't account for this)
- `approach_distance_tolerance`: [how close Nav2 must get to goal pose]
- `within_distance`: 0.55m (OLD threshold, problematic - replaced by pose-based approach)

**Note**: Nav2 already knows robot geometry, so most nodes don't need explicit `robot_radius` parameter

## Open Questions / Future Validation
- [ ] Verify OAK-D 3D pose accuracy vs. ground truth
- [ ] Validate URDF accuracy for gripper actuator frame
- [ ] Tune `navigation.sim.yaml` goal pose tolerances
- [ ] Test table rectangle approximation with non-rectangular tables
- [ ] Handle can unreachable scenarios gracefully
- [ ] Add support for heavier objects (different BT)

## Testing Strategy
- **Incremental validation**: Test each phase independently
- **Reduced test trees**: Position robot manually to exercise single features
- **Repeatability**: Verify consistent behavior across multiple runs
- **Edge cases**: Narrow tables, side approaches, obstacles blocking optimal paths
