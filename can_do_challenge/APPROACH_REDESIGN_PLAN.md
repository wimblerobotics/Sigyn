# ApproachCanWithOAKD Redesign: Phased Implementation Plan

## Overview
Replace blind visual servoing with obstacle-aware, optimized pose selection for safe and reliable can grasping.

---

## Phase 1: Basic Obstacle-Aware Navigation

### Goal
Replace `MoveTowardsCan` cmd_vel approach with Nav2 obstacle-aware navigation.

### New BT Nodes Required

#### `ComputeCanApproachGoal`
**Type**: Action  
**Purpose**: Compute a single goal pose near can position for Nav2 to navigate to

**Inputs**:
- `can_location` (PointStamped): 3D pose of can from OAK-D detector
- `gripper_overhang_offset` (double): Small safety margin beyond Nav2's footprint (default: 0.05m)

**Outputs**:
- `goal_pose` (PoseStamped): Goal pose in map frame, facing can

**Logic**:
1. Get can position in map frame
2. Compute vector from robot's current pose to can
3. Place goal pose along line toward can, offset slightly for gripper overhang
4. Orient goal pose to face can directly
5. Return PoseStamped

**Note**: Nav2 handles obstacle avoidance and robot footprint. This node just computes a reasonable goal pose facing the can. The old `approach_distance`/`within_distance` approach was problematic because it measured robot-to-can distance without accounting for table geometry.

**Failure Conditions**:
- Transform lookup fails
- Can location invalid

---

### Modified BT: ApproachCanWithOAKD_Phase1

```xml
<BehaviorTree ID="ApproachCanWithOAKD_Phase1">
    <Sequence name="NavBasedApproach">
        <!-- Ensure we have fresh can detection -->
        <Action ID="OAKDDetectCan"
            objectOfInterest="{objectOfInterest}"
            can_detected="{can_detected}"
            can_location="{can_location}" />
        
        <!-- Compute goal pose near can -->
        <Action ID="ComputeCanApproachGoal"
            can_location="{can_location}"
            gripper_overhang_offset="0.05"
            goal_pose="{approach_goal}" />
        
        <!-- Navigate using Nav2 (obstacle aware) -->
        <Action ID="NavigateToPoseAction"
            goal="{approach_goal}"
            error_code_id="{nav_error_code}" />
        
        <!-- Final check: still see can? -->
        <Action ID="WaitForNewOAKDFrame" />
        <Condition ID="CanWithinReach"
            can_detected="{can_detected}"
            can_location="{can_location}"
            within_distance="{within_distance}" />
    </Sequence>
</BehaviorTree>
```

### Testing
1. Place can on table edge, robot across room
2. Launch Phase 1 BT
3. Verify: Robot navigates around obstacles, stops ~0.45m from can
4. Measure actual standoff distance vs. desired

### Success Criteria
- No collisions with table or obstacles
- Robot arrives within 0.5m of can
- Facing can within ±15°

---

## Phase 2: Multi-Candidate Pose Optimization

### Goal
Analyze table geometry, generate multiple candidate poses, select optimal approach.

### New BT Nodes Required

#### `ExtractTableBoundary`
**Type**: Action  
**Purpose**: Extract table perimeter from costmap using can location

**Inputs**:
- `can_location` (PointStamped): Can position
- `search_radius` (double): Max radius to search for table edges (default: 2.0m)

**Outputs**:
- `table_corners` (Polygon): 4-8 corner points of detected table boundary
- `table_detected` (bool): Whether table was successfully extracted

**Logic**:
1. Query local costmap (frame can be looked up from costmap topic/TF, likely map or odom frame)
2. Find connected obstacle region containing can position
3. Use OpenCV to fit minimum area rectangle or convex hull
4. Return corner points in map frame

**Failure Conditions**:
- Can position not in obstacle region (can on floor?)
- No costmap data available
- Polygon extraction fails

**Implementation Note**: Costmap frame can be determined at runtime, no need for parameter.

---

#### `GenerateApproachCandidates`
**Type**: Action  
**Purpose**: Generate 4-8 candidate approach poses around table

**Inputs**:
- `table_corners` (Polygon): Table boundary
- `can_location` (PointStamped): Can position
- `gripper_overhang_offset` (double): Small offset beyond table edge (default: 0.05m)
- `num_candidates` (int): Number of candidate poses (default: 8)

**Outputs**:
- `candidate_poses` (PoseArray): Array of candidate approach poses

**Logic**:
1. For each table edge:
   - Find point on edge closest to can
   - Compute normal vector pointing outward from table
   - Place candidate pose at small offset from edge (Nav2 handles robot footprint)
   - Orient pose perpendicular to edge (facing can)
2. Sort candidates by distance from robot's current position
3. Return PoseArray

**Note**: No need for explicit `robot_radius` - Nav2 already accounts for robot geometry. Only add small `gripper_overhang_offset` since Nav2 doesn't know about gripper extending beyond body.

---

#### `SelectBestApproachPose`
**Type**: Action  
**Purpose**: Evaluate candidates and select best feasible pose

**Inputs**:
- `candidate_poses` (PoseArray): Candidate poses to evaluate
- `max_path_length` (double): Reject paths longer than this (default: 5.0m)
- `planner_id` (string): Nav2 planner to use (default: "GridBased")

**Outputs**:
- `selected_pose` (PoseStamped): Best valid approach pose
- `selected_path` (Path): Path to selected pose
- `selection_success` (bool): Whether any valid pose found

**Logic**:
1. For each candidate (in sorted order):
   - Call `ComputePathToPose` with candidate goal
   - If path valid and within length limit:
     - Check if final pose is within tolerance of goal
     - Score based on: path length, estimated reachability
   - Store valid paths with scores
2. Select path with best score
3. Return selected pose and path

**Failure Conditions**:
- No valid paths found (all blocked or too far)
- All paths exceed max length

---

#### `VerifyCanReachability`
**Type**: Condition  
**Purpose**: Check if can is graspable from current/planned pose

**Inputs**:
- `can_location` (PointStamped): Can position
- `table_corners` (Polygon): Table boundary (optional)
- `max_gripper_extension` (double): Maximum safe extension (default: 0.46m)
- `check_from_pose` (PoseStamped): Pose to check from (default: current robot pose)

**Outputs**:
- None (returns SUCCESS/FAILURE)

**Logic**:
1. Compute distance from check_from_pose to can
2. If table_corners provided:
   - Find table edge closest to robot
   - Compute perpendicular distance from edge to can
   - Verify distance ≤ max_gripper_extension
3. Else: simple distance check
4. Verify can Z-height within elevator range

**Success**: Can is reachable  
**Failure**: Can too far, wrong height, or blocked

---

### Modified BT: ApproachCanWithOAKD_Phase2

```xml
<BehaviorTree ID="ApproachCanWithOAKD_Phase2">
    <Sequence name="OptimizedApproach">
        <!-- Get fresh can detection -->
        <Action ID="OAKDDetectCan"
            objectOfInterest="{objectOfInterest}"
            can_detected="{can_detected}"
            can_location="{can_location}" />
        
        <!-- Extract table geometry from costmap -->
        <Action ID="ExtractTableBoundary"
            can_location="{can_location}"
            table_corners="{table_corners}"
            table_detected="{table_detected}" />
        
        <!-- Generate multiple candidate approach poses -->
        <Action ID="GenerateApproachCandidates"
            table_corners="{table_corners}"
            can_location="{can_location}"
            gripper_overhang_offset="0.05"
            num_candidates="8"
            candidate_poses="{candidate_poses}" />
        
        <!-- Select best feasible pose -->
        <Action ID="SelectBestApproachPose"
            candidate_poses="{candidate_poses}"
            max_path_length="5.0"
            selected_pose="{approach_goal}"
            selected_path="{approach_path}"
            selection_success="{pose_selected}" />
        
        <!-- Fail early if no valid pose -->
        <ReactiveFallback>
            <Condition ID="CheckBoolFlag" flag="{pose_selected}" expected="true" />
            <Sequence>
                <Action ID="SaySomething" message="No valid approach pose found. Can may be unreachable." />
                <AlwaysFailure />
            </Sequence>
        </ReactiveFallback>
        
        <!-- Navigate to selected pose -->
        <Action ID="NavigateToPoseAction"
            goal="{approach_goal}"
            error_code_id="{nav_error_code}" />
        
        <!-- Verify we're in position -->
        <Action ID="WaitForNewOAKDFrame" />
        <Condition ID="VerifyCanReachability"
            can_location="{can_location}"
            table_corners="{table_corners}"
            max_gripper_extension="{max_gripper_extension}" />
    </Sequence>
</BehaviorTree>
```

### Testing
1. **Narrow table test**: Place can on narrow table, robot on long side
   - Should navigate to short end, not bash into long side
2. **Obstacle blocking test**: Place chair blocking some approaches
   - Should select alternate pose around obstacle
3. **Unreachable test**: Place can in center of wide table
   - Should report unreachable and fail early

### Success Criteria
- Selects shorter path around table edges
- Avoids obstacles between robot and table
- Fails gracefully when can is unreachable
- Final pose allows gripper to reach can with <18" extension

---

## Phase 3: Advanced Reachability & Refinement

### Goal
Add elevator height pre-computation, Pi camera fine-tuning, post-grasp verification.

### New BT Nodes Required

#### `ComputeElevatorHeightFromOAKD`
**Type**: Action  
**Purpose**: Pre-compute elevator height from OAK-D 3D detection

**Inputs**:
- `can_location` (PointStamped): Can centroid in 3D (with Z)
- `can_height` (double): Physical height of can (default: 0.115m for soda can)
- `gripper_offset` (double): Vertical offset from gripper frame to fingers

**Outputs**:
- `target_elevator_height` (double): Desired elevator position (meters)

**Logic**:
1. Get can Z-coordinate in base_link frame
2. Compute can top position: Z + (can_height / 2)
3. Compute grasp point: slightly below can top (e.g., 60% of can height)
4. Convert to elevator position accounting for gripper offset
5. Return target height

---

#### `MoveElevatorToHeight`
**Type**: Action  
**Purpose**: Command elevator to specific height (already exists in node model)

**Note**: Already defined in TreeNodesModel, verify implementation exists

---

#### `FineAlignWithPiCamera`
**Type**: Action  
**Purpose**: Use Pi camera visual servoing for final positioning

**Inputs**:
- `objectOfInterest` (string): Object to center (e.g., "CokeZeroCan")
- `max_iterations` (int): Max adjustment cycles (default: 10)
- `tolerance_pixels` (int): Acceptable centering error (default: 20 px)

**Outputs**:
- `alignment_success` (bool): Whether can was aligned

**Logic**:
1. Wait for new Pi camera frame
2. Check if can is centered in frame
3. If not: compute small adjustment to robot pose or extender
4. Apply adjustment
5. Repeat until centered or max iterations

**Note**: May reuse/refactor existing `CenterCanInPiCamera` subtree

---

#### `VerifyCanGrasped`
**Type**: Condition  
**Purpose**: Confirm can is still in gripper after pickup

**Inputs**:
- `objectOfInterest` (string): Object that should be grasped

**Outputs**:
- None (SUCCESS if grasped, FAILURE if dropped)

**Logic**:
1. Wait for Pi camera frame (gripper should be elevated with can)
2. Check if can is visible in expected grasp position
3. Optionally: check gripper force sensors (future)
4. Return SUCCESS if confident can is grasped

---

### Modified BT: ApproachCanWithOAKD_Phase3

```xml
<BehaviorTree ID="ApproachCanWithOAKD_Phase3">
    <Sequence name="FullOptimizedApproach">
        <!-- [Phase 2 nodes: detect, extract table, generate/select poses] -->
        <!-- ... (same as Phase 2 up through navigation) ... -->
        
        <!-- Pre-compute elevator height from OAK-D detection -->
        <Action ID="ComputeElevatorHeightFromOAKD"
            can_location="{can_location}"
            can_height="0.115"
            target_elevator_height="{target_elevator_height}" />
        
        <!-- Navigate to selected pose -->
        <Action ID="NavigateToPoseAction"
            goal="{approach_goal}"
            error_code_id="{nav_error_code}" />
        
        <!-- Move elevator to computed height while/after navigating -->
        <Action ID="MoveElevatorToHeight"
            targetHeight="{target_elevator_height}" />
        
        <!-- Fine-tune with Pi camera -->
        <Action ID="FineAlignWithPiCamera"
            objectOfInterest="{objectOfInterest}"
            max_iterations="10"
            alignment_success="{aligned}" />
        
        <!-- Final reachability check -->
        <Condition ID="VerifyCanReachability"
            can_location="{can_location}"
            max_gripper_extension="{max_gripper_extension}" />
    </Sequence>
</BehaviorTree>
```

### Post-Grasp Verification (in GraspCan or ExecuteGrasp subtree)

```xml
<BehaviorTree ID="ExecuteGraspWithVerification">
    <Sequence>
        <Action ID="OpenGripper" />
        <Action ID="ExtendTowardsCan" objectOfInterest="{objectOfInterest}" />
        <Action ID="CloseGripperAroundCan" canDiameter="0.066" />
        <Action ID="StepElevatorUp" stepMeters="0.1" />
        
        <!-- Verify grasp using Pi camera -->
        <ReactiveFallback name="VerifyGrasp">
            <Condition ID="VerifyCanGrasped" objectOfInterest="{objectOfInterest}" />
            <Sequence>
                <Action ID="SaySomething" message="Grasp verification failed. Can may have been dropped." />
                <Action ID="ReportGraspFailure" />
                <AlwaysFailure />
            </Sequence>
        </ReactiveFallback>
    </Sequence>
</BehaviorTree>
```

### Testing
1. **Height pre-computation**: Verify elevator moves to correct height during navigation
2. **Fine alignment**: Test Pi camera adjustment with can slightly off-center
3. **Grasp verification**: Deliberately fail grasp, verify detection and reporting

### Success Criteria
- Elevator at correct height upon arrival (±2cm)
- Pi camera fine-tuning achieves centering in <5 iterations
- Dropped can is detected and reported
- Overall success rate >90% for well-placed cans

---

## Implementation Checklist

### Phase 1: Basic Obstacle-Aware Navigation
- [ ] Implement `ComputeCanApproachGoal` node
- [ ] Create Phase 1 BT XML (main_phase1.xml)
- [ ] Test with simple obstacle avoidance scenarios
- [ ] Verify Nav2 respects obstacles and doesn't crash into table
- [ ] Measure actual gripper extension distance and update parameters

### Phase 2: Multi-Candidate Pose Optimization
- [ ] Implement `ExtractTableBoundary` (OpenCV rectangle fitting from costmap)
- [ ] Implement `GenerateApproachCandidates` (8 poses around table edges)
- [ ] Implement `SelectBestApproachPose` (Nav2 path evaluation)
- [ ] Implement `VerifyCanReachability` condition
- [ ] Create Phase 2 BT XML (main_phase2.xml)
- [ ] Test with narrow tables (should approach short end, not long side)
- [ ] Test with obstacles blocking some approaches (should select alternate)
- [ ] Test with unreachable can (should fail early with explanation)

### Phase 3: Advanced Reachability & Refinement
- [ ] Implement `ComputeElevatorHeightFromOAKD`
- [ ] Refactor/create `FineAlignWithPiCamera` (reuse existing Pi camera logic)
- [ ] Implement `VerifyCanGrasped` condition
- [ ] Integrate into Phase 3 BT XML (main_phase3.xml)
- [ ] Test elevator height pre-computation accuracy
- [ ] Test Pi camera fine-tuning convergence
- [ ] Test post-grasp verification (deliberate failures)
- [ ] Full end-to-end testing with multiple can placements

### Final Polish
- [ ] Handle edge cases and error conditions
- [ ] Add comprehensive logging/debugging output
- [ ] Create parameter YAML files for all configurable values
- [ ] Update AvailableNodes.md with all new node documentation
- [ ] Update main.orig.xml with final working version
- [ ] Record demo videos

---

## Rollback Strategy

Each phase is independently testable. If a phase doesn't work:
1. Debug in isolation with reduced test tree
2. Roll back to previous working phase
3. Fix issues before advancing

## Key Context for Implementation (New Machine)

### Where to Start
1. **Examine existing code first**:
   - Look at `oakd_detector` package to understand detector output format (PointStamped? PoseStamped? In what frame?)
   - Review existing Nav2 nodes in `AvailableNodes.md` - particularly `NavigateToPoseAction`, `ComputePathToPose`
   - Check current BT node implementations in `can_do_challenge/src/` to understand patterns
   - Review `navigation.sim.yaml` for current Nav2 tolerances

2. **Implementation strategy**:
   - Start with Phase 1 for quick safety win
   - Each phase builds on previous, all independently testable
   - Create separate BT XML files per phase (main_phase1.xml, main_phase2.xml, main_phase3.xml)
   - Use reduced test trees to isolate individual features

3. **Critical measurements needed**:
   - Actual gripper extension distance (currently estimated 12-18 inches)
   - Gripper overhang beyond robot body circle
   - Test Nav2 goal pose tolerances with table scenarios

4. **Key insights from design discussion**:
   - Nav2 already handles robot geometry - don't duplicate this logic
   - Old `within_distance` approach was flawed (robot-to-can distance ignored table)
   - New approach: compute goal poses relative to table geometry, let Nav2 handle obstacle avoidance
   - Gripper has only 2 DOF: elevator (vertical) and extender (forward/back)
   - Can must be on flat table, within elevator height range, within gripper reach from table edge

### Integration Points
- **Nav2**: Use for all robot movement (obstacle-aware)
- **OAK-D**: 3D can detection, depth costmap, future semantic segmentation
- **Pi Camera**: Final alignment fine-tuning, post-grasp verification
- **Behavior Trees**: Modular, each phase independently testable

### Testing Philosophy
- Position robot manually to exercise specific features
- Incremental validation after each phase
- Fail early with explanatory messages (`SaySomething` node)
- Verify repeatability across multiple runs
