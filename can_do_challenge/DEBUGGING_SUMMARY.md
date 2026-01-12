# Can Do Challenge - Debugging Summary

## Issues Fixed

### 1. Type Mismatch Error - expectedCanLocation
**Error:** 
```
The creation of the tree failed because the port [expectedCanLocation] 
was initially created with type [geometry_msgs::msg::Point_<std::allocator<void> >] 
and, later type [std::string] was used somewhere else.
```

**Root Cause:** 
The `ComputePathToCanLocation` node had its `location` input port declared as `std::string` in C++ (`bt_nodes.hpp` line 201), but was being passed a `geometry_msgs::msg::Point` from the blackboard.

**Fix:**
Changed the port declaration in `bt_nodes.hpp`:
```cpp
// BEFORE:
BT::InputPort<std::string>("location"),

// AFTER:
BT::InputPort<geometry_msgs::msg::Point>("location"),
```

Updated the implementation in `bt_nodes.cpp` to extract values from the Point:
```cpp
auto location = getInput<geometry_msgs::msg::Point>("location");
goal.pose.position.x = location.value().x;
goal.pose.position.y = location.value().y - 0.5;  // 0.5m in front
```

### 2. XML Typo - ExtendTowardsCangrasp
**Error:**
```
ExtendTowardsCangrasp is not a registered node, nor a Subtree
```

**Root Cause:** 
Typo in `main.xml` line 152 - used "ExtendTowardsCangrasp" instead of "ExtendTowardsCan".

**Fix:**
Changed the action name to match the registered node:
```xml
<!-- BEFORE: -->
<Action ID="ExtendTowardsCangrasp" objectOfInterest="{objectOfInterest}"/>

<!-- AFTER: -->
<Action ID="ExtendTowardsCan" objectOfInterest="{objectOfInterest}"/>
```

### 3. Empty objectOfInterest Variable
**Error:**
```
[LoadCanLocation] Can '' not found in database
```

**Root Cause:**
The `objectOfInterest` variable was being passed to the `SetupCanChallenge` subtree but wasn't being written to the shared blackboard, so `LoadCanLocation` couldn't read it.

**Fix:**
Added `SetBlackboard` builtin action to explicitly set the value on the blackboard:
```xml
<BehaviorTree ID="SetupCanChallenge">
    <Sequence>
        <SetBlackboard output_key="objectOfInterest" value="CokeZeroCan"/>
        <Action ID="LoadCanLocation" canName="{objectOfInterest}" location="{expectedCanLocation}"/>
        ...
    </Sequence>
</BehaviorTree>
```

## Verification

After all fixes, the behavior tree executes successfully with the following sequence:

1. ✓ SaveRobotPose - Starting pose saved
2. ✓ LoadCanLocation - CokeZeroCan location loaded (0.0, 20.0, 0.75)
3. ✓ RetractGripper - Gripper retracted
4. ✓ LowerElevator - Elevator lowered
5. ✓ NavigateToCanLocation - Path computed and followed (with 3 search attempts)
6. ✓ VisuallyAcquireCan - Can detected by OAK-D and Pi Camera
7. ✓ GraspCan - Complete grasp sequence executed
8. ✓ ReturnToStart - Navigation back initiated

## Lessons Learned

1. **Type Safety**: Always ensure C++ port declarations match the types used in XML
2. **BehaviorTree.CPP Quirks**: 
   - Cannot use member access syntax like `{variable.member}` in XML
   - Subtree input ports don't automatically propagate to shared blackboard
   - Use `SetBlackboard` builtin for explicit blackboard writes
3. **Debugging Strategy**: Use systematic grep searches and file reads to trace data flow through the tree

## Next Steps

Phase 1 is now complete! The behavior tree executes with placeholder implementations. Next phases:

- **Phase 2**: Create Gazebo simulation world with table and can
- **Phase 3**: Implement real vision algorithms (HSV + Canny detection)
- **Phase 4**: Integrate Nav2 action clients for real path planning
- **Phase 5**: Connect to gripper/elevator control systems
