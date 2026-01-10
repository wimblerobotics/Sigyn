# Reactive NavigateToPose Implementation

## Overview

The improved `NavigateToPose` behavior tree node is designed to be fully reactive and handle interruptions gracefully. Unlike the original blocking implementation, this version:

1. **Never blocks** the behavior tree execution
2. **Handles interruptions** without losing navigation state  
3. **Can resume interrupted goals** after handling high-priority tasks
4. **Properly manages action lifecycle** with clear state transitions

## Key Features

### 1. Non-Blocking Action Client
- Uses non-blocking calls with minimal timeouts (1ms) for goal checking
- Action server availability is checked without blocking
- Goals are sent asynchronously and monitored through state machine

### 2. Interruption Handling
- When `onHalted()` is called, the node marks itself as interrupted
- Current goal parameters are preserved for potential resume
- Provides `goal_interrupted` output port to inform parent behavior tree

### 3. State Machine
The node uses an internal state machine with these states:
- `IDLE`: Ready to send new goal
- `SENDING_GOAL`: Goal sent, waiting for acceptance
- `GOAL_ACTIVE`: Goal accepted and executing
- `GOAL_COMPLETED`: Goal finished successfully
- `GOAL_FAILED`: Goal failed or was cancelled
- `GOAL_INTERRUPTED`: Goal was halted by behavior tree

### 4. Goal Resume Capability  
- Detects when the same goal is requested after interruption
- Avoids sending duplicate goals for the same target
- Handles the case where navigation needs to be repeated after handling interruptions

## Usage Patterns

### Basic Navigation
```xml
<NavigateToPose target_pose="{waypoint}" 
                timeout="120.0" 
                goal_interrupted="{nav_interrupted}"/>
```

### Handling Interruptions with Fallback
```xml
<ReactiveFallback>
  <!-- High priority: Handle low battery -->
  <Sequence name="BatteryEmergency">
    <CheckBatteryState low_battery_threshold="15.0"/>
    <NavigateToPose target_pose="{charging_station}" timeout="60.0"/>
    <!-- Wait for charging completion -->
  </Sequence>
  
  <!-- Lower priority: Continue main navigation -->
  <NavigateToPose target_pose="{main_target}" 
                  timeout="300.0" 
                  goal_interrupted="{nav_interrupted}"/>
</ReactiveFallback>
```

### Resume Interrupted Navigation
```xml
<Sequence>
  <!-- Handle the interruption (e.g., charging) -->
  <HandleEmergency/>
  
  <!-- Check if navigation was interrupted and resume if needed -->
  <Fallback>
    <Sequence name="ResumeIfInterrupted">
      <!-- Only runs if goal_interrupted is true -->
      <Condition port="{nav_interrupted}" expected="true"/>
      <NavigateToPose target_pose="{original_target}" timeout="300.0"/>
    </Sequence>
    <AlwaysSuccess/> <!-- If not interrupted, continue normally -->
  </Fallback>
</Sequence>
```

## State Transitions

```
IDLE → SENDING_GOAL → GOAL_ACTIVE → GOAL_COMPLETED ✓
                   ↘               ↘  
                    GOAL_FAILED ✗   GOAL_INTERRUPTED (on halt)
                                    ↓
                                   IDLE (on resume)
```

## Thread Safety

- Uses `std::atomic` for `result_received_` and `navigation_result_` 
- Action callbacks can safely set these from ROS executor threads
- State machine variables are only modified from behavior tree thread

## Timeout Handling

- Tracks goal start time and checks timeout in `onRunning()`
- Cancels goal and returns FAILURE when timeout exceeded
- Timeout is configurable per goal via input port

## Benefits Over Original Implementation

1. **Reactivity**: Behavior tree can respond to other conditions while navigation is active
2. **Robustness**: Handles interruptions gracefully without losing context
3. **Efficiency**: No blocking waits that could freeze the behavior tree
4. **Flexibility**: Supports complex navigation patterns with fallbacks and retries
5. **Debuggability**: Clear state machine makes it easier to understand current status

## Example Use Cases

- **Emergency Response**: Stop navigation to handle low battery, then resume
- **Dynamic Re-planning**: Change navigation target mid-flight
- **Multi-Priority Tasks**: Balance navigation with sensor monitoring, charging, etc.
- **Recovery Behaviors**: Retry navigation after clearing obstacles or recovering localization

This implementation makes NavigateToPose a true "reactive" behavior tree node that plays well with complex, hierarchical behavior trees while maintaining robust action client management.
