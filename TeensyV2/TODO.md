# TODO
* Implement IsUnsafe, ResetSafetyFlags for battery.
* Implement temperature module for both temps.
* Board1 should pick up roboclaw temp, motor1 current, motor2 current, etc.
* Battery_monitor, add list of battry names by index and publish battery name as 'location' value.
* What are all the topics being published?
* Battery message is not showing capacity for LIPO. Is there a field for 'is charging'?

# Not published?
* /sigyn/teensy_bridge/commands/config
* /sigyn/teensy_bridge/commands/estop
* ros2 topic echo /sigyn/teensy_v2/battery_diagnostics
* ros2 topic echo /sigyn/teensy_v2/battery_state
* /sigyn/teensy_v2/battery_state
* /sigyn/teensy_v2/execution_time
* /sigyn/teensy_v2/global_estop
* /sigyn/teensy_v2/loop_frequency
* /sigyn/teensy_v2/memory_usage
* /sigyn/teensy_v2/performance_diagnostics

# /sigyn/teensy_bridge/diagnostics
```code
header:
  stamp:
    sec: 1753083653
    nanosec: 673948254
  frame_id: ''
status:
- level: "\0"
  name: teensy_system_performance
  message: System performance nominal
  hardware_id: teensy_v2
  values:
  - key: loop_frequency_hz
    value: '1996.0'
```

# /sigyn/teensy_v2/safety_diagnostics 
```code
header:
  stamp:
    sec: 1753083903
    nanosec: 955097529
  frame_id: ''
status:
- level: "\0"
  name: teensy_v2_global_safety
  message: System operational
  hardware_id: ''
  values:
  - key: global_estop
    value: 'false'
  - key: active_boards
    value: '0'      #########
  - key: system_safety_level
    value: '0'
```

# /sigyn/teensy_v2/safety_status 
```code
data: System OK - 0 boards active ###########
```