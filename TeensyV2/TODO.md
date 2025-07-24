# TODO
* Implement IsUnsafe, ResetSafetyFlags for battery.
* Board1 should pick up roboclaw temp, motor1 current, motor2 current, etc.
* Battery message is not showing capacity for LIPO. Is there a field for 'is charging'?


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