# TeensyV2 Serial Message Protocol

This document summarizes the serial messaging used between TeensyV2 boards and the ROS2 bridge.

## Format
- Flat, parseable text lines per message
- General pattern:
  - `TYPE:key1=val1,key2=val2,...`
- Some messages embed compact JSON to carry many fields (e.g., RoboClaw status)

## Message Types
- `ODOM`: High-rate odometry (≥70 Hz)
- `ROBOCLAW`: Motor controller diagnostics (JSON payload)
- `BATT`: Battery status
- `IMU`: Orientation data (quaternion)
- `TEMP`: Temperature sensor reading
- `ESTOP`: Emergency stop events
- `PERF`: Performance statistics (JSON payload)

## Examples
```
BATT:id=0,v=39.8,p=0.82,c=1.2,s=OK
IMU:id=0,qx=0.1,qy=0.2,qz=0.3,qw=0.9
ODOM:px=0.123,py=0.456,ox=0.0,oy=0.0,oz=0.707,ow=0.707,vx=0.5,vy=0.0,wz=0.1
ROBOCLAW:{"LogicVoltage":4.1,"MainVoltage":27.0,"Encoder_Left":12345}
ESTOP:src=safety,state=active,reason=overcurrent
TEMP:id=1,temp=45.2,status=ERROR
```

## QoS & Rates
- Keep ODOM compact to sustain ≥70 Hz
- Status/diagnostics at lower rates (3–30 Hz)

## Error Handling
- SerialManager retries on transient errors
- PERF includes counters to spot drops
- Safety messages are prioritized

## Extending
- Prefer adding new TYPEs over overloading existing fields
- Use JSON only when necessary (large structured payloads)
- Document new types here and update the ROS2 bridge parser
