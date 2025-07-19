===== TeensyV2 Board 2 (Sensor Board) Starting =====
INIT:serial_ready=true
Initializing modules...
INIT:starting_module_setup
PERF_INIT:monitor_started=true
INIT:module=PerformanceMonitor,setup_time_ms=0
INIT:module=BatteryMonitor,setup_time_ms=0
INIT:module_setup_complete
===== Board 2 Initialization Complete =====
Target loop frequency: 50Hz
Battery monitoring enabled
Ready for sensor data collection
PERF:{"loop_count":1,"freq":0.2,"modules":[{"name":"PerformanceMonitor","min":0.0,"max":0.0,"avg":0.0},{"name":"BatteryMonitor","min":0.0,"max":0.0,"avg":0.0}]}
SAFETY: Battery critical condition detected!
  Voltage: 0.00 V
  Current: 0.00 A
SAFETY_CRITICAL:board=2,critical=true,reason=battery,voltage=0.00,current=0.00
SAFETY: Battery condition returned to normal
SAFETY_CRITICAL:board=2,critical=false,reason=battery_recovered
PERF_STATS:freq=1996.0,loops=1990,mod_viol=0,freq_viol=0,unsafe=false
PERF:{"loop_count":1989,"freq":397.1,"modules":[{"name":"PerformanceMonitor","min":0.0,"max":0.0,"avg":0.0},{"name":"BatteryMonitor","min":0.0,"max":0.3,"avg":0.0}]}
BATT:id=0,v=42.37,c=1.068,pct=1.00,state=DISCHARGING


Eventually only the following is produced:
===== TeensyV2 Board 2 (Sensor Board) Starting =====
INIT:serial_ready=true
Initializing modules...
INIT:starting_module_setup
PERF_INIT:monitor_started=true
INIT:module=PerformanceMonitor,setup_time_ms=0
INIT:module=BatteryMonitor,setup_time_ms=0
INIT:module_setup_complete
===== Board 2 Initialization Complete =====
Target loop frequency: 50Hz
Battery monitoring enabled
Ready for sensor data collection
PERF:{"loop_count":1,"freq":0.2,"modules":[{"name":"PerformanceMonitor","min":0.0,"max":0.0,"avg":0.0},{"name":"BatteryMonitor","min":0.0,"max":0.0,"avg":0.0}]}
SAFETY: Battery critical condition detected!
  Voltage: 0.00 V
  Current: 0.00 A
SAFETY_CRITICAL:board=2,critical=true,reason=battery,voltage=0.00,current=0.00
SAFETY: Battery condition returned to normal
SAFETY_CRITICAL:board=2,critical=false,reason=battery_recovered
PERF_STATS:freq=1996.0,loops=1990,mod_viol=0,freq_viol=0,unsafe=false
PERF:{"loop_count":1989,"freq":397.1,"modules":[{"name":"PerformanceMonitor","min":0.0,"max":0.0,"avg":0.0},{"name":"BatteryMonitor","min":0.0,"max":0.3,"avg":0.0}]}
BATT:id=0,v=42.37,c=1.068,pct=1.00,state=DISCHARGING



os@sigyn7900:~$ ros2 topic list
/parameter_events
/rosout
/sigyn/teensy_bridge/battery/status
/sigyn/teensy_bridge/commands/config
/sigyn/teensy_bridge/commands/estop
/sigyn/teensy_bridge/diagnostics
/sigyn/teensy_bridge/safety/estop_status
/sigyn/teensy_v2/battery_current
/sigyn/teensy_v2/battery_diagnostics
/sigyn/teensy_v2/battery_state
/sigyn/teensy_v2/battery_voltage
/sigyn/teensy_v2/execution_time
/sigyn/teensy_v2/global_estop
/sigyn/teensy_v2/loop_frequency
/sigyn/teensy_v2/memory_usage
/sigyn/teensy_v2/performance_diagnostics
/sigyn/teensy_v2/safety_diagnostics
/sigyn/teensy_v2/safety_status
ros@sigyn7900:~$ ros2 topic echo /sigyn/teensy_v2/battery_state 
^Cros@sigyn7900:~$ rqt
^C[INFO] [1752883285.325165472] [rclcpp]: signal_handler(signum=2)
Traceback (most recent call last):
  File "/opt/ros/jazzy/lib/python3.12/site-packages/rqt_topic/topic_widget.py", line 123, in refresh_topics
    topic_list = self._node.get_topic_names_and_types()
                 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/node.py", line 2061, in get_topic_names_and_types
    with self.handle:
rclpy._rclpy_pybind11.InvalidHandle: cannot use Destroyable because destruction was requested
ros@sigyn7900:~$ cat /dev/teensy_sensor2 
Board2 Status:
  Loop frequency: 1996.0 Hz
  Execution time: 1 us
  Battery voltage: 42.34 V
  Battery current: 0.96 A
  Battery state: 0
  Sensor health: OK
  Free memory: 447952 bytes
Board2 Status:
  Loop frequency: 1996.0 Hz
  Execution time: 1 us
  Battery voltage: 42.39 V
  Battery current: 1.01 A
  Battery state: 0
  Sensor health: OK
  Free memory: 447952 bytes
Board2 Status:
  Loop frequency: 1992.0 Hz
  Execution time: 1 us
  Battery voltage: 42.37 V
  Battery current: 0.99 A
  Battery state: 0
  Sensor health: OK
  Free memory: 447952 bytes
Board2 Status:
  Loop frequency: 1996.0 Hz
  Execution time: 1 us
  Battery voltage: 42.42 V
  Battery current: 1.04 A
  Battery state: 0
  Sensor health: OK
  Free memory: 447952 bytes


Board2 Status:
  Loop frequency: 1996.0 Hz
  Execution time: 1 us
  Battery voltage: 42.35 V
  Battery current: 1.04 A
  Battery state: 0
  Sensor health: OK
  Free memory: 447952 bytes

# TODO
* Board2 Status, don't make it multiline.
* Board2 Status, battery tests belong in battery module for IsUnsafe.
* Implement IsUnsafe, ResetSafetyFlags for battery.
* Implement temperature module for both temps.
* Board1 should pick up roboclaw temp, motor1 current, motor2 current, etc.
* Getting PERF_STATS and PERF message, PERF can probably go away.
* Battery_monitor, add list of battry names by index and publish battery name instead of index. Needs parsing change on PC side.
* WHy is loop count < freq?
* Run loop as fast as possible, not constrained to some lesser speed. Each module will decide when to react and send data.
* What are all the topics being published?
* Need battery topic per battery kind. Need to get list of batteries from teensy along with list of other sensors.
