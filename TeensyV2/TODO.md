# TODO
* Predicte battery discharge time based on current draw. Signal 'needs charging' earlier. Signal system_fault when critical.
* Implement IsUnsafe, ResetSafetyFlags for battery.
* Board1 should pick up roboclaw temp, motor1 current, motor2 current, etc.
* Battery message is not showing capacity for LIPO. Is there a field for 'is charging'?
* Revisit safety architecture (multi-board safety, inter-board signals, fault severity mapping) and re-implement cleanly.
* Implement heartbeat/watchdog from sigyn_to_sensor_v2; if comms stop, assert E-stop on this board.
* Audit and consolidate markdown docs (.md): remove redundancy, fix outdated/misleading content.