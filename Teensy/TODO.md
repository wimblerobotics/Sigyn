* Fix battery_module on board2 to deal with device_index.
* Device index 2 (main battery) has separate voltage divider/current sense.
* Other devices (>2) will have single I2C for both voltage and current.
* New string handling for battery_module: DIAG:BATTERY:0,40.40V83.98%,1.98A
* Remove battery monitor from board1.
* Move sensor data reporting to sensor modules and out of SerialManager.
* Enable RoboClaw runaway detection and emergency stop.
* Decode RoboClaw status messages for better diagnostics.
* Implement temperature monitoring for RoboClaw and motors.
* Is there builtin analog average filtering in Teensy?
* Capture all the reasons e-stop is active and report them.
* Capture date/time from PC and rename log files and use in time stamps for logs.
* Implement ROS side of temperature monitoring.
* Implement ROS side of VL53L0X monitoring.