Here are examples of things written to the SD log file, which also includes
messages sent to sigyn_to_teensy via the USB port (once you remove the timestamps from the log).
Each log message usually is prefixed with the seconds since the Teensy 4.1 device was started, enclosed within square brackets.
Messasges that begin with a series of capital letters followed by a colon (e.g., 'DIAG:') are
typically copies of messages that are sent to the sigyn_to_teensy module.

## Compilation time.
This module captures the time is was compiled. The message is written once at startup.
```code
[0000000.335] [SdModule::SdModule] Compiled on: Jun 13 2025, 10:54:01
```

## RoboClaw firmware version.
A check is made to see if the RoboClaw firmware version is compatible with this module. This message is written whenever a new connection to the RoboClaw is made. 
This happens at startup and whenever the connection is dropped and reformed whenever an error in
communication is detected. Note the trailing carriage return in the version string.
```code
[0000000.335] DIAG:INFO [RoboClawModule::GetVersion] version match, found: \'USB Roboclaw 2x15a v4.2.8
\n\'
```

Also, if the firmware check passes, you'll see this.
```code
[0000000.335] DIAG:INFO [RoboClawModule::Loop] Version check passed
```

## Battery
The voltage of the main battery followed by the estimated charge percentage. 
This is an average and message is sent once per second.
```code
0000016.108] BATTERY:42.14V, 100.00%
```

## Module statistics
Written once per second. It begins with how many times the TModule::Loop() method was executed in the
last give period in milliseconds, which should be close to 1 second. The Loop() method iterates over all the register TModule instances so all of those
modules (e.g., Battery, RoboClaw, SD Car) also got their loop() (note: lowercase loop here) executed.

This is followed by statistics for each registered module, showing the minimum, maximum and average
execution time, in milliseconds, for the given module over the last sampling period.

(again, it should be close to one second).
```code
[0000016.188] DIAG:{"loops":219652,"Ms":1007.1,"mdls":[TModuleStats:{"n":"MBAT","MnMxAv":[0.0,0.0,0.0]},TModuleStats:{"n":"Robo","MnMxAv":[0.0,33.6,0.0]},TModuleStats:{"n":"Sd","MnMxAv":[0.0,5.2,0.0]}]}
```

The message body after the 'DIAG:' prefix is in JSON format, like:
```code
{
    "loops": 219652,
    "Ms": 1007.1,
    "mdls": [TModuleStats: {
            "n": "MBAT",
            "MnMxAv": [
                0.0,
                0.0,
                0.0
            ]
        },TModuleStats: {
            "n": "Robo",
            "MnMxAv": [
                0.0,
                33.6,
                0.0
            ]
        },TModuleStats: {
            "n": "Sd",
            "MnMxAv": [
                0.0,
                5.2,
                0.0
            ]
        }
    ]
}
```

The `mdls` introduces a list of modules.
Each module is represented by a name shown for the `n` value.
The `MnMxAv` shows the list of minimum, maximum and average execution time of each `loop()` method caoo
for the named module over the last `Ms` period.

## Odometry
Afer the 'ODOM:' prefix comes the pose values for x and y (`px`, `py`) then the orientation quaternion for x, y, z and w values (`ox`, `oy`, `oz`, `ow`), then the linear velocity for x and y (`vx`, `vy`) and the angular z velocity (`vz`). To prevent flooding of the SD card log, only a fraction of the odometry messages are logged, as they are produced tens of times per second.
```code
[0000016.536] ODOM:px=0.00,py=0.00,ox=0.00,oy=0.00,oz=0.00,ow=1.00,vx=0.00,vy=0.00,wz=0.00
```

## RoboClaw state
This shows the battery voltages for the logic and motor, then the encoder values for the left (M1) and right (M2) wheels, then the current in amps for the left and right motors, then the motor speeds in quadrature pulses per second for the left and right motors, then the 32 bit error code from the RoboClaw module.
```code
[0000017.096] ROBOCLAW:{"LogicVoltage":4.0,"MainVoltage":27.0,"Encoder_Left":0,"Encoder_Right":0,"LeftMotorCurrent":-0.760,"RightMotorCurrent":-0.660,"LeftMotorSpeed":0,"RightMotorSpeed":0,"Error":0}
```

## SD card directory
A request to get the directory of the SD card results in a tab-separated list of the file names, each file name being followed by the size of the file (in bytes). The last file name in the list doesn't have the file size and is the current log file being generated. This list is computed once at startup and the cached value is used as the result when requested.
```code
[0000236.100] DIAG:SDIR:.Spotlight-V100,0\tLOG00001.TXT,622209\tLOG00002.TXT,0\tLOG00003.TXT,0\tLOG00004.TXT,163489\t.Trashes,0\t.fseventsd,0\tLOG00005.TXT
```

