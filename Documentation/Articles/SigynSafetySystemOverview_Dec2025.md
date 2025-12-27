# Sigyn Safety System -- A General Overview
version 1, December 2025
Copyright 2025 by Michael Wimble

Sigyn is my personal, assistive robot. It's a work in progress.
Eventually Sigyn will roam the house, looking for things that need attention, taking care of things that it can and summoning help when it needs help. It will look after me and perform chores for me.
Each iteration of the hardware and software gets a bit closer to achieving those goals, but it's been a long road getting here.

The previous version of Sigyn, "Raven", had the goal of improving safety, reliability and robustness. If you cannot trust your robot, it has very little value. This paper is just a brief overview of where the robot is now in terms of intent, architecture and implementation and a description of where it will evolve next.

## Safety Issues

There are a lot of hardware components in Sigyn.

1. There is a honking big battery powering the system.
   
   There is a 10-cell LIPO battery. It must not discharge to the point where it damages the cells.
   In my robot, the charging is handled by a built-in battery management chip and an external charger and this iteration of the robot doesn't deal with charging safety. I just need to deal with making sure the robot gets to the charging station before the battery discharges too deeply.

   I do need to monitor the voltage and current to all of the electronic subsystems, though. Each subsystem has its own power supply to convert the 36 volts from the battery to the needed voltage for the subsystem. I need a recording of the historical power usage that will survive power failure so I can  debug what went wrong and why.

1. There are two powerful motors to move the robot.
   
   If the robot runs into an obstacle, it's possible for the motors to overheat or to draw too much current which could permanently damage the expensive motors. The safety system needs to detect the motors drawing too much power or running at an unexpected velocity and the motors must be very quickly shut down until the problem is resolved.

1. The motors are controlled by a motor controller.
   
   The motor controller can detect some forms of unexpected or unsafe state, but it has its limitations. And, it's possible that the motor controller itself is not responding properly to motor commands. Especially worrisome is that if the wheel encoder signals stop working correctly, the motor controller will just speed up the robot as it tries to get the robot to run at the expected speed. This puts the robot in a runaway state that will eventually cause the robot to crash into something. The safety system's biggest response to any bad situation is to first signal "e-stop", emergency stop, which immediately stops drops power to the motors. 

1. Temperatures can rise for several components, risking their safety or proper operation.
   
   The safety system should monitor all possible temperatures and attempt to put the robot in a safe state until the temperature is safe again.

1. The robot wants to implement a "rings of protection" strategy to deal with obstacles.

   Normally, the various proximity sensors will detect if the robot is getting too close to something and it will issue motor commands to avoid it. 
   The robot cannot see everything around it, it has blind spots. The robot doesn't always get notification in time to avoid hitting something--the robot has to deal with a lot of data all of the time and the responsible software can't always get notified in time or react in time to avoid hitting an obstacle.

   Sigyn wants to have two or three rings at various distances where if a sensor sees an obstacle in one of the rings it takes special action. For instance, if the robot is designed to never get closer than, say, 5 inches from an object and a sensor sees an object is 4 inches away, its first response might be to slow the robot down quite a bit in the hopes that the robot will then be able to catch up on all of the software computation to deal with the object. But if the sensor sees an object is 3 inches away it might just quickly stop the robot until I can look at what is going on and help the robot deal with the problem.

1. The ROS 2 navigational system relies on sensor data that is reasonably truthful and timely.
   
   The safety system needs to check to see that sensor data is being published at a rate needed for proper navigation. When the robot is moving, if the sensor data is late, the robot cannot figure out where it actually is at the moment nor whether there is any imminent danger. The safety system needs to also know if the sensors are all healthy and acting as expected. There needs to be a plan of action if a sensor fails or if the readings become unreliable.

There a more safety issues. This is motivation, though, for the kinds of things that Sigyn needs to be able to deal with. The safety system is all about trying to be a bit paranoid, to be constantly looking at the state of the robot from multiple different perspectives and to have a plan for dealing with expected problems or, ultimately, just go into a fail safe state and wait for help.

## A Division of Responsibilities

Sigyn uses ROS 2 as a basis for it's operation. That has a lot of implications. For Sigyn, there are limits to how fast the main computer can run the software algorithms, for how fast the robot can move and still be able to react in a timely manner. There are limits as to what kind of expected problems can be dealt with.  There are problems introduced by that fact that Linux, with its preemptive time scheduling algorithm, causes uncertainty in what computations will occur and when.

Since my hardware cannot read all of the sensors and send control signals to all of the actuators in a way that will work well if all of the software is run on my main computer, I've had to split up the hardware and software into several components. In particular, the main computer, a fast, 12-cpu, 24 thread computer with lots of fast memory, deals with all of the high level, complex algorithms. The sensors and actuators are handled by several microcontrollers that are designed to deliver guaranteed performance. And the safety system sits over it all to make sure that things don't go wrong.

## The First Division of Responsibility

My microcontrollers aren't running ROS. They used to run Micro ROS but no longer do even that. Micro ROS introduced too much complexity into the system and too much uncertainty. This article is not going to deal with that decision, this article is about the safety system. So my microcontrollers talk to the main computer over high speed USB.

I designed a custom carrier/expander board that holds a Teensy 4.1 microcontroller. The Teensy runs at about 600 MHz, has about a megabyte of RAM, lots of ROM for the code, and with my board there are a lot of signal pins and level converters to communicate with a lot of hardware devices.

There is a software package I wrote, ***sigyn_to_sensor_v2***, that provides the bridge between ROS2 and the three, custom hardware boards. I won't particularly discuss that package. It communicates with the custom boards over high speed, virtual USB ports, mostly passing JSON messages back and forth, and converts between compressed messages to and from the custom boards and the equivalent ROS2 messages.

Pretty much all of the hardware and software needs that required predictable and high speed timing happen on the custom boards and the ***sigyn_to_sensor_v2*** package links all of those boards to the main computer.

## The Second Division of Responsibility

Even a Teensy 4.1 running at 600 MHz cannot provide the needed functionality for all of the low level hardware.  ROS 2 navigation relies on high frame rates from its sensors. Your slowest navigation-related sensor constrains how fast your robot can safely move. The localization and mapping algorithms rely on the robot not moving very far between calculations that update the map and locate the robot within the map. The control loop that moves the robot towards a goal relies is limited by how fast it can detect an obstacle, plan to avoid it, and can guarantee the robot will move or stop as needed in time. 

This all only works well if the needed sensors and actuators are predictable and quick.

My closest ring of protection sensors are time of flight sensors. There are 8 of them and it takes at least 30 milliseconds to read any one of them. If you were to naively read one sensor after another, it would take about a quarter of a second to read them all. If you're trying to move a heavy robot at any speed and you know your obstacle detector can only tell you where an obstacle was a quarter of a second ago, you probably have to creep your robot at a fairly slow speed to be sure you can stop the robot in time to avoid smacking into something.

One of my custom boards deals with communicating with the motor controller, 2 temperature sensors and 8 time of flight sensors. 
A second board handles 5 voltage and current sensors for the battery and 4 power supplies and also manages two IMU sensors. 
The third board controls the gripper assembly with stepper motors and servo motors.

Why was the hardware distributed this way? Because with carefully crafted code I was able to measure and guarantee the needed performance.

## The Big Pieces of the Teensy Software

The Teensy microcontroller uses a ***setup*** and ***loop*** function, just like an Arduino device.
I have a ***Module*** class that essentially replaces those two functions.
Any piece of hardware that wants run on my board must derive from the ***Module** class, which is enough to register itself with the system. 
When ***setup*** is called, ***Module*** will call the equivalent ***setup*** function for all registered hardware modules. The same for the ***loop*** function. 

But ***Module*** wraps those calls, especially the ***loop*** function call with a timer that keeps track of how long each piece of hardware takes to perform its function. ***Module*** then compares the performance with the required performance for each piece of hardware and provides statistics at the sub-millisecond level for the minimum, maximum and average performance. Those statistics are logged and messages are sent about performance and unexpected behavior.

There is also a ***sd_logger*** module that performs efficient logging to an SD card on each board. This holds a lot of low-level messages about what is happening on each board. If nearly anything goes wrong with the low level hardware, I can alway power down the robot, pull out the SD card, mount it on another computer and look at the details about what is going on with either the hardware or software on the custom board.

There is also a ***serial_manager*** module that communicates, using compact, JSON messages between the custom board that the main computer.

### The Safety Module.

I'm going to diverge a bit from what is actually implemented in one of the branches of my code and talk about what the safety module is going to support.

Each registered module is interrogated by the ***Module*** class as to whether the hardware is safe. If not, the hardware will report that it is unsafe either at a ***warning*** level, a ***degraded*** level, an ***emergency_stop*** level or a ***system_shutdown*** level. There can be multiple hardware failures going on at the same time. The safety module keeps a list of all outstanding safety issues, including the source of the failure and the level of the failure.

Each failure be resolved either internally, such as a temporary failure like a temperature sensor now reading a safe temperature when it was previously out of range, or it can be resolved externally, such as some user interface action by myself that says the condition is now okay. An example of the later would be if the robot sensed that the wheels were turning but the localization software said the robot wasn't actually moving as expected. I could find that the robot what slipping on a towel, remove it, and then tell the robot that the condition is cleared.

If the safety system detects that someone has reported an ***emergency_stop*** or ***system_shutdown*** level of failure, the safety system relays that condition to the custom board that talks to the motor controller and it issues a hardware e-stop signal to power down the motors.

If a ***system_shutdown*** condition where detected, the error would be broadcast system wide and every component would attempt to shut itself down before the robot was forced to power down. Well, that's the plan--that code isn't implemented yet.

Also missing is code to use various fallback communication mechanisms to send me some sort of notification about failures I need to know about. That could be a tone from a speaker, a voice from a synthesizer, a text message sent to my phone, an e-mail message and so on.

## Safety failures detected

Since this is an overview, I'm going to stop going into a lot more detail about the safety system. Instead, let me give you examples of failures that are detected by my code, and maybe some that are awaiting implementation but are planned.

### Motor safety

1. Communication failure
   
   My motors are controlled by a RoboClaw controller. This gives me a lot of interesting functionality. Unfortunately, communicating with the RoboClaw can fail.
   I've written my own driver that deals with various communication faults and failures very rarely are unrecoverable any more. But if can happen.

1. Software version failure
   
   Periodically I update the firmware on the controller with the vendor's latest code. Firmware updates sometimes cause small changes needed in my driver. Sometimes they require a major rewrite, though that hasn't happened in a while. On startup, my driver verifies that it knows how to deal with the firmware in the controller.

1. Motor controller detected failure
   
   The RoboClaw can be set up to measure out of range motor currents, out of range power supply voltages, message failures and a handful of other problems. Some of these, if configured to be detected, require that the motor controller be powered off to recover.

1. 