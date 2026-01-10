I want to redocument the whole Sigyn project. Let's just start withthe TeensyV2 package. Look at all the documents in that package. I recently moved a lot of them into the docs/obsolete directory in preparation. Read all of the existing documents, as needed, and study the TeensyV2 directory. You may need to study sigyn_to_sensor_v2 as well. I want new new set of documents.

- Replace the README.md to reflect the latest code. README should reference the new documents you are going to create.

- Place most documents in the docs directory.

- This will be an open source project, so consider conventions and best practices to generate a high quality product.

- The users will be software developers that have a modest understanding of ROS2 and Teensy.

- Try not to hallucinate. Don't necessarily believe anything that was written before. Especially, statements like "80-100Hz main control loop" should only be said if you find evidence in the code for those actual values. I mention below the two files available where you can get actual timings for messages that were actually produced. The same for, e.g., "4ms execution time per loop". A lot of number can be found as configuration values in the code.

- I want an overall Architecture document. A lot has changed since the old document. Imporant features, at least to me, include:

  - I used to use MicroRos but found the code excessively complex and hard to maintain, especially it needed customization for the Teensy device. I now use USB which, with virtual USB connections to the PC gives very high speed communication. Timestamps for ROS 2 messages are generated on the PC side in sigyn_to_sensor_v2 which may produce inaccurate time stamps but the actual running of the Sigyn robot hasn't shown this to be an issue.

  - No heap allocations in code. Part of code safety for embedded systems.

  - The use of the Module class which allows registration by instantiaing a class for a hardware device or an array of hardware devices (e.g. VL53L0X) or a new service (e.g., PerformanceMonitor).

  - Use of the SerialManager as the communication object between the multiple Teensy 4.1 boards and the ROS 2  components (via sigyn_sensor_v2) running on a PC. How incomming commands are handled quickly. Document a higher level the various messages sent and received and have a separate document about details for each specific message. Talk about how JSON is used with small keyvalues and other attempts to keep the messages small.

  - The SD class to log a lot of low-level detail. If things go wrong with the system, I can remove the SD card and look at the logs to help debug the system. The logs also provide a lot of useful statistical information such as min/max power supply current over time, temperature ranges found, device errors and warnings, performance failures and so on. Also mention, as with everything else, what ROS 2 topic and services are available, via sigyn_to_sensor_v2, to work with this subsystem. Also how the system works even if the SD card is unplugged.

  - The safety system. There is an elaborate document in the docs/safety_system directory but I haven't reviewed it yet and it's much too detailed for what I want documented in this pass. I want a description of why the safety system is needed, without made-up stories and incidents, how a device can have multiple faults pending at the same time, how faults can be self healing, what the severity levels are and the handling of the different severities, how multiple boards communicate (I'm about to implement the GPIO communication shortly) and how sigyn_to_sensor_v2 particpates in safety. The sensor data and safety system integrates with ROS 2 behavior trees (also mentioned again in a later point).

  - Test testing suite -- what it is, how to run it, generally how it works. Also describe the use of interface for dependency injection. Give a few examples of the kind of tests that might be in the suite, but indicate that the actually tests change all the time, so any mention of a test might or might not be an actual test at the moment.

  - A brief description of faults detected by each module. Also indicate that the fault system is in flux so the listed faults may be out of date at any time.

  -The  performance system -- what is measured, how each modue defines its expected performance expectation, how performance violates are detected, how it integrates with the Module architecture. Talk about the statistics reported.

  - Describe the overall architecture of the RoboClaw handler. Especially dealing with communication failure, retry, e-stop, the various fault detections such as motor over current and motor runaway. Describe here as well as in the Module discussion how long running operations are broken down, using a state machine, into multiple, quicker operations that happen over many loop() calls so that loop performance isn't violated.

  - Multiple boards and the use of compile-time constants to define what each board can do. 

  - Integration with PlatformIO. Especially describe config.h

  - Message formats. There are two documents that show actual examples of most of the messages sent by board1 and board2:
teensy_sensor_log_sample.txt (from board1) and teensy_sensor2_log_sample.txt (from board2)
Those files can also give you real numbers you can use if you want to mention performance or other features.

  - The intended integration with behavior trees. Sensor data, such as battery getting low, or unexpected obstacle detection (rings of protection implies higher levels of software should avoid getting near objects and the VL53L0X sensors could indicate those higher levels failed to avoid obstacles) can provide conditional node data. The behavior tree can use software fault to halt the motors.

Feel free to create subdocuments that contain detail that is only described at a higher level in the architecture document. 
Include a table of contents where appropriate and hyperlinks between document sections and between documents.

Ask any questions you need and, when ready, create all new documents for the TeensyV2 package.