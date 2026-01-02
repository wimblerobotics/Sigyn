New phase of project. I am now going to want to begin a testing system.
The first phase is to generate tests for the TeensyV2 system alone.
TeensyV2 is run on a custom board I created, containing a Teensy 4.1 processor. It connects to several sensors and the RoboClaw motor controller.
I think I want one group of tests to:
* Compile to be run on the PC so I can use some testing framework, probably Google's framework with C++ code. That means there will be a CMakeLists.txt.
* This also means I want to use dependency injection, which means that for all of the hardware components, like everything in the battery, bno055, motors, roboclaw, storage and sensors directories I will want to create an interface and then a derived class for the real hardware vs. the mocked hardware.
* This must still be compileable using platform IO, but I also want to create a suite of tests that runs on a PC and tests the components that aren't hardware drivers, especially the safety system and the roboclaw_monitor.
* I want to start simply. Let's see if we can create the framework, add in the dependency on the google testing framework (e.g. add it to package.xml and I can use rosdep to install the required software), and then lets try creating test for something simple, like for sensors/temperatur_monitor.
* I will need to add code to safety_coordinator, but the new safety tests will need to allow me to set the temperature for any simulated device to check for out-of-range safety detection, and also to set the temperature over a range of times to check for predicted temperature overun.

Comment on my design so we can come to a consensus as to whether this will work well to test if safety_coordinator can detect all the tests that I want and set/reset the captured array of faults, even in combination with multiple faults happening at the same time, and testing of safety features in roboclaw_monitor, such as motor runaway, motor over current and such. Once we come to a consensus, we can discuss how to start this project, first simply, and then adding in a fairly comprehensive set of tests.

So, ask questions as needed and tell me if this is a good plan or there is a better way to do this.