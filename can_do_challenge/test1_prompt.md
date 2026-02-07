test1_prompt
Let's go on to the next task.
Let's focus on the  can_do_challenge package.
Ultimately the purpose of this challenge is to use a behavior tree to drive my robot, Sigyn, to drive to a location containing a can, grab it, and return the can to starting point. But for now, we are trying to accomplish just one part of that which is focused on using the same can recognizer the just got working in youlo_oakd_test.

Unfortunately, a large amount of effort has gone into attempting to get the object recognizer code in can_do_challenge to work and it all failed. I want to mostly toss that code and incoporate the new code.

I want to do this in pieces.

- Look at launch/step3_visual_acquire_launch.py as a model and create a new launch/oakd_detection_test1.laynch.py. It should use the BT at bt_xml/oakd_detection_test1.xml.
- The code for the nodes CanDetectedByOAKD and CanWithinReach should be instrumented, for now, to show trace output from the object detector, showing whether a can was detected and the  pose of the detection in base_link coordinates.
- Replace all the code that used to get executed by step3_visual_acquire_launch.py to do object detection with code from the yolo_oakd_test to perform the actual detection. You will have to use the new topic names. What will probably be tricky is the detector may have to run on its own thread in that a problem we had in the past is we want the object detector to just run flat out as fast as it can, publishing detections and annotated images without interference. That means the detector has to do it's own spinning to give ros2 time to do it's own thing. Meanwhile the main code, the one running the BT needs to be doing it's own thing and not blocking nor being blocked by the object detector.

What I want is to be able to launch the code and I will manually move the robot around, with and without a can to be seen, and I want to see the trace output of CanDetectedByOAKD and CanWithinReach in order to see that the detection boolean, detection position and annotated image are showing what I expect and running at 10 frames per second or faster, with no big pauses. 

The main failures in the past were:
- The BT kept blocking or getting blocked by the object detection code.
- The BT did not run consistenty at 100 ticks/second. Sometimes a node would cause the BT to stall for 0.1 to 1 seconds, which would be disasterous when I'm trying to move the robot.