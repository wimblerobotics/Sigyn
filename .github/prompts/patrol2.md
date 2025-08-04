

I have a robot which:
* Is primarily a cylindrical profile with a radius of about 0.44m, a height of 0.3m, and a mass of 22kg.
* Has a 2-wheel differential drive and a rear caster wheel.
* There are a few temperature sensors at various places on the robot, at least one of which can measure ambient temperature.
* Has 2 IMU sensors, at opposing ends of the robot, one mounted upside down relative to the other so that the rate gyroscopes somewhat cancel each other’s static drift.
* Has one LIDAR sensor mounted at the top front of the robot with a viewing angle of about 270 degrees. I could add another LIDAR near the top of a 4-foot pole which is attached to the top, back of the robot.
* Has a gripper assembly consisting of the 4-foot pole with a band which can move, acting as an elevator, an 18-inch  arm attached at one end of the 18-inch arm at a right angle to the elevator, the arm can extend an additional 18 inches, a two-fingered, parallel gripper that can open and shut at the end of the extendible arm.
* There is a 2K camera mounted a bit behind and above the extendible gripper for helping position the gripper for grasping an object and determining when an object has been grasped.
* Encoders are attached to each wheel to provide wheel odometry. Of course, there is some amount of slippage for each wheel when the robot moves.
* Atop the 4-foot pole is another camera, an OAK-D which can generate RGB and point cloud images. It can also run AI models that, e.g., produce segmentation of the image, or do object recognition.
* The robot is in my home. The home is aobut 20 meters on a side with some rooms and areas that the robot will n ot be able to get to, such as the garage, the atrium, and in some bedrooms there are obstackles that keep the robot from moving very far into them.
* The house contents change.  People move about. Furniture positions change. A room that could be intered one time may find it's entry block later. A robot might enter a room and then find it's exit is block for some time, 10s of minutes or hours.
* The robot's goal is to patrol the house. It has onboard cameras and other sensors and it wants to sense as much open space as it can. Eventually I will add logic to look for interesting changes in the house, like a door left open, or a window left open causing the house to cool, or unusual noise, but that is later.
* Has a PC running Linux 24.04 and also running the ROS2 framework/jazzy distribution. The navigation2 stack currently runs and uses the IMUs, odometry, and LIDAR topics for computing SLAM.  The robot is capable of generating and following a plan to move the robot to a given pose in the house. There is some amount of chance of failure when trying to execute the plan, so the goal is abandoned somewhat frequently when the global planner thinks it can’t find a plan forward. Usually, if you request to move the robot again to the same goal, it will succeed. The PC can connect to the local network via Wi-Fi. There is an additional desktop, Linux computer available for additional computing and can be reached via Wi-Fi.
* The robot can run for several hours on a single battery charge, probably 6 or more hours at a time.
* There currently isn’t any automatic way to recharge the battery. I have to plug in the charger manually, but there will be a way for the robot to find a charging station and attach itself. It will probably take one to two hours to recharge.
* I have a static map which was manually drawn from the blueprints for the house. It does not include furniture except for an area for the kitchen table. The house has narrow doorways for the master bedroom, the music room, the guest bedroom, the computer room, and the hobby room. The kitchen and living room have a hallway connecting them. There is a grand hallway from the master bedroom to the guest bathroom to the music room to the guest bedroom, and across from the guest bedroom is the computer room. The living room exits to both the kitchen and the hobby room. The grand hallway is 1 meter wide. The doorways are a combination of 2 feet wide and 30 inches wide, making it a bit hard for the robot to move through.
* I have created a database of where the walls are in the house, giving the starting x and y position, the length, and width of the wall. The length and width can indicate if the wall is east-west or north-south oriented. This database was intended to be available if I want to write some code that would interpret the LIDAR readings to attempt to find wall locations that the LIDAR can see and make a guess as to which room the robot is in and the position in the room. This would be used to override SLAM localization as sometimes the robot will get its location wrong using the SLAM toolbox. The AMCL node in ROS2 is prone to delocalization now and then when roaming along one or two places in the house.
* The robot is somewhat wide and has to pass though narrow hallways and doorways. Moving through a door is particularly problematic as previous attempts, even by the ros2 navigation stack, often cause the robot to bump into the edges of door frames, probably from not properly dealing with the momentum of the robot and properly keeping the robot centered.
* When traversing narrow hallways and going through doorways, the robot probably should keep itself centered. But in a room, the robot wants to patrol around the edge of the room, probably keepin g at least one meter away from the clossest wall, as sensors don't work as well as close distances. The distance  for wall following in a large space should be configurable.
* My house does not have a situation where there are two doorways opposite each other, at least not unless there are a few meters of space between them.
* My main sensors are wheel odometry and LIDAR. I will be adding SONAR and time of flight sensors to give better detection at multiple floor heights, so I'd want to be able to drop in code to deal with them later. I will also be adding 3D depth sensors, such as the OAK-D camera. I may come back later and ask how we can inorporate segmentation analysis into this code, but I don't need it today. Likewise, I might want to explore whether an octomap would be useful, but not today.

What I want to do is write some sort of code package to cause the robot to roam the house, particularly when we are out of the house. 

It should be able to roam the house, detect doorways, and avoid dynamic obstacles. 

It should be able to detect when it is in a room and then patrol the edge of the room, keeping at least one meter away from the closest wall. 

It should be able to detect when it is in a hallway and then follow the hallway, centering itself in the hallway. 

It should be able to detect when it is near a doorway and then go through the doorway, keeping itself centered. 

For most or all of the doors, because of the narrow passageways, the robot probably needs to center itself in front of the doorway, make a sharp turn to face the doorway if it is not already facing the doorway, and then carefully go through the narrow passageway.


Ask questions if you don't understand or think I may have left something out that would help you solve this problem. Otherwise, go ahead and create a new ros2 python package called perimiter_roamer_v2. Be sure to create all the necessary related files, such as resource, setup.py, etc. Create a yaml configuration file in the config directory. Create a launch file in the launch directory. 
