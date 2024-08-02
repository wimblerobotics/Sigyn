# **Sigyn** Requirements
The previous generation of personal assistive robot, ***Raven***, was focused on robustness and reliability issues. The focus for the current generation of personal assistive robot, ***Sigyn***, is the demonstration of autonomous charging and the beginning of the ability to autonomously move to a designated location, recognize and fetch an object, and move it to another location. 

The new physical additions for this generation of robot, Sigyn, is an arm-based gripper and a charging port along with a mating charging station. Further, the robot has transformed from a cubic form to a cylindrical form
## Design goals for this iteration
Raven was a box shaped robot, which proved difficult to maneuver in the tight passageways of my house. Sigyn is cylindrical in shape, which should fit better in the passageways where it needs to go and will be less destructive if it fails to stay in the desired lanes. Sigyn should, therefore, better demonstrate maneuvering through the house and inflict less damage if it should bump into objects.
## Task Goals for this Iteration
### Object Recognition Goals
* Recognize the charging station. Especially recognize in fine detail the required position to mate the robot to the charging station.
* Recognize the refrigerator door handle.
* Recognize an object on the shelf in the refrigerator.
* Recognize the open and closed positions of the refrigerator door.
### Navigation goals
* Move to a pose in front of the refrigerator which would allow the beginning of a movement sequence to open the refrigerator door.
* Move to a pose in front of the charging stating that would allow the beginning of a docking maneuver.
* Demonstrate dynamic avoidance of obstacles while moving.
* Demonstrate consistent localization when moving from a fixed position to and from the refrigerator.
### Charging station goals
* Complete a docking maneuver.
* Complete an undocking maneuver.
* Complete a charging cycle.
### Behavior goals
* Recognize the need to charge the robot.
* Add a reactive component to interrupt any ongoing task and proceed to the docking station.
* Orient the robot in front of the docking station for a docking maneuver.
* Perform a docking maneuver with retry.
* Perform an undocking maneuver with retry.
* Signal distress when unable to complete a behavior.
### Arm goals
* Open the refrigerator door.
* Close the refrigerator door.
* Move the arm and gripper to touch a recognized object on the shelf in the refrigerator.
* Grab and extract an object from the shelf of the refrigerator.

