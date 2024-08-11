# S Description

The URDF for the S robot, and related files.

The launch file will also launch the joint state publisher and robot state publisher so you can use rviz2 to
visualize the robot without needing other packages.

Launch parameters:

- use_sim_time
    
    See [Clock and Time](https://design.ros2.org/articles/clock_and_time.html)

E.g.:

    ros2 launch description description.launch.py 
