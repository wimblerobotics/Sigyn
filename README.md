# The repository for the Sigyn robot.
## Motivation
Worldwide, the number of elderly people (65 years and older) surpassed the number of children under 5 years of age in 2019. It is likely, in my lifetime, that there will be more people needing assistance in their lives than there will people who can provide it. I don’t expect to be able to afford especially extraordinary care, nor do I expect not to need it as I age. My alternative is to create the needed technology so it will be there when I need it.
<div style="text-align: center;">
  <img src="Media/Pictures/historic-and-un-pop-projections-by-age.webp" alt="alt text">
  <p>Source: <a href="https://ourworldindata.org/population-aged-65-outnumber-children">https://ourworldindata.org/population-aged-65-outnumber-children</a></p>
</div>

***Wimble Robotics*** is a one-person effort to create an assistive robot for my personal use.


## Organization
The various directories contain:
* Designs<br/>
    Holds various design documents. E.g. Fusion 360, LightBurn.
* Documentation<br/>
    Documentation artifacts, such as notes, ***wireviz*** documentation.
* Media<br/>
    Interesting pictures, movies.
* base<br/>
    ROS 2 package containing the main lunch files, configuration, maps and such.
* description<br/>
    ROS 2 package containing the URDF and Gazebo worlds.
* rviz<br/>
    rviz2 configuration files.
* scripts<br/>
    Utility scripts.
* udev<br/>
    Various rules to be placed in ***/etc/udev/rules*** to deal with the hardware. Mostly creates symbolic links to various devices, such as the ***teensy 4.1*** and ***LIDAR*** devices.

## Running the Robot
This is a standard ROS 2 package. Build it with colcon and source ***install/setup.bash***

In one window, you need to run the MicroRos agent via
``` bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/teensy_sensor
```
Note that this relies on udev rules to create a symbolic link to the teensy 4.1 device in the ***/dev*** directory (see [00-teensy.rules](udev/00-teensy.rules)).

In another window, run the nav2 stack, which also launches the other needed components.
``` bash
ros2 launch base navigation.launch.py
```

To visualize the system, run ***rviz2***. There is a configuration file which shows things the way I like.
``` bash
rviz2 -d ~/sigyn_ws/src/Sigyn/rviz/config/config.rviz
```

I usuall begin by using begin by using the ***rviz2*** interface to set the 2D Pose Estimate, then I use the teleop module to move the robot around a bit to make sure that localization is working. Then I set a goal and watch it work.

If you don't have your own way of running teleoperation, here's what I use:
``` bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
