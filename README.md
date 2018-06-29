# Multiple_turtlebots

## Overview
This is launch files used to spawn multiple Turtlebots in single Gazebo simulation with AMCL and goal navigation performed.

## Prerequisites

Install ROS Indigo or Kinetic;
Install Tutrlebot packages using:
```sh
sudo apt-get install ros-<your distro>-turtlebot-gazebo
```

## How to use
To launch simulation with multiple Turtlebots:
```sh
roslaunch multiple_turtlebots multiple_turtlebot_world.launch
```

To add them AMCL algorithms and move bases:
```sh
roslaunch multiple_turtlebots amcl_wo_map.launch
```

To view navaigation and AMCL results:
```sh 
roslaunch multiple_turtlebots view_navigation.launch
```
