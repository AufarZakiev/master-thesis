# Multiple_turtlebots

## Overview
This is launch files used to spawn multiple Turtlebots in single Gazebo simulation with AMCL and goal navigation performed.

## Prerequisites

Install ROS Indigo or Kinetic. Also, [this package](https://gitlab.com/LIRS_Projects/simulation-turtlebot-laser) is needed to get the edited for LRF Turtlebot model.

This project needs C++17 and, therefore, gcc version higher than 7.0. Please upgrade to **gcc-7** following [these instructions](https://tuxamito.com/wiki/index.php/Installing_newer_GCC_versions_in_Ubuntu). 

### Indigo:
Install Tutrlebot packages using:
```sh
sudo sudo apt-get install ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs
```
Install Eband local planner:
```sh
sudo apt-get install ros-indigo-eband-local-planner
```
Install Hector SLAM:
```sh
sudo apt-get install ros-indigo-hector-slam
```

### Kinetic:

Install Tutrlebot packages using:
```sh
sudo sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs
```
Install Eband local planner:
```sh
sudo apt-get install ros-kinetic-eband-local-planner
```
Install Hector SLAM:
```sh
sudo apt-get install ros-kinetic-hector-slam
```

## How to use

To spawn multiple Kinect Turtlebots:
```sh
roslaunch multiple_turtlebots multiple_kinect_turtlebot_world.launch
```

To launch AMCL and map_server on all robots:
```sh
roslaunch multiple_turtlebots multiple_kinect_amcl.launch
```

OR

To launch gmapping on all robots:
```sh
roslaunch multiple_turtlebots multiple_kinect_gmapping.launch
```
---
To spawn multiple LRF Turtlebots:
```sh
roslaunch multiple_turtlebots multiple_laser_turtlebot_world.launch
```

To launch hector_mapping on all robots:
```sh
roslaunch multiple_turtlebots multiple_laser_hector.launch
```
----
To view navigation in RViz:
```sh 
roslaunch multiple_turtlebots view_navigation.launch
```

## Troubleshooting
If you met complains about `TURTLEBOT_3D_SENSOR` type following:
```sh
export TURTLEBOT_3D_SENSOR=kinect
```