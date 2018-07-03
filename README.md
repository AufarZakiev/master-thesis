# Multiple_turtlebots

## Overview
This is launch files used to spawn multiple Turtlebots in single Gazebo simulation with AMCL and goal navigation performed.

## Prerequisites

Install ROS Indigo or Kinetic.

### Indigo:
Install Tutrlebot packages using:
```sh
sudo sudo apt-get install ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs
```
Install Eband local planner:
```sh
sudo apt-get install ros-indigo-eband-local-planner
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

## How to use

To spawn multiple Kinect Turtlebots:
```sh
roslaunch multiple_turtlebots multiple_kinect_turtlebot_world.launch
```

To launch AMCL and map_server on all robots:
```sh
roslaunch multiple_turtlebots multiple_kinect_amcl.launch
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