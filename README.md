# Multiple_turtlebots

## Overview
This is launch files used to spawn multiple Turtlebots in single Gazebo simulation with AMCL and goal navigation performed.

## Prerequisites

Install ROS Indigo;
Install Tutrlebot packages using:
```sh
sudo sudo apt-get install ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs
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

## Troubleshooting
If you met complains about `TURTLEBOT_3D_SENSOR` type following:
```sh
export TURTLEBOT_3D_SENSOR=kinect
```