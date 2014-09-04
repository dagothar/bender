BENDER ROS NODE
===============

What is this?
-------------
This directory contains code for bender ROS package. Package serves to
facilitate easy communication with Universal Robots installed on the
laboratory stand, and with associated hardware:

* UR robots,
* EVG55 electrical Schunk grippers,
* Bumblebee stereo camera,
* TrakStar 3D guidance hardware,
* ...

Installation
------------
### Dependencies
Requires following software:

* ROS (indigo),
* RobWork, RobWorkStudio, RobWorkHardware, RobWorkSim,
* TODO

### Installation
Installation is done in following steps:

1. Compile and install RobWork

2. Install and setup ROS

3. Create catkin workspace in your home directory (e.g. ~/catkin_ws)

4. Make a symbolic link to bender package in GIT repository in your
catkin workspace source directory (e.g. ln -s ~/bender/bender_ros/bender ~/catkin_ws/src/bender)

5. Run catkin_make in your catkin workspace root directory
