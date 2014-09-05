BENDER ROS NODE
===============
_Adam Wolniakowski_


Table of Contents
-----------------

1. [What is this?](#what-is-this)

2. [Installation](#installation)

3. [UR node](#ur-node)


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


UR Node
-------
### Launching
You should have a _roscore_ running in one of your terminal windows:

	`roscore`
	
Make sure the robot is powered up and initialized. Open up another
terminal window and launch _ur_node_, specifying a link to the robot
configuration file, e.g:

	`rosrun bender ur_node _config:=/path/to/UR1.xml`
	
The _ur_node_ should now be running. From now on, you can use robot services
and listen to ROS topics, either at terminal, or through other software.
There should be some notifications visible on the robot panel log, as well
as in the terminal.

To test out communication, you can try checking out the robot's state:

	`rostopic echo /ur_node/ur_state`
	
This should post a bunch of messages with robot's actual state on your screen.
Close with *Ctrl+C*.

If you want to try to move robot, try:

	`call /ur_node/ur_move_to_q "target:
		Q: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
		speed: 0.1"`

This should move robot to a safe (initial) configuration.

For more information check [Topics](#topics) and [Services](#services).


### Topics
As of now, the _ur_node_ only publishes one topic with robot's current state:


### Services


