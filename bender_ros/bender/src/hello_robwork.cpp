/**
 * ROS+RW hello world
 */

#include <iostream>
#include <ros/ros.h>
#include <rw/rw.hpp>



USE_ROBWORK_NAMESPACE;
using namespace rw;



int main(int argc, char* argv[])
{
	ros::init(argc, argv, "hello_robwork");
	
	ros::NodeHandle nh("~");
	
	ros::Rate loop_rate(1);
	while (ros::ok()) {
		ROS_INFO("Hello ROS!");
		RW_DEBUG("Hello RobWork!");
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
