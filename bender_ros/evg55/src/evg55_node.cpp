#include <iostream>

#include "ros/ros.h"
#include "evg55/Home.h"
#include "evg55/MovePosition.h"

using namespace std;



/*
 * EVG55 GRIPPER HOME SERVICE
 */
bool homeService(evg55::Home::Request& request, evg55::Home::Response& response) {
	ROS_INFO("Service Home called.");
	
	// TODO: implement
	
	return true;
}



/*
 * EVG55 GRIPPER MOVE TO POSITION SERVICE
 */
bool movePositionService(evg55::MovePosition::Request& request, evg55::MovePosition::Response& response) {
	ROS_INFO("Service MovePosition called.");
	
	// TODO: implement
	
	return true;
}



/* MAIN */
int main(int argc, char* argv[]) {
	// initialize node
	ros::init(argc, argv, "evg55");
	ros::NodeHandle n("~");
	
	// advertise services
	ros::ServiceServer homeSrv = n.advertiseService("home", homeService);
	ros::ServiceServer movePositionSrv = n.advertiseService("move_position", movePositionService);

	ROS_INFO("EVG55 node ready.");
	ros::spin();

	return 0;
}
