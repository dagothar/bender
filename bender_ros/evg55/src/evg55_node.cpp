#include <iostream>
#include <string>

#include "ros/ros.h"
#include "evg55/State.h"
#include "evg55/Home.h"
#include "evg55/Open.h"
#include "evg55/Close.h"
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



/*
 * EVG55 GRIPPER OPEN SERVICE
 */
bool openService(evg55::Open::Request& request, evg55::Open::Response& response) {
	ROS_INFO("Service Open called.");
	
	// TODO: implement
	
	return true;
}



/*
 * EVG55 GRIPPER CLOSE SERVICE
 */
bool closeService(evg55::Close::Request& request, evg55::Close::Response& response) {
	ROS_INFO("Service Close called.");
	
	// TODO: implement
	
	return true;
}



/* MAIN */
int main(int argc, char* argv[]) {
	// initialize node
	ros::init(argc, argv, "evg55");
	ros::NodeHandle n("~");
	
	// load parameters from the parameter server
	string port;
	if (!n.getParam("port", port)) {
		ROS_ERROR("Parameter 'port' not found.");
		return -1;
	}
	
	int baudrate;
	if (!n.getParam("baudrate", baudrate)) {
		ROS_ERROR("Parameter 'baudrate' not found.");
		return -1;
	}
	
	int moduleid;
	if (!n.getParam("moduleid", moduleid)) {
		ROS_ERROR("Parameter 'moduleid' not found.");
		return -1;
	}
	
	// connect to and initialize the gripper
	// TODO: implement
	
	// advertise services
	ros::ServiceServer homeSrv = n.advertiseService("home", homeService);
	ros::ServiceServer movePositionSrv = n.advertiseService("move_position", movePositionService);
	ros::ServiceServer openSrv = n.advertiseService("open", openService);
	ros::ServiceServer closeSrv = n.advertiseService("close", closeService);
	
	// create publisher
	ros::Publisher statePub = n.advertise<evg55::State>("state", 1000);
	
	ROS_INFO("EVG55 node ready.");

	// publish state topic
	ros::Rate loop_rate(100);
	while (ros::ok()) {		
		// create message
		evg55::State msg;
		
		// publish
		statePub.publish(msg);
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
