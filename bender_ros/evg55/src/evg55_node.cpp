#include <iostream>
#include <string>

#include "ros/ros.h"
#include "evg55/State.h"
#include "evg55/Home.h"
#include "evg55/Open.h"
#include "evg55/Close.h"
#include "evg55/MovePosition.h"
#include "evg55/ClearError.h"

#include <evg55/gripper/EVG55.hpp>
#include <evg55/serial/RWHWSerialPort.hpp>

using namespace std;
using namespace evg55::gripper;
using namespace evg55::serial;



/* GRIPPER */
EVG55 gripper;



/*
 * EVG55 GRIPPER HOME SERVICE
 */
bool homeService(evg55::Home::Request& request, evg55::Home::Response& response) {
	ROS_INFO("Service Home called.");
	
	/* try executing command */
	try {
		gripper.home();
	} catch (GripperException& e) {
		ROS_ERROR (e.what());
	}
	
	/* get gripper state */
	response.state.header.stamp = ros::Time::now();
	response.state.position = gripper.getPosition();
	response.state.status = gripper.getStatus();
	response.state.error = gripper.getErrorCode();
	
	return true;
}



/*
 * EVG55 GRIPPER MOVE TO POSITION SERVICE
 */
bool movePositionService(evg55::MovePosition::Request& request, evg55::MovePosition::Response& response) {
	ROS_INFO("Service MovePosition called.");
	
	/* try executing command */
	try {
		gripper.move(request.position);
	} catch (GripperException& e) {
		ROS_ERROR (e.what());
	}
	
	/* get gripper state */
	response.state.header.stamp = ros::Time::now();
	response.state.position = gripper.getPosition();
	response.state.status = gripper.getStatus();
	response.state.error = gripper.getErrorCode();
	
	return true;
}



/*
 * EVG55 GRIPPER OPEN SERVICE
 */
bool openService(evg55::Open::Request& request, evg55::Open::Response& response) {
	ROS_INFO("Service Open called.");
	
	/* try executing command */
	try {
		gripper.open();
	} catch (GripperException& e) {
		ROS_ERROR (e.what());
	}
	
	/* get gripper state */
	response.state.header.stamp = ros::Time::now();
	response.state.position = gripper.getPosition();
	response.state.status = gripper.getStatus();
	response.state.error = gripper.getErrorCode();
	
	return true;
}



/*
 * EVG55 GRIPPER CLOSE SERVICE
 */
bool closeService(evg55::Close::Request& request, evg55::Close::Response& response) {
	ROS_INFO("Service Close called.");
	
	/* try executing command */
	try {
		gripper.close();
	} catch (GripperException& e) {
		ROS_ERROR (e.what());
	}
	
	/* get gripper state */
	response.state.header.stamp = ros::Time::now();
	response.state.position = gripper.getPosition();
	response.state.status = gripper.getStatus();
	response.state.error = gripper.getErrorCode();
	
	return true;
}



/*
 * EVG55 GRIPPER CLEAR ERROR
 */
bool clearErrorService(evg55::ClearError::Request& request, evg55::ClearError::Response& response) {
	ROS_INFO("Service ClearError called.");
	
	/* try executing command */
	try {
		gripper.clearError();
	} catch (GripperException& e) {
		ROS_ERROR (e.what());
	}
	
	/* get gripper state */
	response.state.header.stamp = ros::Time::now();
	response.state.position = gripper.getPosition();
	response.state.status = gripper.getStatus();
	response.state.error = gripper.getErrorCode();
	
	return true;
}



/* MAIN */
int main(int argc, char* argv[]) {
	/* initialize node */
	ros::init(argc, argv, "evg55");
	ros::NodeHandle n("~");
	
	/* load parameters from the parameter server */
	string portName;
	if (!n.getParam("port", portName)) {
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
	
	int rate;
	if (!n.getParam("rate", rate)) {
		ROS_ERROR("Parameter 'rate' not found.");
		return -1;
	}
	
	/* connect to and initialize the gripper */
	SerialPort* port = new RWHWSerialPort();
	port->open(portName, baudrate);
	gripper.connect(port, moduleid);
	
	/* advertise services */
	ros::ServiceServer homeSrv = n.advertiseService("home", homeService);
	ros::ServiceServer movePositionSrv = n.advertiseService("move_position", movePositionService);
	ros::ServiceServer openSrv = n.advertiseService("open", openService);
	ros::ServiceServer closeSrv = n.advertiseService("close", closeService);
	ros::ServiceServer clearSrv = n.advertiseService("clear_error", clearErrorService);
	
	/* advertise topics */
	ros::Publisher statePub = n.advertise<evg55::State>("state", 1000);
	
	ROS_INFO("EVG55 node ready.");

	/* publish state topic */
	ros::Rate loop_rate(rate);
	while (ros::ok()) {
		gripper.poll();
		
		/* create message and publish */
		evg55::State msg;
		
		msg.header.stamp = ros::Time::now();
		msg.position = gripper.getPosition();
		msg.status = gripper.getStatus();
		msg.error = gripper.getErrorCode();
		
		statePub.publish(msg);
		
		/* sleep */
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
