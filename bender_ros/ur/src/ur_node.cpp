/**
 * A simple service server for moving UR around
 */

#include <iostream>
#include <string>
#include <rw/rw.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>

#include <ros/ros.h>
#include <ur/Q.h>
#include <ur/State.h>



using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace rw;
rwhw::URCallBackInterface URInterface;
rwhw::UniversalRobotsRTLogging URRTInterface;



/* MAIN */
int main(int argc, char* argv[])
{
	/* initialize ROS */
	ros::init(argc, argv, "ur_node");
	
	/* initialize node */
	ros::NodeHandle n("~");
	
	/* load parameters from the parameter server */
	string hostip;
	if (!n.getParam("hostip", hostip)) {
		ROS_ERROR("Parameter 'hostip' not found.");
		return -1;
	}
	
	int hostport;
	if (!n.getParam("hostport", hostport)) {
		ROS_ERROR("Parameter 'hostport' not found.");
		return -1;
	}
	
	string robotip;
	if (!n.getParam("robotip", robotip)) {
		ROS_ERROR("Parameter 'robotip' not found.");
		return -1;
	}
	
	int robotport;
	if (!n.getParam("robotport", robotport)) {
		ROS_ERROR("Parameter 'robotport' not found.");
		return -1;
	}
	
	int robotrtport;
	if (!n.getParam("robotrtport", robotrtport)) {
		ROS_ERROR("Parameter 'robotport' not found.");
		return -1;
	}
	
	string script;
	if (!n.getParam("script", script)) {
		ROS_ERROR("Parameter 'script' not found.");
		return -1;
	}
	
	int rate;
	if (!n.getParam("rate", rate)) {
		ROS_ERROR("Parameter 'rate' not found.");
		return -1;
	}
	
	/* establish robot communication */
	URRTInterface.connect(robotip, robotrtport);
	URRTInterface.start();
	URInterface.connect(robotip, robotport);
	URInterface.startCommunication(hostip, hostport, script);
	
	/* advertise services */
	/*ros::ServiceServer urGetQService = nh.advertiseService("ur_get_q", ur_get_q);
	ros::ServiceServer urStopService = nh.advertiseService("ur_stop", ur_stop);
	ros::ServiceServer urMoveQService = nh.advertiseService("ur_move_to_q", ur_move_to_q);
	ros::ServiceServer urMoveTService = nh.advertiseService("ur_move_to_t", ur_move_to_t);
	ros::ServiceServer urServoQService = nh.advertiseService("ur_servo_to_q", ur_servo_to_q);*/
	
	/* advertise topics */
	ros::Publisher statePub = n.advertise<ur::State>("state", 1000);

	ROS_INFO("UR node ready.");
	
	/* publish state topic */
	ros::Rate loop_rate(rate);
	while (ros::ok()) {
		/* get last RT data */
		rwhw::URRTData URData;
		URData = URRTInterface.getLastData();
		rw::math::Q qActual = URData.qActual;
		
		/* create message and publish*/
		ur::State msg;
				
		statePub.publish(msg);
		
		/* sleep */
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

