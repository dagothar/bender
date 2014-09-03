/**
 * A simple publisher for listening to UR state
 */

#include <iostream>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <rw/rw.hpp>
#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <bender/Q.h>



USE_ROBWORK_NAMESPACE;
using namespace rw;



rwhw::UniversalRobotsRTLogging URRTInterface;



int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ur_simple_state");
	
	// configure robot communication
	std::string ip = "192.168.2.4";
	unsigned rtport = 30003;
	
	URRTInterface.connect(ip, rtport);
	URRTInterface.start();
	
	// configure ROS node
	ros::NodeHandle nh("~");
	
	// advertise topic
	ros::Publisher pub = nh.advertise<bender::Q>("ur_q", 1000);
	
	// wait for calls
	ROS_INFO("Started ur_simple_state node.");
	ros::Rate loop_rate(10);
	while (ros::ok()) {
		// get robot state
		rwhw::URRTData URData;
		URData = URRTInterface.getLastData();
		rw::math::Q q = URData.qActual;
		
		// create message
		bender::Q msg;
		if (q.size() == 6) {
			for (int i = 0; i < 6; ++i) {
				msg.Q.data()[i] = q[i];
			}
		}
		
		pub.publish(msg);
		
		ros::spinOnce();
		
		loop_rate.sleep();
	}
	
	return 0;
}

