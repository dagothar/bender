/**
 * A simple service server for moving UR around
 */

#include <iostream>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <rw/rw.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <bender/Q.h>
#include <bender/URMoveToQ.h>



USE_ROBWORK_NAMESPACE;
using namespace rw;



rwhw::URCallBackInterface URInterface;



bool ur_move_to_q(bender::URMoveToQ::Request& req, bender::URMoveToQ::Response& res)
{
	// de-scramble Q message
	rw::math::Q q(6, req.target.Q.data());
	
	// log message
	std::stringstream sstr;
	sstr << q;
	ROS_INFO("Requested URMoveToQ: Q=%s with speed %s", sstr.str().c_str(), "a");
	
	// perform service
	URInterface.moveQ(q, 0.1);
	
	return true;
}



int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ur_simple_service");
	
	// configure robot communication
	std::string ip = "192.168.2.4";
	unsigned urport = 30002;
	unsigned port = 30001;
	std::string script = "urscript1.ur";
	
	URInterface.connect(ip, urport);
	URInterface.startInterface(port, script);
	
	// configure ROS node
	ros::NodeHandle nh("~");
	
	// advertise services
	ros::ServiceServer service = nh.advertiseService("ur_move_to_q", ur_move_to_q);
	
	// wait for calls
	ROS_INFO("Started ur_simple_service node.");
	ros::spin();
	
	return 0;
}

