/**
 * A simple service server for moving UR around
 */

#include <iostream>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <rw/rw.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#include <bender/Q.h>
#include <bender/URStop.h>
#include <bender/URMoveToQ.h>
#include <bender/URMoveToT.h>
#include <bender/URServoToQ.h>



USE_ROBWORK_NAMESPACE;
using namespace rw;
using namespace boost::program_options;
namespace po = boost::program_options;



rwhw::URCallBackInterface URInterface;



/**
 * ur_stop service
 */
bool ur_stop(bender::URStop::Request& req, bender::URStop::Response& res)
{
	ROS_INFO("Requested URStop");
	
	// perform service
	URInterface.stopRobot();
	
	return true;
}



/**
 * ur_move_to_q service
 */
bool ur_move_to_q(bender::URMoveToQ::Request& req, bender::URMoveToQ::Response& res)
{
	// de-scramble Q message
	rw::math::Q q(6, req.target.Q.data());
	
	// log message
	std::stringstream sstr;
	sstr << q;
	ROS_INFO("Requested URMoveToQ: Q=%s with speed %f", sstr.str().c_str(), req.speed);
	
	// perform service
	URInterface.moveQ(q, req.speed);
	
	return true;
}



/**
 * ur_move_to_t service
 */
bool ur_move_to_t(bender::URMoveToT::Request& req, bender::URMoveToT::Response& res)
{
	/*// de-scramble Q message
	rw::math::Q q(6, req.target.Q.data());
	
	// log message
	std::stringstream sstr;
	sstr << q;
	ROS_INFO("Requested URMoveToQ: Q=%s with speed %f", sstr.str().c_str(), req.speed);
	
	// perform service
	URInterface.moveQ(q, req.speed);*/
	
	return true;
}



/**
 * ur_servo_to_q service
 */
bool ur_servo_to_q(bender::URServoToQ::Request& req, bender::URServoToQ::Response& res)
{
	/*// de-scramble Q message
	rw::math::Q q(6, req.target.Q.data());
	
	// log message
	std::stringstream sstr;
	sstr << q;
	ROS_INFO("Requested URMoveToQ: Q=%s with speed %f", sstr.str().c_str(), req.speed);
	
	// perform service
	URInterface.moveQ(q, req.speed);*/
	
	return true;
}



int main(int argc, char* argv[])
{
	// configuration
	std::string hostIp = "192.168.2.8"; // localhost IP address
	std::string robotId = "UR1"; // robot ID
	std::string robotIp = "192.168.2.4"; // IP address of the robot
	unsigned urport = 30002; // port to use locally
	unsigned port = 30001; // port to use on robot
	std::string scriptFilename = "urscript1.ur"; // UR script filename
	
	// initialize ROS
	ros::init(argc, argv, "ur_node");
	
	// parse command line options
	std::string usage = "This ROS node is responsible for communication & control of UR robot.\n\n"
		"Usage:\n"
		"rosrun bender ur_node";
	options_description desc("Options");
	desc.add_options()
		("help,h", "help message")
		("host", value<std::string>(&hostIp), "local machine IP")
		("id", value<std::string>(&robotId)->required(), "robot ID (used for identification in ROS)")
		("id", value<std::string>(&robotIp)->required(), "robot IP address")
		("script", value<std::string>(&scriptFilename), "UR script filename (default: urscript.ur)")
	;
	variables_map vm;	
	
	// establish robot communication
	URInterface.connect(robotIp, urport);
	URInterface.start("192.168.2.8", port, scriptFilename);
	
	// configure ROS node
	ros::NodeHandle nh("~");
	
	// advertise services
	ros::ServiceServer urStopService = nh.advertiseService("ur_stop", ur_stop);
	ros::ServiceServer urMoveQService = nh.advertiseService("ur_move_to_q", ur_move_to_q);
	ros::ServiceServer urMoveTService = nh.advertiseService("ur_move_to_t", ur_move_to_t);
	ros::ServiceServer urServoQService = nh.advertiseService("ur_servo_to_q", ur_servo_to_q);

	// wait for calls
	ROS_INFO("Started ur_node.");
	ros::spin();
	
	return 0;
}

