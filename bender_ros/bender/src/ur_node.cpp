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
	
	
	// initialize ROS
	ros::init(argc, argv, "ur_node");
	
	// parse command line options
	string usage = "This is a script used to generate tasks for a single gripper, simulate them and"
		" evaluate gripper's performance.\n\n"
		"Usage:\n"
		"evaluate-gripper";
	options_description desc("Options");
	desc.add_options()
		("help,h", "help message")
		("ntargets,t", value<int>(&ntargets)->default_value(0), "number of tasks to generate")
		("nsamples,s", value<int>(&nsamples)->default_value(0), "number of samples to use")
		("dwc", value<string>(&dwcFilename)->required(), "dynamic workcell file")
		("td", value<string>(&tdFilename)->required(), "task description file")
		("gripper,g", value<string>(&gripperFilename)->required(), "gripper file")
		("samples", value<string>(), "surface samples file")
		("out,o", value<string>(), "task file")
		("nosim", "don't perform simulation")
		("robustness,r", value<int>(&rtargets), "test robustnesss with s number of targets")
		("sigma_a", value<double>(&sigma_a)->default_value(8), "Standard deviation in of angle in degrees.")
        ("sigma_p", value<double>(&sigma_p)->default_value(0.003), "Standard deviation of position in meters.")
	;
	variables_map vm;
	
	// configure robot communication TODO
	std::string ip = "192.168.2.4";
	unsigned urport = 30002;
	unsigned port = 30001;
	std::string script = "urscript1.ur";
	
	// establish robot communication
	URInterface.connect(ip, urport);
	URInterface.start("192.168.2.8", port, script);
	
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

