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
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string.hpp>
#include <bender/Q.h>
#include <bender/URGetQ.h>
#include <bender/URStop.h>
#include <bender/URMoveToQ.h>
#include <bender/URMoveToT.h>
#include <bender/URServoToQ.h>



USE_ROBWORK_NAMESPACE;
using namespace rw;
using namespace boost::program_options;
using namespace boost::numeric;
using namespace boost::property_tree;



rwhw::URCallBackInterface URInterface;



/**
 * ur_get_q service
 */
bool ur_get_q(bender::URGetQ::Request& req, bender::URGetQ::Response& res)
{
	ROS_INFO("Requested URGetQ");
	
	// perform service
	
	return true;
}



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
	// unscramble Q message
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



/**
 * @class URNodeConfiguration
 * 
 * @brief Contains ur_node configuration read from XML configuration file
 */
struct URNodeConfiguration
{
	/* data */
	std::string id;
	std::string hostIP;
	unsigned hostPort;
	std::string robotIP;
	unsigned robotPort;
	std::string script;
	
	/* constructors */
	URNodeConfiguration() :
		id("UR1"),
		hostIP("192.168.2.8"),
		hostPort(30002),
		robotPort(30001),
		script("urscript1.ur")
	{ }
	
	static URNodeConfiguration& loadfromXML(std::string filename)
	{
		URNodeConfiguration* config = new URNodeConfiguration();
		
		try {
			// get path
			boost::filesystem::path p(filename);
			std::string configPath = p.parent_path().string()+"/";
			
			boost::property_tree::ptree tree;
			read_xml(filename, tree);
			boost::property_tree::ptree root = tree.get_child("ur_node");

			// read robot id
			std::string id = root.get_child("id").get_value<std::string>();
			boost::trim(id);
			config->id = id;
			
			// read host ip
			std::string hostIP = root.get_child("hostIP").get_value<std::string>();
			boost::trim(hostIP);
			config->hostIP = hostIP;
			
			// read host port
			std::string hostPort = root.get_child("hostPort").get_value<std::string>();
			boost::trim(hostPort);
			config->hostPort = boost::lexical_cast<unsigned>(hostPort);
			
			// read robot ip
			std::string robotIP = root.get_child("robotIP").get_value<std::string>();
			boost::trim(robotIP);
			config->robotIP = robotIP;
			
			// read robot port
			std::string robotPort = root.get_child("robotPort").get_value<std::string>();
			boost::trim(robotPort);
			config->robotPort = boost::lexical_cast<unsigned>(robotPort);
			
			// read script
			std::string script = root.get_child("script").get_value<std::string>();
			boost::trim(script);
			config->script = configPath + script;
			
		} catch (const ptree_error& e) {
			// convert from parse errors to RobWork errors.
			RW_THROW(e.what());
		}
		
		return *config;
	}
	
	friend std::ostream& operator<<(std::ostream& stream, const URNodeConfiguration& config)
	{
		stream << "CONFIGURATION:\n"
			" * ID: " << config.id << "\n"
			" * host IP/port: " << config.hostIP << ":" << config.hostPort << "\n"
			" * robot IP/port: " << config.robotIP << ":" << config.robotPort << "\n"
			" * script: " << config.script << std::endl;
		
		return stream;
	}
};



int main(int argc, char* argv[])
{
	// configuration
	std::string configFile;
	URNodeConfiguration config;
	
	// initialize ROS
	ros::init(argc, argv, "ur_node");
	
	// parse command line options
	std::string usage = "This ROS node is responsible for communication & control of UR robot.\n\n"
		"Usage:\n"
		"rosrun bender ur_node CONFIG";
	// named options
	options_description desc("Options");
	desc.add_options()
		("help,h", "help message")
		("config", value<std::string>(&configFile), "configuration XML file")
		("id", value<std::string>(&config.id), "robot ID (used for identification in ROS)")
		("hostip", value<std::string>(&config.hostIP), "local machine IP")
		("hostport", value<unsigned>(&config.hostPort), "local machine port")
		("robotip", value<std::string>(&config.robotIP), "robot IP")
		("hostport", value<unsigned>(&config.robotPort), "robot port")
		("script", value<std::string>(&config.script), "UR script filename")
	;
	// positional options
	positional_options_description pos_desc;
	pos_desc.add("config", -1);
	
	// parse options
	variables_map vm;
	try {
		store(command_line_parser(argc, argv).options(desc).positional(pos_desc).run(), vm);
		notify(vm);
		
		if (vm.count("config")) {
			config = URNodeConfiguration::loadfromXML(vm["config"].as<std::string>());
		}
		
		if (vm.count("id")) {
			config.id = vm["id"].as<std::string>();
		}
		
		if (vm.count("hostip")) {
			config.hostIP = vm["hostip"].as<std::string>();
		}
		
		if (vm.count("hostport")) {
			config.hostPort = vm["hostport"].as<unsigned>();
		}
		
		if (vm.count("robotip")) {
			config.robotIP = vm["robotip"].as<std::string>();
		}
		
		if (vm.count("robotport")) {
			config.robotPort = vm["robotport"].as<unsigned>();
		}
		
		if (vm.count("script")) {
			config.script = vm["script"].as<std::string>();
		}
	} catch (...) {
		std::cout << usage << std::endl;
		std::cout << desc << std::endl;
		
		return -1;
	}
	
	ROS_INFO("Starting ur_node...");
	std::cout << config << std::endl;
	
	// establish robot communication
	URInterface.connect(config.robotIP, config.robotPort);
	URInterface.start(config.hostIP, config.hostPort, config.script);
	
	// configure ROS node
	ros::NodeHandle nh("~");
	
	// advertise services
	ros::ServiceServer urGetQService = nh.advertiseService("ur_get_q", ur_get_q);
	ros::ServiceServer urStopService = nh.advertiseService("ur_stop", ur_stop);
	ros::ServiceServer urMoveQService = nh.advertiseService("ur_move_to_q", ur_move_to_q);
	ros::ServiceServer urMoveTService = nh.advertiseService("ur_move_to_t", ur_move_to_t);
	ros::ServiceServer urServoQService = nh.advertiseService("ur_servo_to_q", ur_servo_to_q);

	// wait for calls
	ROS_INFO("Started.");
	ros::spin();
	
	return 0;
}

