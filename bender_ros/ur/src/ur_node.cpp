/**
 * A simple service server for moving UR around
 */

#include <iostream>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <rw/rw.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
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
#include <bender/URState.h>
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
rwhw::UniversalRobotsRTLogging URRTInterface;



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
	unsigned robotRTPort;
	std::string script;
	
	/* constructors */
	URNodeConfiguration() :
		id("UR1"),
		hostIP("192.168.2.8"),
		hostPort(30002),
		robotPort(30001),
		robotRTPort(30003),
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
			
			// read robot rt port
			std::string robotRTPort = root.get_child("robotRTPort").get_value<std::string>();
			boost::trim(robotRTPort);
			config->robotRTPort = boost::lexical_cast<unsigned>(robotRTPort);
			
			// read script
			std::string script = root.get_child("robotScript").get_value<std::string>();
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
	// initialize ROS
	ros::init(argc, argv, "ur_node");
	
	ROS_INFO("Starting ur_node...");
	
	// configure ROS node
	ros::NodeHandle nh("~");
	
	// read parameters
	std::string configFile;
	nh.getParam("config", configFile);
	URNodeConfiguration config = URNodeConfiguration::loadfromXML(configFile);
	std::cout << config << std::endl;
	
	// establish robot communication
	URRTInterface.connect(config.robotIP, config.robotRTPort);
	URRTInterface.start();
	URInterface.connect(config.robotIP, config.robotPort);
	//URInterface.startInterface(config.hostIP, config.hostPort, config.script);
	URInterface.startCommunication(config.hostIP, config.hostPort, config.script);
	
	// advertise topics
	ros::Publisher urStatePublisher = nh.advertise<bender::URState>("ur_state", 1000);
	
	// advertise services
	ros::ServiceServer urGetQService = nh.advertiseService("ur_get_q", ur_get_q);
	ros::ServiceServer urStopService = nh.advertiseService("ur_stop", ur_stop);
	ros::ServiceServer urMoveQService = nh.advertiseService("ur_move_to_q", ur_move_to_q);
	ros::ServiceServer urMoveTService = nh.advertiseService("ur_move_to_t", ur_move_to_t);
	ros::ServiceServer urServoQService = nh.advertiseService("ur_servo_to_q", ur_servo_to_q);

	// main loop
	ROS_INFO("Started.");
	ros::Rate loop_rate(100);
	while (ros::ok()) {
		// get last RT data
		rwhw::URRTData URData;
		URData = URRTInterface.getLastData();
		
		// extract data
		rw::math::Q qActual = URData.qActual;
		
		// create message
		bender::URState msg;
		
		bender::Q qActualMsg;
		if (qActual.size() == 6) {
			for (int i = 0; i < 6; ++i) {
				qActualMsg.Q.data()[i] = qActual[i];
			}
		}
		msg.qActual = qActualMsg;
		
		// publish
		urStatePublisher.publish(msg);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

