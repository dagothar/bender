#include "SimplePlugin.hpp"

#include <rws/RobWorkStudio.hpp>
#include <rw/common/Exception.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>



using namespace std;
USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rws;



SimplePlugin::SimplePlugin() :
	RobWorkStudioPlugin("SimplePlugin", QIcon(":/pa_icon.png"))	
{
    ui.setupUi(this);
    setupGUI();
}



SimplePlugin::~SimplePlugin()
{
}



void SimplePlugin::initialize()
{
	/* Initialize ROS
	 */
	int argc = 0;
	char* argv[] = {""};
	ros::init(argc, argv, "simple_plugin");
	
	/* Create ROS node
	 */
	_node = new ros::NodeHandle("~");
	
	/* Start listeners
	 */
	_ur1sub =_node->subscribe("/ur1/ur_state", 1000, &SimplePlugin::ur1StateCallback, this);
	_ur2sub =_node->subscribe("/ur2/ur_state", 1000, &SimplePlugin::ur2StateCallback, this);
	
	/* Start loop
	 */
	_timer = new QTimer();
	_timer->setInterval(1);
	connect(_timer, SIGNAL(timeout()), this, SLOT(update()));
	_timer->start();
}



void SimplePlugin::open(WorkCell* workcell)
{
	// set up workcell
	_workcell = workcell;
	
	// find the robot
	if (_workcell->getDevices().empty()) {
		return;
	} else {
		_ur1 = _workcell->findDevice("UR1");
		_ur2 = _workcell->findDevice("UR2");
	}
	
	// get default state
	_state = _workcell->getDefaultState();
}



void SimplePlugin::close()
{
}



void SimplePlugin::setupGUI()
{
}



void SimplePlugin::update()
{
	//ROS_INFO("Update");
	ros::spinOnce();
	
	if (_ur1 && _ur2) {
		getRobWorkStudio()->setState(_state);
	}
}



void SimplePlugin::ur1StateCallback(const bender::URState::ConstPtr& msg)
{
	rw::math::Q q(6, msg->qActual.Q.data());
	
	//cout << q << std::endl;
	
	if (_ur1) {
		_ur1->setQ(q, _state);
	}
}



void SimplePlugin::ur2StateCallback(const bender::URState::ConstPtr& msg)
{
	rw::math::Q q(6, msg->qActual.Q.data());
	
	if (_ur2) {
		_ur2->setQ(q, _state);
	}
}



Q_EXPORT_PLUGIN(SimplePlugin);
