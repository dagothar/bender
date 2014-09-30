#include "SimplePlugin.hpp"

#include <rws/RobWorkStudio.hpp>
#include <rw/common/Exception.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <bender/URMoveToQ.h>
#include <bender/URStop.h>



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
		_ur1ghost = _workcell->findDevice("UR1_ghost");
		_ur2ghost = _workcell->findDevice("UR2_ghost");
	}
	
	// get default state
	_state = _workcell->getDefaultState();
}



void SimplePlugin::close()
{
}



void SimplePlugin::setupGUI()
{
	connect(ui.ur1MoveButton, SIGNAL(clicked()), this, SLOT(ur1Move()));
	connect(ui.ur2MoveButton, SIGNAL(clicked()), this, SLOT(ur2Move()));
	connect(ui.stopButton, SIGNAL(clicked()), this, SLOT(stopRobots()));
}



void SimplePlugin::update()
{
	//ROS_INFO("Update");
	ros::spinOnce();
	
	if (_ur1 && _ur2) {
		_state = getRobWorkStudio()->getState();
		_ur1->setQ(_ur1q, _state);
		_ur2->setQ(_ur2q, _state);
		getRobWorkStudio()->setState(_state);
	}
}



void SimplePlugin::ur1Move()
{
	if (_ur1) {
		Q q = _ur1ghost->getQ(getRobWorkStudio()->getState());
		
		bender::URMoveToQ srv;
		for (int i = 0; i < 6; ++i) {
			srv.request.target.Q.data()[i] = q[i];
		}
		srv.request.speed = 0.1;
		
		ros::service::call("/ur1/ur_move_to_q", srv);
		
		cout << "Calling UR1 move: " << q << endl;
	}
}



void SimplePlugin::ur2Move()
{
	if (_ur2) {
		Q q = _ur2ghost->getQ(getRobWorkStudio()->getState());
		
		bender::URMoveToQ srv;
		for (int i = 0; i < 6; ++i) {
			srv.request.target.Q.data()[i] = q[i];
		}
		srv.request.speed = 0.1;
		
		ros::service::call("/ur2/ur_move_to_q", srv);
		
		cout << "Calling UR2 move: " << q << endl;
	}
}



void SimplePlugin::stopRobots()
{
	bender::URStop srv1, srv2;
	
	ros::service::call("/ur1/ur_stop", srv1);
	ros::service::call("/ur2/ur_stop", srv2);
}



void SimplePlugin::ur1StateCallback(const bender::URState::ConstPtr& msg)
{
	rw::math::Q q(6, msg->qActual.Q.data());
	
	_ur1q = q;
}



void SimplePlugin::ur2StateCallback(const bender::URState::ConstPtr& msg)
{
	rw::math::Q q(6, msg->qActual.Q.data());
	
	_ur2q = q;
}



Q_EXPORT_PLUGIN(SimplePlugin);
