#pragma once

#include <rw/rw.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rws/propertyview/PropertyViewEditor.hpp>
#include <rw/common/Timer.hpp>
#include <ros/ros.h>
#include <bender/URState.h>
#include <QObject>
#include <QtGui>
#include <QTimer>
#include "ui_SimplePlugin.h"



/**
 * @brief A plugin for controlling Bender remotely.
 */
class SimplePlugin : public rws::RobWorkStudioPlugin
{
	Q_OBJECT
	Q_INTERFACES(rws::RobWorkStudioPlugin)
	
	public:
		//! @brief constructor
		SimplePlugin();

		//! @brief destructor
		virtual ~SimplePlugin();

		//! @copydoc rws::RobWorkStudioPlugin::open(rw::models::WorkCell* workcell)
		virtual void open(rw::models::WorkCell* workcell);

		//! @copydoc rws::RobWorkStudioPlugin::close()
		virtual void close();

		//! @copydoc rws::RobWorkStudioPlugin::initialize()
		virtual void initialize();

	private slots:
		//! @brief updates workcell
		void update();

	private:
		//! @brief set-up GUI
		void setupGUI();
		
		//! @brief callback for UR1 state
		void ur1StateCallback(const bender::URState::ConstPtr& msg);
		
		//! @brief callback for UR2 state
		void ur2StateCallback(const bender::URState::ConstPtr& msg);
		
		// GUI
		Ui::SimpleWidget ui;
		
		QTimer* _timer;
		
		// RW stuff
		rw::models::WorkCell* _workcell;
		rw::kinematics::State _state;
		rw::models::Device::Ptr _ur1;
		rw::models::Device::Ptr _ur2;
		
		// ROS stuff
		ros::NodeHandle* _node;
		ros::Subscriber _ur1sub;
		ros::Subscriber _ur2sub;
};
