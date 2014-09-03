#pragma once

#include <rw/rw.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <QObject>
#include <QtGui>
#include <QTimer>

#include "ui_adventure.h"



class AdventurePlugin : public rws::RobWorkStudioPlugin
{
	Q_OBJECT
	Q_INTERFACES(rws::RobWorkStudioPlugin)
	
	public:
		//! @brief constructor
		AdventurePlugin();

		//! @brief destructor
		virtual ~AdventurePlugin();

		//! @copydoc rws::RobWorkStudioPlugin::open(rw::models::WorkCell* workcell)
		virtual void open(rw::models::WorkCell* workcell);

		//! @copydoc rws::RobWorkStudioPlugin::close()
		virtual void close();

		//! @copydoc rws::RobWorkStudioPlugin::initialize()
		virtual void initialize();
		
		/**
		 * @brief Listening for generic events emitted by RW
		 */
		void genericEventListener(const std::string& event);
		
		/**
		 * @brief Listening for keyboard events
		 */
		void keyEventListener(int key, Qt::KeyboardModifiers modifier);
		
		/**
		 * @brief Listening for frame selected events
		 */
		void frameSelectedEventListener(rw::kinematics::Frame* frame);
		
		/**
		 * @brief Listening for position selected event
		 */
		void positionSelectedEventListener(rw::math::Vector3D<> pos);

	private:
		/* GUI */
		Ui::AdventureWidget ui;
		
		/* stuff */
		rw::models::WorkCell* _wc;
};
