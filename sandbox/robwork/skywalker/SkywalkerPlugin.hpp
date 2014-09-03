#pragma once

#include <rw/rw.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <QObject>
#include <QtGui>
#include <QTimer>
#include "FPPController.hpp"
#include "ui_skywalker.h"



class SkywalkerPlugin : public rws::RobWorkStudioPlugin
{
	Q_OBJECT
	Q_INTERFACES(rws::RobWorkStudioPlugin)
	
	public:
		//! @brief constructor
		SkywalkerPlugin();

		//! @brief destructor
		virtual ~SkywalkerPlugin();

		//! @copydoc rws::RobWorkStudioPlugin::initialize()
		virtual void initialize();
		
		/**
		 * @brief Listening for keyboard events
		 */
		void keyEventListener(int key, Qt::KeyboardModifiers modifier);
		
	private slots:
		//! @brief enable/disable functionality
		void enableClicked();
		
		//! @brief save view point
		void saveView();
		
		//! @brief restore view point
		void restoreView();
		
		//! @brief make snapshot of view area
		void snapshot();

	private:
		/* GUI */
		Ui::SkywalkerWidget ui;
		
		/* other stuff */
		bool _enabled;
		rw::math::Transform3D<> _position; // player's position
		rws::FPPController::Ptr _controller; // camera controller
};
