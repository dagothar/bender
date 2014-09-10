#pragma once

#include <rw/rw.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rws/propertyview/PropertyViewEditor.hpp>
#include <rw/common/Timer.hpp>
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

	private:
		//! @brief set-up GUI
		void setupGUI();
		
		// GUI
		Ui::SimpleWidget ui;
};
