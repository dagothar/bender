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
}



void SimplePlugin::open(WorkCell* workcell)
{
}



void SimplePlugin::close()
{
}



void SimplePlugin::setupGUI()
{
}



Q_EXPORT_PLUGIN(SimplePlugin);
