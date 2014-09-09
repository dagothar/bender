#include "HypnotoadPlugin.hpp"

#include <rws/RobWorkStudio.hpp>
#include <rw/common/Exception.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>



using namespace std;
USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rws;



HypnotoadPlugin::HypnotoadPlugin() :
	RobWorkStudioPlugin("HypnotoadPlugin", QIcon(":/pa_icon.png"))
{
    //ui.setupUi(this);
    //setupGUI();
}



HypnotoadPlugin::~HypnotoadPlugin()
{
}



void HypnotoadPlugin::initialize()
{
}



void HypnotoadPlugin::open(WorkCell* workcell)
{
}



void HypnotoadPlugin::close()
{
}



Q_EXPORT_PLUGIN(HypnotoadPlugin);
