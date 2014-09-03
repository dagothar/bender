#include "AdventurePlugin.hpp"

#include <rws/RobWorkStudio.hpp>



using namespace std;

USE_ROBWORK_NAMESPACE;
using namespace robwork;



AdventurePlugin::AdventurePlugin() :
		RobWorkStudioPlugin("AdventurePlugin", QIcon(":/adventure_icon.png"))
{
	ui.setupUi(this);
}



AdventurePlugin::~AdventurePlugin()
{
}



void AdventurePlugin::open(rw::models::WorkCell* workcell)
{
	Math::seed(TimerUtil::currentTimeUs());
		
	_wc = workcell;
}



void AdventurePlugin::initialize()
{
	// add a generic event listener
	getRobWorkStudio()->genericEvent().add(
			boost::bind(&AdventurePlugin::genericEventListener, this, _1), this);
		
	// add a keyboard event listener
	getRobWorkStudio()->keyEvent().add(
			boost::bind(&AdventurePlugin::keyEventListener, this, _1, _2), this);
			
	// add a frame selected event listener
	getRobWorkStudio()->frameSelectedEvent().add(
			boost::bind(&AdventurePlugin::frameSelectedEventListener, this, _1), this);
			
	// add a position selected event listener
	getRobWorkStudio()->positionSelectedEvent().add(
			boost::bind(&AdventurePlugin::positionSelectedEventListener, this, _1), this);
}



void AdventurePlugin::close()
{
}



void AdventurePlugin::genericEventListener(const std::string& event)
{
}



void AdventurePlugin::keyEventListener(int key, Qt::KeyboardModifiers modifier)
{
}



void AdventurePlugin::frameSelectedEventListener(rw::kinematics::Frame* frame)
{
	Log::infoLog() << "FRAME EVENT!" << endl;
	
	if (frame) {
		Log::infoLog() << "Clicked: " << frame->getName() << endl;
	}
}



void AdventurePlugin::positionSelectedEventListener(rw::math::Vector3D<> pos)
{
	Log::infoLog() << "POSITION EVENT:" << pos << endl;
}



Q_EXPORT_PLUGIN(AdventurePlugin);
