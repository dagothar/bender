#include "SkywalkerPlugin.hpp"

#include <rws/RobWorkStudio.hpp>
#include <rws/CameraController.hpp>
#include <rws/SceneOpenGLViewer.hpp>



using namespace std;

USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rws;



SkywalkerPlugin::SkywalkerPlugin() :
		RobWorkStudioPlugin("SkywalkerPlugin", QIcon(":/skywalker_icon.png")),
		_enabled(false)
{
	ui.setupUi(this);
	
	// connect stuff
	connect(ui.enableBox, SIGNAL(clicked()), this, SLOT(enableClicked()));
	connect(ui.saveButton, SIGNAL(clicked()), this, SLOT(saveView()));
	connect(ui.restoreButton, SIGNAL(clicked()), this, SLOT(restoreView()));
	connect(ui.snapshotButton, SIGNAL(clicked()), this, SLOT(snapshot()));
}



SkywalkerPlugin::~SkywalkerPlugin()
{
}



void SkywalkerPlugin::initialize()
{
	// add a keyboard event listener
	getRobWorkStudio()->keyEvent().add(
			boost::bind(&SkywalkerPlugin::keyEventListener, this, _1, _2), this);
}



void SkywalkerPlugin::keyEventListener(int key, Qt::KeyboardModifiers modifier)
{
	/* let's move the view */
	/*Transform3D<> viewT = getRobWorkStudio()->getViewTransform();
	Rotation3D<> rot;
	Transform3D<> nviewT;
	
	switch (key) {
		case 'A':
			rot = EAA<>(Vector3D<>::z(), -5.0*Deg2Rad).toRotation3D();
			nviewT = Transform3D<>(rot * viewT.P(), rot * viewT.R());
			getRobWorkStudio()->setViewTransform(nviewT);
			getRobWorkStudio()->updateAndRepaint();
			break;
			
		case 'D':
			rot = EAA<>(Vector3D<>::z(), 5.0*Deg2Rad).toRotation3D();
			nviewT = Transform3D<>(rot * viewT.P(), rot * viewT.R());
			getRobWorkStudio()->setViewTransform(nviewT);
			getRobWorkStudio()->updateAndRepaint();
			break;
			
		case 'W':
			rot = EAA<>(viewT.R() * Vector3D<>::x(), -5.0*Deg2Rad).toRotation3D();
			nviewT = Transform3D<>(rot * viewT.P(), rot * viewT.R());
			getRobWorkStudio()->setViewTransform(nviewT);
			getRobWorkStudio()->updateAndRepaint();
			break;
			
		case 'S':
			rot = EAA<>(viewT.R() * Vector3D<>::x(), 5.0*Deg2Rad).toRotation3D();
			nviewT = Transform3D<>(rot * viewT.P(), rot * viewT.R());
			getRobWorkStudio()->setViewTransform(nviewT);
			getRobWorkStudio()->updateAndRepaint();
			break;
			
		case 'Z':
			nviewT = Transform3D<>(1.1*viewT.P(), viewT.R());
			getRobWorkStudio()->setViewTransform(nviewT);
			getRobWorkStudio()->updateAndRepaint();
			break;
			
		case 'X':
			nviewT = Transform3D<>(viewT.P()/1.1, viewT.R());
			getRobWorkStudio()->setViewTransform(nviewT);
			getRobWorkStudio()->updateAndRepaint();
			break;
	}*/
	
	switch (key) {
		case 'A':
			_controller->movePlayer(Vector3D<>(0.0, 1.0, 0.0));
			break;
			
		case 'D':
			_controller->movePlayer(Vector3D<>(0.0, -1.0, 0.0));
			break;
			
		case 'W':
			_controller->movePlayer(Vector3D<>(1.0, 0.0, 0.0));
			break;
			
		case 'S':
			_controller->movePlayer(Vector3D<>(-1.0, 0.0, 0.0));
			break;
			
		case 'Q':
			_controller->movePlayer(Vector3D<>(0.0, 0.0, -1.0));
			break;
			
		case 'E':
			_controller->movePlayer(Vector3D<>(0.0, 0.0, 1.0));
			break;
	}
	
	getRobWorkStudio()->setViewTransform(_controller->getTransform());
	getRobWorkStudio()->updateAndRepaint();
}



void SkywalkerPlugin::enableClicked()
{
}



void SkywalkerPlugin::saveView()
{
	getRobWorkStudio()->getPropertyMap().addForce<Transform3D<> >("SkywalkerView",
			"View saved from the Skywalker plugin",
			getRobWorkStudio()->getViewTransform());
}



void SkywalkerPlugin::restoreView()
{
	Transform3D<> nviewT = getRobWorkStudio()->getPropertyMap().get<Transform3D<> >("SkywalkerView",
			getRobWorkStudio()->getViewTransform());
	
	getRobWorkStudio()->setViewTransform(nviewT);
	getRobWorkStudio()->updateAndRepaint();
}



void SkywalkerPlugin::snapshot()
{
	Transform3D<> viewT = getRobWorkStudio()->getViewTransform();
	
	SceneOpenGLViewer::Ptr viewer =  (SceneOpenGLViewer*)getRobWorkStudio()->getView()->getSceneViewer().get();
	
	_controller = new FPPController(640, 480);
	viewer->setCameraController(_controller);
	
	//CameraController::Ptr controller = viewer->getCameraController();
	_controller->setCenter(viewT.P(), Vector2D<>(0, 0));
}



Q_EXPORT_PLUGIN(SkywalkerPlugin);
