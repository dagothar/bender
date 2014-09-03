#include "FPPController.hpp"

#include <rwlibs/os/rwgl.hpp>

#include <math.h> // Needed for sqrtf
#include <rw/math/Math.hpp>



using namespace rw;
using namespace rw::math;
using namespace rws;



FPPController::FPPController(double NewWidth, double NewHeight) :
	_centerPt(NewWidth/2.0, NewHeight/2.0),
    _sensitivity(0.005),
    _pace(0.1),
    _pan(0.0),
    _tilt(0.0),
    _playerPosition(-5.0, 0.0, 0.0)
{
    _viewTransform = Transform3D<>::makeLookAt(Vector3D<>(5, 5, 1), Vector3D<>(0, 0, 1), Vector3D<>::z());

    this->setBounds(NewWidth, NewHeight);
}



void FPPController::setBounds(double NewWidth, double NewHeight)
{
    _width = NewWidth;
    _height = NewHeight;

    // Set adjustment factor for width/height
    _adjustWidth  = 1.0f / ((NewWidth  - 1.0f));
    _adjustHeight = 1.0f / ((NewHeight - 1.0f));
}



void FPPController::click(float x, float y)
{
	_lastPos = Vector2D<>(x, y);
}



rw::math::RPY<double> FPPController::drag(float x, float y)
{
	// calculate change in players view pan & tilt
	Vector2D<> newPos(x, y);
    Vector2D<> diff = newPos - _lastPos;
    
    _pan -= _sensitivity * diff(0);
    _tilt -= _sensitivity * diff(1);
    
    _playerDirection = RPY<>(90.0 * Deg2Rad + _pan, 180.0 * Deg2Rad, -90.0 * Deg2Rad + _tilt);
    
    return _playerDirection;
}



void FPPController::handleEvent(QEvent* e)
{
    if (e->type() == QEvent::MouseButtonPress) { // if it's justaclick, store initial position

        QMouseEvent *event = static_cast<QMouseEvent*>(e);
        click(event->x(), event->y());

    } else if(e->type() == QEvent::MouseMove) { // now it's a drag

        QMouseEvent *event = static_cast<QMouseEvent*>(e);
        
        if (event->buttons() == Qt::LeftButton) { // using LMB

			RPY<double> rpy = drag(event->x(), event->y());

			Transform3D<> &wTc = _viewTransform;			
			wTc = Transform3D<>(_playerPosition, _playerDirection);

			click(event->x(), event->y());
			
		}
	}
}


void FPPController::setCenter(const rw::math::Vector3D<>& center,
		const rw::math::Vector2D<>& screenCenter)
{
    _pivotPoint = center;
    _centerPt = screenCenter;
}

rw::math::Transform3D<> FPPController::getTransform() const
{
    return _viewTransform;
}

void FPPController::setTransform(const rw::math::Transform3D<>& t3d)
{
    _viewTransform = t3d;
}



void FPPController::movePlayer(rw::math::Vector3D<> delta)
{
	_playerPosition(2) += _pace * delta(2);
	_playerPosition(0) += _pace * (delta(0) * cos(_pan) - delta(1) * sin(_pan));
	_playerPosition(1) += _pace * (delta(0) * sin(_pan) + delta(1) * cos(_pan));
	
	_viewTransform = Transform3D<>(_playerPosition, _playerDirection);
}


