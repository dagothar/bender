#pragma once

#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/RPY.hpp>
#include <rws/CameraController.hpp>

namespace rws{

	/**
	 * @brief Use the first person perspective method to control the camera view point in a scene.
	 */
	class FPPController: public CameraController {
	public:
		//! @brief Smart pointer to this type of class
		typedef rw::common::Ptr<FPPController> Ptr;

		/**
		 * @brief constructor
		 */
		FPPController(double NewWidth, double NewHeight);

		/**
		 * @brief destructor
		 */
		virtual ~FPPController() {}
		
		/**
         * @brief Registers mouse click at the position (x,y).
         * @param x [in] the x-coordinate of the current mouse position
         * @param y [in] the y-coordinate of the current mouse position
         */
        void click(float x, float y);

        /**
         * @brief Calculates the rotation of the object/scene based on
         * the mouse being dragged to the position (x,y).
         * @param x [in] the x-coordinate of the current mouse position
         * @param y [in] the y-coordinate of the current mouse position
         * @return the rotation that should be applied to the object/scene
         */
        rw::math::RPY<double> drag(float x, float y);
        
        /**
         * @brief Sets mouse sensitivity for rotating view.
         * 
         * @param s [in] sensitivity multiplier
         */
        void setSensitivity(float s=0.01) { _sensitivity = s; }

        //! @copydoc CameraController::draw
        void draw() {}

        //! @copydoc CameraController::setBounds
        void setBounds(double NewWidth, double NewHeight);

        //! @copydoc CameraController::handleEvent
        virtual void handleEvent(QEvent* event);

        //! @copydoc CameraController::getTransform
        virtual rw::math::Transform3D<> getTransform() const;

        //! @copydoc CameraController::setTransform
        void setTransform(const rw::math::Transform3D<>& t3d);

        //! @copydoc CameraController::setCenter
        void setCenter(const rw::math::Vector3D<>& center,
                          const rw::math::Vector2D<>& screenCenter);

        //! @copydoc CameraController::getCenter
        rw::math::Vector3D<> getCenter() { return _pivotPoint; }
        
        //! @brief Get player position.
        rw::math::Vector3D<> getPlayerPosition() const { return _playerPosition; }
        
        //! @brief Set player position
        void setPlayerPosition(rw::math::Vector3D<> pos) { _playerPosition = pos; }
        
        //! @brief Issue movement command.
        void movePlayer(rw::math::Vector3D<> delta);

	private:
		rw::math::Transform3D<> _viewTransform;
	
		//rw::math::Transform3D<> _playerPose;
		rw::math::Vector3D<> _playerPosition;
		rw::math::RPY<> _playerDirection;
		float _pan, _tilt;
		
		float _sensitivity, _pace;
		
		double _height, _width;
		double _adjustWidth;    // Mouse bounds width
		double _adjustHeight;   // Mouse bounds height
		
		rw::math::Vector2D<> _lastPos; // last clicked position on screen
		
		rw::math::Vector2D<> _stVec;          // Saved click vector
		rw::math::Vector2D<> _enVec;          // Saved drag vector
		
		rw::math::Vector2D<> _centerPt; // Center of the ball
		
		rw::math::Vector3D<> _pivotPoint;
		double _zoomFactor, _zoomScale;
		

	};

}

