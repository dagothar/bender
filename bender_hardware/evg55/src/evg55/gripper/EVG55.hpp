#ifndef _EVG55_HPP
#define _EVG55_HPP

#include <evg55/serial/SerialPort.hpp>
#include "GripperException.hpp"

namespace evg55 {
namespace gripper {

class EVG55 {
public:
	//! State flags.
	enum State {
		StatusReferenced = 0x01,
		StatusMoving = 0x02,
		StatusProgramMode = 0x04,
		StatusWarning = 0x08,
		StatusError = 0x10,
		StatusBrake = 0x20,
		StatusMoveEnd = 0x40,
		StatusPositionReached = 0x80
	};
	
	//! Error codes.
	enum Error {
		ErrorSoftLow = 0xd5
		//TODO: add all flags
	};
	
	//! Timeout for move commands
	static const float MoveTimeout = 10.0; // s
	
	//! Max gripper opening
	static const float MaxOpening = 75.0; // mm
	
	//! Max gripper velocity
	static const float MaxVelocity = 100.0; // mm/s
	
	//! Max gripping current
	static const float MaxCurrent = 5.0; // A
	
	//! Allowed error for positioning
	static const float EpsPosition = 1.0; // mm
	
public:
	/* CONSTRUCTORS */
	/**
	 * @brief Constructor.
	 */
	EVG55();
	
	/**
	 * @brief Destructor.
	 */
	virtual ~EVG55();
	
	/* INITIALIZATION */
	/**
	 * @brief Connects to a gripper module through serial port.
	 * Specify a correct module id.
	 * @param port [in] serial port to use
	 * @param id [in] module id
	 * @return \b true if connection succesful, \b false otherwise
	 */
	bool connect(serial::SerialPort* port, unsigned char id);
	
	/**
	 * @brief Disconnects gripper module.
	 */
	void disconnect();
	
	/**
	 * @brief Sends error acknowledgement command.
	 * Attempts to clear error state of the module.
	 */
	bool clearError();
	
	/* GET INFO */
	/**
	 * @brief Polls gripper module for new information.
	 * Calls GetState.
	 */
	bool poll();
	
	/**
	 * @brief Returns gripper state bytes.
	 * @param doPoll [in] if \b true poll() is called, otherwise uses stored information
	 */
	unsigned short getStatus(bool doPoll = false);
	
	/**
	 * @brief Returns \b true if gripper is in OK state, and \b false if there is any kind of error.
	 * @param doPoll [in] if \b true poll() is called, otherwise uses stored information.
	 */
	bool isOk(bool doPoll = false);
	
	/**
	 * @brief Returns \b true if gripper is referenced.
	 * @param doPoll [in] if \b true poll() is called, otherwise uses stored information
	 */
	bool isReferenced(bool doPoll = false);
	
	/**
	 * @brief Returns gripper's position.
	 * @param doPoll [in] if \b true poll() is called, otherwise uses stored information
	 */
	float getPosition(bool doPoll = false);
	
	/**
	 * @brief Returns gripper's velocity.
	 * @param doPoll [in] if \b true poll() is called, otherwise uses stored information
	 */
	float getVelocity(bool doPoll = false) const;
	
	/**
	 * @brief Returns gripper's current.
	 * @param doPoll [in] if \b true poll() is called, otherwise uses stored information
	 */
	float getCurrent(bool doPoll = false) const;
	
	/**
	 * @brief Returns gripper error code.
	 * @param doPoll [in] if \b true poll() is called, otherwise uses stored information
	 */
	unsigned char getErrorCode(bool doPoll = false);
	
	/* COMMAND API */
	/**
	 * @brief Attempts to reference the gripper.
	 * @return \b true if succesful
	 */
	bool home();
	
	/**
	 * @brief Set gripper opening.
	 * Doesn't wait till move completed.
	 * @param pos [in] position in mm
	 * @return \b true if move succesful
	 */
	bool move(float pos);
	
	/**
	 * @brief Set gripper opening.
	 * Waits till move completed.
	 * @param pos [in] position in mm
	 * @return \b true if move succesful
	 */
	bool moveWait(float pos);
	
	/**
	 * @brief Opens gripper.
	 */
	bool open();
	
	/**
	 * @brief Opens gripper.
	 * Waits till move completed.
	 */
	bool openWait();
	
	/**
	 * @brief Closes gripper.
	 */
	bool close();
	
	/**
	 * @brief Closes gripper.
	 * Waits till move completed.
	 */
	bool closeWait();
	
private:
	// connection information
	serial::SerialPort* _port;
	unsigned char _id;
	
	// flags
	bool _connected; // if connection has already been done
	bool _ok;
	bool _referenced;
	
	// gripper state
	float _position;
	float _velocity;
	float _current;
	volatile unsigned short _status;
	unsigned char _errorCode;
};

}
}

#endif // _EVG55_HPP
