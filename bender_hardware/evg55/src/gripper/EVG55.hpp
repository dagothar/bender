#ifndef _EVG55_HPP
#define _EVG55_HPP

#include <serial/SerialPort.hpp>

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
		ErrorSoftLow = 0x0d5
		//TODO: add all flags
	};
	
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
	 * @return \b true if connections succesful, \b false otherwise
	 */
	bool connect(serial::SerialPort* port, unsigned char id);
	
	/**
	 * @brief Returns \b true if gripper is connected.
	 */
	bool isConnected() const;
	
	/* GET INFO */
	/**
	 * @brief Get gripper state.
	 * Sends a GetState command to the gripper module and waits for response.
	 * Updates state flags, and gripper information (position, velocity, current, error code).
	 */
	unsigned short getState();
	
	/**
	 * @brief Returns gripper's position.
	 * Call getState() to update the current info first!
	 */
	float getPosition() const;
	
	/**
	 * @brief Returns gripper's velocity.
	 * Call getState() to update the current info first!
	 */
	float getVelocity() const;
	
	/**
	 * @brief Returns gripper's current.
	 * Call getState() to update the current info first!
	 */
	float getCurrent() const;
	
	/* COMMAND API */
	/**
	 * @brief Reference gripper.
	 * Attempts to reference the gripper.
	 * @return \b true if referenced.
	 */
	bool home();

private:
	serial::SerialPort* _port;
	unsigned char _id;
	
	/*
	volatile unsigned short _status;
	volatile bool _connected;
	volatile bool _referenced;
	volatile bool _error;
	volatile unsigned short _errorCode;
	mutable volatile bool _newInfo; // set by status thread when receiving new info
	volatile float _position;
	
	mutable boost::mutex _statusMutex;
	boost::thread _statusThread;*/
};

}
}

#endif // _EVG55_HPP
