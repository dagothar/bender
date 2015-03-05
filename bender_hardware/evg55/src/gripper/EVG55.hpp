#ifndef _EVG55_HPP
#define _EVG55_HPP

#include <boost/thread.hpp>
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
	 * @brief Disconnects gripper module.
	 * Stops listening thread.
	 */
	void disconnect();
	
	/* GET INFO */
	/**
	 * @brief Returns \b true if gripper is connected.
	 */
	bool isConnected() const;
	
	/**
	 * @brief Returns \b true if gripper is referenced.
	 */
	bool isReferenced() const;
	
	/**
	 * @brief Returns \b true if gripper is moving.
	 */
	bool isMoving() const;
	
	/**
	 * @brief Returns gripper's position.
	 */
	float getPosition() const;
	
	/**
	 * @brief Returns gripper's velocity.
	 */
	float getVelocity() const;
	
	/**
	 * @brief Returns gripper's current.
	 */
	float getCurrent() const;
	
	/* COMMAND API */
	/**
	 * @brief Reference gripper.
	 * Attempts to reference the gripper.
	 */
	void home();

private:
	//! A function listening to gripper status.
	void listenerFunc();
	
	serial::SerialPort* _port;
	unsigned char _id;
	
	mutable boost::mutex _mtx;
	boost::thread _listenerThread; // gripper listening thread
	
	volatile bool _connected;
	volatile bool _referenced;
	volatile float _position;
	volatile float _velocity;
	volatile float _current;
	
	/*
	volatile unsigned short _status;
	volatile bool _connected;
	volatile bool _referenced;
	volatile bool _error;
	volatile unsigned short _errorCode;
	mutable volatile bool _newInfo; // set by status thread when receiving new info
	volatile float _position; */
};

}
}

#endif // _EVG55_HPP
