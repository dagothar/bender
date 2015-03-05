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
	 * @return \b true if connection succesful, \b false otherwise
	 */
	bool connect(serial::SerialPort* port, unsigned char id);
	
	/**
	 * @brief Disconnects gripper module.
	 */
	void disconnect();
	
	/* GET INFO */
	/**
	 * @brief Polls gripper module for new information.
	 */
	bool poll();
	
	/**
	 * @brief Returns gripper state bytes.
	 * @param doPoll [in] if \b true poll() is called, otherwise uses stored information
	 */
	unsigned short getStatus(bool doPoll = false);
	
	/**
	 * @brief Returns \b true if gripper is connected.
	 * @param doPoll [in] if \b true poll() is called, otherwise uses stored information
	 */
	bool isConnected(bool doPoll = true);
	
	/**
	 * @brief Returns \b true if gripper is in OK state, and \b false if there is any kind of error.
	 * @param doPoll [in] if \b true getState() is called, otherwise uses stored information.
	 */
	bool isOk(bool doPoll = true);
	
	/**
	 * @brief Returns \b true if gripper is referenced.
	 * @param doPoll [in] if \b true getState() is called, otherwise uses stored information
	 */
	bool isReferenced(bool doPoll = false);
	
	/**
	 * @brief Returns \b true if gripper is moving.
	 * @param doPoll [in] if \b true getState() is called, otherwise uses stored information
	 */
	bool isMoving(bool doPoll = false);
	
	/**
	 * @brief Returns gripper's position.
	 * @param doPoll [in] if \b true getState() is called, otherwise uses stored information
	 */
	float getPosition(bool doPoll = false);
	
	/**
	 * @brief Returns gripper's velocity.
	 */
	float getVelocity() const;
	
	/**
	 * @brief Returns gripper's current.
	 */
	float getCurrent() const;
	
	/**
	 * @brief Returns gripper error code.
	 * @param doPoll [in] if \b true getState() is called, otherwise uses stored information
	 */
	unsigned char getErrorCode(bool doPoll = false);
	
	/* COMMAND API */
	/**
	 * @brief Sends reference gripper command.
	 * Attempts to reference the gripper.
	 */
	void home();
	
	/**
	 * @brief Sends move gripper to position command.
	 * @param pos [in] position in mm
	 */
	void move(float pos);
	
	/**
	 * @brief Send move to position command, and wait till move complete.
	 * @param pos [in] position in mm
	 * @return \b true if position reached succesfully
	 */
	bool moveWait(float pos);
	
private:
	//! A function listening to gripper status.
	void listenerFunc();
	
	serial::SerialPort* _port;
	unsigned char _id;
	
	bool _connected;
	bool _ok;
	bool _referenced;
	float _position;
	float _velocity;
	float _current;
	volatile unsigned short _status;
	unsigned char _errorCode;
};

}
}

#endif // _EVG55_HPP
