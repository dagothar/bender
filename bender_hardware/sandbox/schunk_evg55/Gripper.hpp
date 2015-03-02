#ifndef _GRIPPER_HPP
#define _GRIPPER_HPP

#include <string>
#include "SerialPort.hpp"



/**
 * Gripper interface.
 */
class Gripper {
public:
	//! Destructor.
	virtual ~Gripper() {}
	
	/**
	 * @brief Connects to the gripper module via RS232 port.
	 * @param name [in] port name
	 * @return \b true if succesful, \b false if not succesful
	 */
	virtual bool connect(SerialPort* port) = 0;
	
	/**
	 * @brief Returns \b true if the gripper module is connected succesfully.
	 */
	virtual bool isConnected() const = 0;
	
	/**
	 * @brief Disconnects gripper module.
	 */
	virtual void disconnect() = 0;
	
	/**
	 * @brief Executes referencing command.
	 */
	virtual bool home() = 0;
	
	/**
	 * @brief Executes opening movement.
	 */
	virtual bool open() = 0;
	
	/**
	 * @brief Executes closing movement.
	 */
	virtual bool close() = 0;
	
	/**
	 * @brief Stops gripper movement.
	 */
	virtual void stop() = 0;
	
	/**
	 * @brief Returns gripper configuration.
	 */
	virtual double getConfiguration() const = 0;
	
	/**
	 * @brief Sets gripper configuration.
	 */
	virtual bool setConfiguration(double q) = 0;
	
	/**
	 * @brief Returns gripper status.
	 */
	virtual unsigned getStatus() const = 0;
};

#endif // _GRIPPER_HPP
