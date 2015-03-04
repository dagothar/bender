#ifndef _SERIAL_PORT_HPP
#define _SERIAL_PORT_HPP

#include <string>



/**
 * Interface for serial port implementations.
 */
class SerialPort {	
public:
	//! Destructor.
	virtual ~SerialPort() {}

	/**
	 * @brief Opens and initializes serial port with given name and baudrate.
	 */
	virtual bool open(const std::string& name, unsigned baudrate) = 0;
	
	/**
	 * @brief Closes serial port connection.
	 */
	virtual void close() = 0;
	
	/**
	 * @brief Writes data to the serial port.
	 */
	virtual bool write(const char* buf, unsigned n) = 0;
	
	/**
	 * @brief Reads data from the serial port (waits for all data).
	 */
	virtual bool read(char* buf, unsigned n) = 0;
	
	/**
	 * @brief Reads data from the serial port (uses timeout).
	 */
	virtual bool read(char* buf, unsigned n, unsigned timeout) = 0;
	
	/**
	 * @brief Cleans all waiting data.
	 */
	virtual void clean() = 0;
};

#endif // _SERIAL_PORT_HPP
