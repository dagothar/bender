#ifndef _RWHW_SERIAL_PORT_HPP
#define _RWHW_SERIAL_PORT_HPP

#include <string>
#include <rwhw/serialport/SerialPort.hpp>
#include "SerialPort.hpp"



/**
 * Implements serial port interface using RWHW SerialPort
 */
class RWHWSerialPort: public SerialPort {	
public:
	/**
	 * @brief Constructor.
	 */
	RWHWSerialPort();
	
	//! Destructor.
	virtual ~RWHWSerialPort();
	
	// inherited from SerialPort
	virtual bool open(const std::string& name, unsigned baudrate);

	virtual void close();

	virtual bool write(const char* buf, unsigned n);

	virtual bool read(char* buf, unsigned n);

	virtual bool read(char* buf, unsigned n, unsigned timeout);
	
	virtual void clean();
	
private:
	rwhw::SerialPort::Ptr _port;
};

#endif // _RWHW_SERIAL_PORT_HPP
