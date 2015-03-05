#ifndef _MCSPROTOCOL_MCSPROTOCOL_HPP
#define _MCSPROTOCOL_MCSPROTOCOL_HPP

#include <serial/SerialPort.hpp>
#include "Command.hpp"
#include "Response.hpp"

namespace evg55 {
namespace mcsprotocol {

/**
 * Defines Schunk Motion Control protocol
 */
class MCSProtocol {
public:
	/**
	 * @brief Sends a command to the module.
	 * @param port [in] serial port to use
	 * @param command [in] a packet to send
	 * @return \b true if succesful.
	 */
	static bool send(serial::SerialPort* port, const Command& command);
	
	/**
	 * @brief Reads a packet received from a module.
	 * @param port [in] serial port to use
	 * @param message [out] a received message
	 * @return \b true if succesful.
	 */
	static bool receive(serial::SerialPort* port, Response& response);
};

}
}

#endif // _MCSPROTOCOL_MCSPROTOCOL_HPP
