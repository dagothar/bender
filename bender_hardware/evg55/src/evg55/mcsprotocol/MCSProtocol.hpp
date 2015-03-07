#ifndef _MCSPROTOCOL_MCSPROTOCOL_HPP
#define _MCSPROTOCOL_MCSPROTOCOL_HPP

#include <evg55/serial/SerialPort.hpp>
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
	
	/**
	 * @brief Wait for a command acknowledgement.
	 * Command acknowledgement contains command code.
	 * @param port [in] serial port to use
	 * @param command [in] a command for which to wait for acknowledgement
	 * @param response [out] a received response
	 * @param tries [in] number of tries
	 * @return \b true if succesful.
	 */ 
	static bool ack(serial::SerialPort* port, const Command& command, Response& response, unsigned tries=10);
	
	/**
	 * @brief Send a command and wait for acknowledgement.
	 * @param port [in] serial port to use
	 * @param command [in] a packet to send
	 * @param tries [in] number of tries
	 * @return \b true if succesful.
	 */
	static bool emit(serial::SerialPort* port, const Command& command, unsigned tries=10);
};

}
}

#endif // _MCSPROTOCOL_MCSPROTOCOL_HPP
