#ifndef _MCS_PROTOCOL_HPP
#define _MCS_PROTOCOL_HPP

#include "SerialPort.hpp"

#include <vector>

/**
 * Defines Schunk Motion Control protocol
 */
class MCSProtocol {
public:
	//! Defines available commands
	enum Command {
		GetState = 0x95,
		Reference = 0x92,
		MoveCurrent = 0xb3,
		MovePosition = 0xb0
	};
	
	//! Data type.
	typedef std::vector<unsigned char> Data;
	
	//! Packet type.
	typedef std::vector<unsigned char> Packet;

public:
	/**
	 * @brief Print packet.
	 */
	static void printPacket(const MCSProtocol::Packet& msg);
	
	/**
	 * @brief Creates a packet from data.
	 */
	static Packet makePacket(unsigned char module, Data data);
	
	/**
	 * @brief Sends a ready packet to the module.
	 * Appends neccesary headers (id, dlen) + calculates CRC checksum.
	 */
	static bool send(SerialPort* port, const Packet& packet);
	
	/**
	 * @brief Reads a packet received from a module.
	 */
	static bool receive(SerialPort* port, unsigned char id, Packet& packet);
	
	/**
	 * @brief Sends a ping to a module connected to the port.
	 * @return \b true if module with id found.
	 */
	static bool ping(SerialPort* port, unsigned char id);
	
	/**
	 * @brief Check if the response packet contains OK message.
	 * @return \b true if OK
	 */
	static bool isOk(const Packet& packet);
	
	/**
	 * @brief Executes homing command.
	 */
	static bool homeCmd(SerialPort* port, unsigned char id);
	
	/**
	 * @brief Executes move to position.
	 * @param pos [in] position in mm.
	 */
	static bool movePositionCmd(SerialPort* port, unsigned char id, float pos);
	
	//! Makes data for parameterless command.
	static Data makeData(Command cmd);
	
	//! Makes data for a single parameter command with float.
	static Data makeData(Command cmd, float value);
};

#endif // _MCS_PROTOCOL_HPP
