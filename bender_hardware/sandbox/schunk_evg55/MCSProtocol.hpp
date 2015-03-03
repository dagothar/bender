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
		CommandAcknowledge = 0x8b,
		CommandStop = 0x91,
		CommandGetState = 0x95,
		CommandReference = 0x92,
		CommandMoveCurrent = 0xb3,
		CommandMovePosition = 0xb0
	};
	
	//! Possible message types.
	enum MessageType {
		MessageOk,
		MessageError,
		MessageFailed,
		MessageMoveBlocked,
		MessageReachedPosition
	};
	
	//! Data type.
	typedef std::vector<unsigned char> Data;
	
	//! Packet type.
	typedef std::vector<unsigned char> Packet;
	
	//! A type for module to master response.
	struct Message {
		MessageType messageType;
		unsigned char moduleId;
		Data data;
	};

public:
	/**
	 * @brief Print packet.
	 */
	static void printPacket(const MCSProtocol::Packet& msg);
	
	/**
	 * @brief Creates a packet from data.
	 * Prepends 0x05 M->S header, module id and dlen,
	 * and appends CRC sum.
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
	static bool receive(SerialPort* port, Message& message);
	
	//! Makes data by converting a float type parameter into a string of bytes.
	static Data makeData(float value);
	
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
	 * @brief Executes stop command.
	 */
	static bool stopCmd(SerialPort* port, unsigned char id);
	
	/**
	 * @brief Executes error acknowledgement command.
	 * Clears error state of the module.
	 */
	static bool ackCmd(SerialPort* port, unsigned char id);
	
	/**
	 * @brief Executes homing command.
	 */
	static bool homeCmd(SerialPort* port, unsigned char id);
	
	/**
	 * @brief Executes move to position.
	 * @param pos [in] position in mm.
	 */
	static bool movePositionCmd(SerialPort* port, unsigned char id, float pos);
	
	/**
	 * @brief Executes move with given current.
	 * @param cur [in] current
	 */
	static bool moveCurrentCmd(SerialPort* port, unsigned char id, float cur);
};

#endif // _MCS_PROTOCOL_HPP
