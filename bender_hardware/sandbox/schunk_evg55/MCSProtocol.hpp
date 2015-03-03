#ifndef _MCS_PROTOCOL_HPP
#define _MCS_PROTOCOL_HPP

#include "SerialPort.hpp"

#include <vector>

/**
 * Defines Schunk Motion Control protocol
 */
class MCSProtocol {
public:
	//! Defines important byte positions within a packet
	enum Index {
		IndexHeader = 0,
		IndexModuleId = 1,
		IndexDlen = 2,
		IndexCommand = 3,
		IndexData = 4
	};
	
	//! Defines available commands
	enum Command {
		CommandAcknowledge = 0x8b,
		CommandStop = 0x91,
		CommandGetState = 0x95,
		CommandReference = 0x92,
		CommandMoveCurrent = 0xb3,
		CommandMovePosition = 0xb0,
		CommandError = 0x03,
		CommandFailed = 0x05,
		CommandOk1 = 0x4f,
		CommandOk2 = 0x4b,
		CommandMoveBlocked = 0x93,
		CommandPositionReached = 0x94
	};
	
	//! Possible message types.
	enum MessageType {
		MessageNone,
		MessageOk,
		MessageError,
		MessageFailed,
		MessageMoveBlocked,
		MessagePositionReached,
		MessageOther
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
	 * @param module [in] module id
	 * @param data [in] contents of the packet to wrap
	 * @return a properly formatted packet, ready to be sent.
	 */
	static Packet makePacket(unsigned char module, Data data);
	
	/**
	 * @brief Sends a ready packet to the module.
	 * Appends neccesary headers (id, dlen) + calculates CRC checksum.
	 * @param port [in] serial port to use
	 * @param packet [in] a packet to send
	 * @return \b true if succesful.
	 */
	static bool send(SerialPort* port, const Packet& packet);
	
	/**
	 * @brief Reads a packet received from a module.
	 * @param port [in] serial port to use
	 * @param message [out] a received message
	 * @return \b true if succesful.
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
	 * @brief Makes a reference command packet.
	 */
	static Packet makeReferenceCommand(unsigned char id);
	
	/**
	 * @brief Makes a 'get state' command packet.
	 */
	static Packet makeGetStateCommand(unsigned char id);
	
	/**
	 * @brief Makes a 'move to position' command packet.
	 */
	static Packet makeMovePositionCommand(unsigned char id, float pos);
	
	/**
	 * @brief Makes a 'move current' command packet.
	 */
	static Packet makeMoveCurrentCommand(unsigned char id, float cur);
};

#endif // _MCS_PROTOCOL_HPP
