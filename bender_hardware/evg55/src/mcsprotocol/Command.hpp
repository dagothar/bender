#ifndef _MCSPROTOCOL_COMMAND_HPP
#define _MCSPROTOCOL_COMMAND_HPP

#include "Packet.hpp"

namespace mcsprotocol {

/**
 * Represent a command sent to MCS module.
 */
class Command: public Packet {
public:
	//! Command bytes.
	enum Commands {
		Reference = 0x92,
		MovePosition = 0xb0,
		MoveVelocity = 0xb5,
		MoveCurrent = 0xb3,
		MoveGrip = 0xb7,
		Stop = 0x91,
		EmergencyStop = 0x90,
		GetState = 0x95,
		Reboot = 0xe0,
		Acknowledge = 0x8b
	};
public:
	/**
	 * @brief Constructor.
	 * Creates an empty command to be sent to module with \b id.
	 */
	Command(Byte id);
	
	//! Destructor.
	virtual ~Command();
	
protected:

private:
};

}

#endif // _COMMAND_HPP
