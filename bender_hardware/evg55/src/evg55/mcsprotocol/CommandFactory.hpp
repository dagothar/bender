#ifndef _MCSPROTOCOL_COMMANDFACTORY_HPP
#define _MCSPROTOCOL_COMMANDFACTORY_HPP

#include "Command.hpp"

namespace evg55 {
namespace mcsprotocol {

/**
 * Creates Schunk Motion protocol commands
 */
class CommandFactory {
public:
	/**
	 * @brief Make module reboot command.
	 */
	static Command makeRebootCommand(Byte id);
	
	/**
	 * @brief Creates a Reference command.
	 */
	static Command makeReferenceCommand(Byte id);
	
	/**
	 * @brief Creates an error acknowledgement command.
	 */
	static Command makeAcknowledgementCommand(Byte id);
	
	/**
	 * @brief Creates a stop command.
	 */
	static Command makeStopCommand(Byte id);
	
	/**
	 * @brief Creates a toggle impulse messages command.
	 */
	static Command makeToggleImpulseMessagesCommand(Byte id);
	
	/**
	 * @brief Creates a GetState command.
	 * @param id [in] module id
	 * @param interval [in] interval for sending state info from module (in seconds)
	 */
	static Command makeGetStateCommand(Byte id, float interval=0.0);
	
	/**
	 * @brief Creates a SetTargetVelocity command.
	 * @param id [in] module id
	 * @param vel [in] velocity limit (in mm/s)
	 */
	static Command makeSetTargetVelocityCommand(Byte id, float vel);
	
	/**
	 * @brief Creates a SetTargetVelocity command.
	 * @param id [in] module id
	 * @param cur [in] current limit (in A)
	 */
	static Command makeSetTargetCurrentCommand(Byte id, float cur);
	
	/**
	 * @brief Creates a MovePosition command.
	 * @param id [in] module id
	 * @param pos [in] gripper opening (in mm)
	 */
	static Command makeMovePositionCommand(Byte id, float pos);
	
	/**
	 * @brief Creates a MoveGrip command.
	 * Executes a gripping movement with current limit.
	 * @param id [in] module id
	 * @param cur [in] current limit (in A)
	 */
	static Command makeMoveGripCommand(Byte id, float cur);
};

}
}

#endif // _MCSPROTOCOL_COMMANDFACTORY_HPP
