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
	 * @brief Creates a Reference command.
	 */
	static Command makeReferenceCommand(Byte id);
	
	/**
	 * @brief Creates a GetState command.
	 * @param id [in] module id
	 * @param interval [in] interval for sending state info from module (in seconds)
	 */
	static Command makeGetStateCommand(Byte id, float interval=0.0);
	
	/**
	 * @brief Creates a MovePosition command.
	 * @param id [in] module id
	 * @param pos [in] gripepr opening (in dm?)
	 */
	static Command makeMovePositionCommand(Byte id, float pos);
};

}
}

#endif // _MCSPROTOCOL_COMMANDFACTORY_HPP
