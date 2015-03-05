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
	 */
	static Command makeGetStateCommand(Byte id);
};

}
}

#endif // _MCSPROTOCOL_COMMANDFACTORY_HPP
