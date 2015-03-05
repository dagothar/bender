#include "CommandFactory.hpp"

using namespace mcsprotocol;

Command CommandFactory::makeReferenceCommand(Byte id) {
	Command cmd(id);
	
	cmd.setCommand(Command::Reference);
	
	return cmd;
}
	
Command makeGetStateCommand(Byte id) {
	Command cmd(id);
	
	cmd.setCommand(Command::GetState);
	
	return cmd;
}
