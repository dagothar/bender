#include "CommandFactory.hpp"

using namespace evg55::mcsprotocol;

Command CommandFactory::makeReferenceCommand(Byte id) {
	Command cmd(id, Command::Reference);
	
	return cmd;
}

Command CommandFactory::makeAcknowledgementCommand(Byte id) {
	Command cmd(id, Command::Acknowledge);
	
	return cmd;
}
	
Command CommandFactory::makeGetStateCommand(Byte id, float interval) {
	Command cmd(id, Command::GetState);
	
	cmd.addParameter(interval);

	return cmd;
}

Command CommandFactory::makeMovePositionCommand(Byte id, float pos) {
	Command cmd(id, Command::MovePosition);
	
	cmd.addParameter(pos);

	return cmd;
}

Command CommandFactory::makeMoveGripCommand(Byte id, float cur) {
	Command cmd(id, Command::MoveGrip);
	
	cmd.addParameter(cur);

	return cmd;
}
