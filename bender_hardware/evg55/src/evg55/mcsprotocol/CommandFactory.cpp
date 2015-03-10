#include "CommandFactory.hpp"

using namespace evg55::mcsprotocol;

Command CommandFactory::makeRebootCommand(Byte id) {
	Command cmd(id, Command::Reboot);
	
	return cmd;
}

Command CommandFactory::makeReferenceCommand(Byte id) {
	Command cmd(id, Command::Reference);
	
	return cmd;
}

Command CommandFactory::makeAcknowledgementCommand(Byte id) {
	Command cmd(id, Command::Acknowledge);
	
	return cmd;
}

Command CommandFactory::makeStopCommand(Byte id) {
	Command cmd(id, Command::Stop);
	
	return cmd;
}

Command CommandFactory::makeToggleImpulseMessagesCommand(Byte id) {
	Command cmd(id, Command::ToggleImpulseMessages);
	
	return cmd;
}
	
Command CommandFactory::makeGetStateCommand(Byte id, float interval) {
	Command cmd(id, Command::GetState);
	
	if (interval > 0.0) {
		cmd.addParameter(interval);
	}

	return cmd;
}

Command CommandFactory::makeSetTargetVelocityCommand(Byte id, float vel) {
	Command cmd(id, Command::SetTargetVelocity);
	
	cmd.addParameter(vel);

	return cmd;
}

Command CommandFactory::makeSetTargetCurrentCommand(Byte id, float cur) {
	Command cmd(id, Command::SetTargetCurrent);
	
	cmd.addParameter(cur);

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
