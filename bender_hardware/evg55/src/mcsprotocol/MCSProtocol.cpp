#include "MCSProtocol.hpp"

using namespace evg55::mcsprotocol;
using namespace evg55::serial;

bool MCSProtocol::send(SerialPort* port, const Command& command) {
	char buf[command.size()];
	for (unsigned i = 0; i < command.size(); ++i) {
		buf[i] = command[i];
	}
	
	return port->write(buf, command.size());
}

bool MCSProtocol::receive(SerialPort* port, Response& message) {
	return true;
}
