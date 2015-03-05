#include "MCSProtocol.hpp"

#include <iostream>

using namespace std;
using namespace evg55::mcsprotocol;
using namespace evg55::serial;

bool MCSProtocol::send(SerialPort* port, const Command& command) {
	char buf[command.size()];
	for (unsigned i = 0; i < command.size(); ++i) {
		buf[i] = command[i];
	}
	
	port->clean();
	return port->write(buf, command.size());
}

bool MCSProtocol::receive(SerialPort* port, Response& response) {
	// create and clean a buffer for incoming data
	const int BUF_LEN = 256;
	char buf[BUF_LEN];
	for (int i = 0; i < BUF_LEN; buf[i++] = 0);
	
	// read incoming message
	port->read(buf, BUF_LEN, 50);
	port->clean();
	
	// copy message to response
	size_t msgLength = 5 + static_cast<unsigned char>(buf[2]);
	ByteVector message;
	for (size_t i = 0; i < msgLength && i < BUF_LEN; ++i) {
		message.push_back(buf[i]);
	}
	
	response = Response(message);
	
	return response.isOk();
}
