#include "MCSProtocol.hpp"

#include <iostream>
#include <unistd.h>

#define LOG

using namespace std;
using namespace evg55::mcsprotocol;
using namespace evg55::serial;

bool MCSProtocol::send(SerialPort* port, const Command& command) {
	char buf[command.size()];
	for (unsigned i = 0; i < command.size(); ++i) {
		buf[i] = command[i];
	}
	
#ifdef LOG
	cout << "Sending: " << command << endl;
#endif

	port->clean();
	return port->write(buf, command.size());
}

bool MCSProtocol::receive(SerialPort* port, Response& response) {
	/*create and clean a buffer for incoming data */
	const int BUF_LEN = 256;
	char buf[BUF_LEN];
	for (int i = 0; i < BUF_LEN; buf[i++] = 0);
	
	/* read header of the incoming message */
	port->read(buf, 3, 250);
	
	/* check if message has a proper header */
	if (buf[0] != 0x07) return false;
	
	/* check out dlen to see how many bytes more to read */
	size_t dlen = buf[2];
	
	/* read incoming message */
	port->read(buf+3, dlen+2, 250);
	
	/* copy message to response */
	size_t msgLength = 5 + dlen;
	ByteVector message;
	for (size_t i = 0; i < msgLength && i < BUF_LEN; ++i) {
		message.push_back(buf[i]);
	}
	
	response = Response(message);
	
#ifdef LOG
	cout << "Received: " << response << endl;
#endif
	
	return response.isOk();
}

bool MCSProtocol::ack(serial::SerialPort* port, const Command& command, Response& response, unsigned tries) {
	unsigned count = 0;

#ifdef LOG
	//cout << "ACK {" << endl;
#endif
	do {
		usleep(100);
		
		++count;
		if (!receive(port, response)) {
			continue;
		}
		
		//cout << response << endl;
		
		if (command.getCommand() == response.getCommand() && response.getDlen() != 2) { // check if the command matches, and is not an error message
			
#ifdef LOG
			cout << "}" << endl;
#endif

			return true;
		}
	} while(count < tries);
	
#ifdef LOG
	//cout << "}" << endl;
#endif
	
	return false;
}

bool MCSProtocol::emit(serial::SerialPort* port, const Command& command, Response& response, unsigned tries) {
	send(port, command);

	return MCSProtocol::ack(port, command, response, tries);
}
